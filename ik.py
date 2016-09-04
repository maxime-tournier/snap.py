
import numpy as np

from collections import namedtuple

from snap.math import *

from snap import viewer, gl

# TODO joint nullspace (subclass)

class Body(object):

    def __hash__(self):
        return id(self)

    def __str__(self):
        return 'Body({0})'.format(self.name)


    def draw(self):
        with gl.frame(self.dofs):
            gl.glScale(* (self.dim  /2))
            gl.cube()
    

    def __init__(self, **kwargs):

        self.dim = kwargs.get('dim', np.ones(3))
        self.dofs = kwargs.get('dofs', Rigid3())
        self.name = kwargs.get('name', 'unnamed body')
        
        volume = self.dim[0] * self.dim[1] * self.dim[2]

        mass = volume

        dim2 = self.dim * self.dim
        sum_dim2 = sum(dim2)

        self.mass = mass
        self.inertia = mass / 12.0 * (sum_dim2 - dim2)

        self.inertia_tensor = np.zeros( (6, 6) )
        
        self.inertia_tensor[:3, :3] = np.diag(self.inertia)
        self.inertia_tensor[3:, 3:] = mass * np.identity(3)

        
class Joint(namedtuple('Joint', 'parent child name nullspace compliance')):

    def __hash__(self):
        return id(self)

    def __str__(self):
        return 'Joint({0}, {1})'.format(self.parent[0].name, self.child[0].name)


    def jacobian(self):
        '''constraint jacobian'''
        
        p = self.parent[0].dofs * self.parent[1]
        c = self.child[0].dofs * self.child[1]        
        
        r = p.inv() * c
        
        dp = self.parent[1].inv().Ad()
        dc = self.child[1].inv().Ad()

        return -self.nullspace.dot(r.inv().Ad() * dp), self.nullspace.dot(dc)

    def error(self):
        '''constraint violation'''

        p = self.parent[0].dofs * self.parent[1]
        c = self.child[0].dofs * self.child[1]        
        
        r = p.inv() * c

        twist = np.zeros( 6 )
        
        twist[:3] = r.orient.log()
        twist[3:] = r.orient.inv()(r.center)
        
        return -self.nullspace.dot(twist)
        
    
class Constraint(namedtuple('Constraint', 'body local target stiffness')):
    '''an ik constraint'''
    
    def __hash__(self):
        return id(self)

    def jacobian(self):
        # TODO this is constant, optimize
        return Rigid3(center = self.local).inv().Ad()[3:, :]

    def error(self):
        return self.body.dofs.orient.inv()(self.target - self.body.dofs(self.local))
    

    def compliance(self):
        return 1.0 / self.stiffness * np.identity(3)

    
class Vertex(namedtuple('Vertex', 'data in_edges out_edges')):

    def __hash__(self): return id(self)

    def __str__(self):
        return self.data.name

    
class Edge(namedtuple('Edge', 'src dst data')):

    def __hash__(self): return id(self)

    def __str__(self):
        return '({0} -> {1})'.format( str(self.src), str(self.dst) )

    
class Graph(namedtuple('Graph', 'vertices edges')):


    def orient(self, start):
        marked = set()

        edges = {}

        for e in self.edges:
            edges.setdefault(e.dst, []).append(e)
            edges.setdefault(e.src, []).append(e)            

        def postfix(v):

            res = []

            for e in edges[v]:
                
                if e not in marked:
                    marked.add(e)
                    # TODO remove e from edges[other] ?
                    
                    assert e.src != e.dst

                    other = e.src if e.src != v else e.dst

                    # reorient edge
                    e = Edge(other, v, e.data)

                    v.in_edges.append(e)
                    other.out_edges.append(e)

                    assert len(other.out_edges) == 1, "cyclic graph"
                    res += postfix(other)

            return res + [v]

        # print('start:', start.data.name)
        
        res = postfix(start)
        assert len(res) == len(self.vertices), 'non-connected graph'

        return res
    

class Skeleton(namedtuple('Skeleton', 'bodies joints constraints')):

    def draw(self):
        for b in self.bodies:  b.draw()
    
    def update(self, graph):

        data = { }

        def add_vertex(x):
            v = Vertex(x, in_edges = [], out_edges = [])
            graph.vertices.append(v)
            data[x] = v
            return v
        
        
        for b in self.bodies:
            add_vertex(b)

            
        for j in self.joints:
            v = add_vertex(j)

            # constraint row blocks
            e1 = Edge( v, data[j.parent[0]], j)
            e2 = Edge( v, data[j.child[0]], j)
            
            graph.edges.append( e1 )
            graph.edges.append( e2 )

            
        for c in self.constraints:
            v = add_vertex(c)
            e = Edge(v, data[c.body], c)

            graph.edges.append(e)

        return data


    def fill_matrix(self, matrix, graph):
        
        for v in graph.vertices:

            if type(v.data) is Body:
                matrix[v] = v.data.inertia_tensor

                # TODO fill forces/momentum here

                
            if type(v.data) is Joint:
                assert len(v.in_edges) + len(v.out_edges) == 2, 'is the graph oriented ?'

                # edges from v
                edges = v.out_edges + v.in_edges

                # parent/child bodies
                p = v.data.parent[0]
                c = v.data.child[0]

                matrix[v] = -v.data.compliance
                
                Jp, Jc = v.data.jacobian()

                for e in edges:
                    
                    # was the edge reversed during orientation?
                    transpose = (e.src != v)

                    parent = (e.src.data is p or e.dst.data is p)

                    block = Jp if parent else Jc
                    matrix[e] = block.T if transpose else block
                    

            if type(v.data) is Constraint:

                matrix[v] = -v.data.compliance()

                assert len(v.out_edges) == 1
                e = v.out_edges[0]
                
                matrix[e] = v.data.jacobian()
                

    def fill_vector(self, vector, graph):

        for v in graph.vertices:

            if type(v.data) is Body:
                # TODO fill forces/momentum
                vector[v] = np.zeros( 6 )
                
            if type(v.data) is Joint:
                vector[v] = v.data.error()

            if type(v.data) is Constraint:
                vector[v] = v.data.error()
                
    def step(self, vector, dt = 1):

        for k, v in vector.items():

            if type(k.data) is Body:

                delta = Rigid3()
                
                delta.orient = Quaternion.exp( dt * v[:3])
                delta.center = dt * v[3:]

                k.data.dofs[:] = k.data.dofs * delta
                
        
def factor(matrix, forward):

    for v in forward:

        mv = matrix[v]

        for e in v.in_edges:
            me = matrix[e]
            mv -= me.T.dot( matrix[e.src][0] ).dot(me)
        
        mv_inv = np.linalg.inv(mv)
        matrix[v] = (mv, mv_inv)

        for e in v.out_edges:
            me = matrix[e]
            me[:] = mv_inv.dot(me)


def solve(vector, matrix, forward):
    
    for v in forward:

        xv = vector[v]

        for e in v.in_edges:
            xv -= matrix[e].T.dot(vector[e.src])
            
    for v in reversed( forward ):
        xv = vector[v]

        xv[:] = matrix[v][1].dot(xv)
        
        for e in v.out_edges:
            xv -= matrix[e].dot( vector[e.dst] )


def make_skeleton():

    mass = 1
    inertia = np.ones(3)
    dim = np.ones(3)

    def body(**kwargs):
        kwargs.setdefault('name', 'unnamed body')

        kwargs.setdefault('mass', mass)
        kwargs.setdefault('inertia', inertia)
        kwargs.setdefault('dim', dim)
        
        kwargs.setdefault('dofs', Rigid3() )                        
        
        return Body(**kwargs)


    def joint(*args, **kwargs):
        kwargs.setdefault('name', 'unnamed joint')
        kwargs.setdefault('nullspace', np.identity(6))
        return Joint(*args, **kwargs)

    
    def spherical(*args, **kwargs):
        kwargs.setdefault('name', 'unnamed joint')

        axis = kwargs.pop('axis', vec(1, 0, 0))

        # nullspace = np.zeros( (3, 6) )
        # nullspace[:, 3:] = np.identity(3)
        

        nullspace = np.identity(6)
        kwargs['nullspace'] = nullspace

        compliance = kwargs.get('compliance', 1) * np.identity(6)
        compliance[3:, 3:] = 0

        kwargs['compliance'] = compliance
        
        return Joint(*args, **kwargs)
    
    
    
    # bodies
    size = 1
    
    head = body(name = 'head', dim = size * vec(1, 1, 1) )
    trunk = body(name = 'trunk', dim = size * vec(2, 3, 1) )
    
    larm = body(name = 'larm', dim = size * vec(0.5, 2, 0.5) )
    rarm = body(name = 'rarm', dim = size * vec(0.5, 2, 0.5) )    
    
    lforearm = body(name = 'lforearm', dim = size * vec(0.5, 2, 0.5))
    rforearm = body(name = 'rforearm', dim = size * vec(0.5, 2, 0.5))    

    bodies = [head, trunk, larm, rarm, lforearm, rforearm]

    # joints
    neck = spherical( (trunk, Rigid3(center = vec(0, 3 * trunk.dim[1] / 5, 0))),
                      (head, Rigid3(center = vec(0, -head.dim[1] / 2, 0))),
                      name = 'neck')

    lshoulder = spherical( (trunk, Rigid3(orient = Quaternion.exp( -math.pi / 4 * ez),
                                          center = vec(-trunk.dim[0] / 2,
                                                       trunk.dim[1] / 2,
                                                       0))),
                           (larm, Rigid3(center = vec(0, larm.dim[1] / 2, 0))),
                           name = 'lshoulder' )

    rshoulder = spherical( (trunk, Rigid3(orient = Quaternion.exp( math.pi / 4 * ez),
                                          center = vec(trunk.dim[0] / 2,
                                                       trunk.dim[1] / 2,
                                                       0))),
                           (rarm, Rigid3(center = vec(0, rarm.dim[1] / 2, 0))),
                           name = 'rshoulder' )

    relbow = spherical( (rarm, Rigid3(center = vec(0, -rarm.dim[1] / 2, 0))),
                        (rforearm, Rigid3(center = vec(0, rforearm.dim[1] / 2, 0))),
                        name = 'relbow' )
    
    lelbow = spherical( (larm, Rigid3(center = vec(0, -larm.dim[1] / 2, 0))),
                        (lforearm, Rigid3(center = vec(0, lforearm.dim[1] / 2, 0))),
                        name = 'lelbow')
    
    joints = [neck, lshoulder, rshoulder, relbow, lelbow]


    c = Constraint(lforearm, vec(0, -lforearm.dim[1] / 2, 0),
                   vec(1, 1, 1),
                   1e1)
    
    constraints = [c]
    
    
    return Skeleton(bodies, joints, constraints)


skeleton = make_skeleton()

graph = Graph([], [])


data = skeleton.update( graph )


forward = graph.orient( data[skeleton.bodies[1]] )


def solver():

    matrix = {}
    vector = {}

    while True:
        
        # assemble
        skeleton.fill_matrix(matrix, graph)
        skeleton.fill_vector(vector, graph)

        # solve
        factor(matrix, forward)
        solve(vector, matrix, forward)
        
        # step
        skeleton.step(vector, 1.0)
        
        yield
        
s = solver()
next(s)

        
def draw():

    gl.glColor(1, 1, 1)
    skeleton.draw()


def keypress(key):
    if key == ' ':
        next(s)
        return True


def animate():
    try:
        next(s)
    except StopIteration:
        import sys
        sys.exit(1)
        
    
viewer.run()
