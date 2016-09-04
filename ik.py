
import numpy as np

from collections import namedtuple

from snap.math import *

from snap import viewer, gl

# TODO joint nullspace (subclass)

class Body(namedtuple('Body', 'mass inertia dim name dofs')):

    def __hash__(self):
        return id(self)

    def __str__(self):
        return 'Body({0})'.format(self.name)

    def inertia_tensor(self):
        return np.identity(6)


    def draw(self):
        with gl.frame(self.dofs):
            gl.glScale(* (self.dim  /2))
            gl.cube()
    

            
class Joint(namedtuple('Joint', 'parent child name nullspace')):

    def __hash__(self):
        return id(self)

    def __str__(self):
        return 'Joint({0}, {1})'.format(self.parent[0].name, self.child[0].name)


    def compliance_matrix(self):
        rows, cols = self.nullspace.shape
        return np.zeros( (rows, rows ) )
    
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
        twist[3:] = r.center
        
        return -self.nullspace.dot(twist)
        
    
class Constraint(namedtuple('Constraint', 'body local target stiffness')):

    def __hash__(self):
        return id(self)


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

                    print(str(e))
                    
                    v.in_edges.append(e)
                    other.out_edges.append(e)

                    assert len(other.out_edges) == 1, "cyclic graph"
                    res += postfix(other)

            return res + [v]

        # print('start:', start.data.name)
        
        res = postfix(start)
        assert len(res) == len(self.vertices)

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
            e = Edge(data[c.body], v)

            graph.edges.append(e)

        return data


    def fill_matrix(self, matrix, graph):
        
        for v in graph.vertices:

            if type(v.data) is Body:
                matrix[v] = v.data.inertia_tensor()

                # TODO fill forces/momentum here

                
            if type(v.data) is Joint:
                assert len(v.in_edges) + len(v.out_edges) == 2, 'is the graph oriented ?'

                # edges from v
                edges = v.out_edges + v.in_edges

                # parent/child bodies
                p = v.data.parent[0]
                c = v.data.child[0]

                matrix[v] = v.data.compliance_matrix()
                
                Jp, Jc = v.data.jacobian()

                for e in edges:
                    
                    # was the edge reversed during orientation?
                    transpose = (e.src != v)

                    parent = (e.src.data is p or e.dst.data is p)

                    block = Jp if parent else Jc
                    matrix[e] = block.T if transpose else block
                    

            # TODO constraints

    def fill_vector(self, vector, graph):

        for v in graph.vertices:

            if type(v.data) is Body:
                # TODO fill forces/momentum
                vector[v] = np.zeros( 6 )
                
            if type(v.data) is Joint:
                vector[v] = v.data.error()

                
    def step(self, vector):

        for k, v in vector.items():

            if type(k.data) is Body:

                delta = Rigid3()
                
                delta.orient = Quaternion.exp(v[:3])
                delta.center = v[3:]

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

    def hinge(*args, **kwargs):
        kwargs.setdefault('name', 'unnamed joint')

        axis = kwargs.pop('axis', vec(1, 0, 0))

        nullspace = np.zeros( (3, 6) )
        nullspace[:, 3:] = np.identity(3)
        
        kwargs['nullspace'] = nullspace
        return Joint(*args, **kwargs)
    
    
    # bodies
    size = 1
    
    head = body(name = 'head', dim = size * vec(1, 1, 1) )
    trunk = body(name = 'trunk', dim = size * vec(2, 3, 1) )
    
    # larm = body(name = 'larm')
    # rarm = body(name = 'rarm')    

    # lforearm = body(name = 'lforearm')
    # rforearm = body(name = 'rforearm')    

    bodies = [head, trunk] #, larm, rarm, lforearm, rforearm]

    # joints
    neck = hinge( (trunk, Rigid3(center = vec(0, trunk.dim[1] / 2, 0))),
                  (head, Rigid3(center = vec(0, -head.dim[1] / 2, 0))),
                  name = 'neck')

    # lshoulder = hinge( (trunk, Rigid3()), (larm, Rigid3()), name = 'lshoulder' )

    # rshoulder = hinge( (trunk, Rigid3()), (rarm, Rigid3()), name = 'rshoulder' )

    # relbow = hinge( (rarm, Rigid3()), (rforearm, Rigid3()), name = 'relbow' )

    # lelbow = hinge( (larm, Rigid3()), (lforearm, Rigid3()), name = 'lelbow')
    
    joints = [neck] #, lshoulder, rshoulder, relbow, lelbow]
    
    return Skeleton(bodies, joints, [])


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

        for k, v in vector.items():
            print(str(k), v)
        
        # solve
        factor(matrix, forward)
        solve(vector, matrix, forward)
        
        # step
        skeleton.step(vector)
        
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

