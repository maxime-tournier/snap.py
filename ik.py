
import numpy as np

from collections import namedtuple

from snap.math import *

from snap import viewer, gl, tool

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


    def jacobian(self, pdofs = None, cdofs = None):
        '''constraint jacobian'''

        pdofs = pdofs if pdofs is not None else self.parent[0].dofs
        cdofs = cdofs if cdofs is not None else self.child[0].dofs        
        
        p = pdofs * self.parent[1]
        c = cdofs * self.child[1]        
        
        r = p.inv() * c
        
        dp = self.parent[1].inv().Ad()
        dc = self.child[1].inv().Ad()

        return -self.nullspace.dot(r.inv().Ad().dot(dp)), self.nullspace.dot(dc)

    
    def error(self):
        '''constraint violation'''

        p = self.parent[0].dofs * self.parent[1]
        c = self.child[0].dofs * self.child[1]        
        
        r = p.inv() * c

        twist = np.zeros( 6 )
        
        twist[:3] = r.orient.log()
        twist[3:] = r.orient.inv()(r.center) # translation in local coords, as velocity
        
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
        return (1.0 / self.stiffness) * np.identity(3)


    
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


    def fill_matrix(self, matrix, vector, graph, old, dt, **kwargs):

        gs = kwargs.get('gs', False)
        
        for v in graph.vertices:

            if type(v.data) is Body:
                m = matrix.setdefault(v, np.zeros( (6, 6) ) )
                # matrix[v] = v.data.inertia_tensor
                m += v.data.inertia_tensor
                
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

                # geometric stiffness
                if gs and v in vector:

                    # assert p in old
                    # assert c in old

                    # get constraint force
                    mu = -vector[v] / dt

                    # recover old jacobian
                    # Jp, Jc = v.data.jacobian(old[p], old[c])

                    
                    # old force, body-fixed
                    fp = Jp.T.dot(mu)

                    # in new frame
                    # fp = (old[p].inv() * p.dofs).Ad().T.dot(fp)
                    
                    op = v.data.parent[1]
                    xp = op.center

                    # apply on joint center
                    fp = Rigid3(center = xp).Ad().T.dot(fp)

                    # keep only translation part
                    fp = fp[3:]

                    # 
                    
                    # old force, body-fixed
                    fc = Jc.T.dot(mu)

                    # in new frame
                    # fc = (old[c].inv() * c.dofs).Ad().T.dot(fc)
                    
                    oc = v.data.child[1]
                    xc = oc.center

                    # apply on joint center
                    fc = Rigid3(center = xc).Ad().T.dot(fc)

                    # keep only translation part
                    fc = fc[3:]


                    for e in edges:
                        parent = (e.src.data is p or e.dst.data is p)

                        if parent:
                            vp = e.src if e.src.data is p else e.dst
                            assert vp.data is p
                            
                            mp = matrix.setdefault(vp, np.zeros( (6, 6) ) )

                            hfp = Quaternion.hat(fp)
                            hxp = Quaternion.hat(xp)
                            
                            block = ( hfp.dot(hxp) + hxp.dot(hfp) ) / 2
                            
                            mp[:3, :3] -= (dt * dt) * block
                            
                        else:
                            vc = e.src if e.src.data is c else e.dst
                            assert vc.data is c
                            
                            mc = matrix.setdefault(vc, np.zeros( (6, 6) ) )

                            hfc = Quaternion.hat(fc)
                            hxc = Quaternion.hat(xc)
                            
                            block = ( hfc.dot(hxc) + hxc.dot(hfc) ) / 2
                            
                            mc[:3, :3] -= (dt * dt) * block

                            
            if type(v.data) is Constraint:

                matrix[v] = -v.data.compliance()

                assert len(v.out_edges) == 1
                e = v.out_edges[0]
                
                matrix[e] = v.data.jacobian()

                # geometric stiffness
                if gs and v in vector:

                    # get constraint force
                    mu = -vector[v] / dt

                    # local coords
                    p = v.data.body
                    world = old[p].orient(mu)
                    fp = v.data.body.dofs.orient.inv()(world)
                    
                    assert len(v.out_edges) == 1
                    
                    for e in v.out_edges:
                        vp = e.dst
                        assert vp.data is v.data.body

                        mp = matrix.setdefault(vp, np.zeros( (6, 6) ) )

                        xc = v.data.local
                        
                        block = ((np.outer(fp, xp) + np.outer(xp, fp) ) / 2
                                 - fp.dot(xp) * np.identity(3)
                        )

                        mp[:3, :3] -= (dt * dt) * block

                        
    def fill_vector(self, vector, graph, dt):

        for v in graph.vertices:

            if type(v.data) is Body:
                # TODO fill forces/momentum
                vector[v] = np.zeros( 6 )
                
            if type(v.data) is Joint:
                vector[v] = v.data.error()

            if type(v.data) is Constraint:
                vector[v] = v.data.error() / dt

                
    def step(self, vector, old, dt = 1.0):

        for k, v in vector.items():
            
            if type(k.data) is Body:

                delta = Rigid3()
                delta.orient = Quaternion.exp( dt * v[:3])
                delta.center = dt * v[3:]

                # delta = Rigid3.exp(dt * v)
                
                copy = Rigid3()
                copy[:] = k.data.dofs
                
                old[k.data] = copy

                # k.data.dofs = k.data.dofs * delta
                k.data.dofs.center = k.data.dofs.center + k.data.dofs.orient(delta.center)
                k.data.dofs.orient = k.data.dofs.orient * delta.orient

            
        
def factor(matrix, forward):

    for v in forward:

        mv = matrix[v]

        for e in v.in_edges:
            me = matrix[e]
            assert e.src != v

            mv -= me.T.dot( matrix[e.src][0] ).dot(me)

        mv_inv = np.linalg.inv( mv )
        
        matrix[v] = (mv, mv_inv)

        for e in v.out_edges:
            matrix[e] = mv_inv.dot(matrix[e])
            

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


def make_skeleton(**kwargs):

    mass = 1
    inertia = np.ones(3)
    dim = np.ones(3)

    rho = 1
    
    def body(**kwargs):
        kwargs.setdefault('name', 'unnamed body')

        d = kwargs.setdefault('dim', dim)
        volume = d[0] * d[1] * d[2]

        m = rho * volume

        d2 = d * d
        inertia = m * (sum(d2) - d2) / 12.0
        
        kwargs.setdefault('mass', m )
        kwargs.setdefault('inertia', inertia)

        kwargs.setdefault('dofs', Rigid3() )                        
        
        return Body(**kwargs)


    def joint(*args, **kwargs):
        kwargs.setdefault('name', 'unnamed joint')
        kwargs.setdefault('nullspace', np.identity(6))
        return Joint(*args, **kwargs)

    
    def spherical(*args, **kwargs):
        kwargs.setdefault('name', 'unnamed joint')

        nullspace = np.identity(6)
        kwargs['nullspace'] = nullspace

        compliance = kwargs.get('compliance', 1) * np.identity(6)
        compliance[3:, 3:] = 0 * np.identity(3)
        
        kwargs['compliance'] = compliance
        
        return Joint(*args, **kwargs)


    def hinge(*args, **kwargs):
        kwargs.setdefault('name', 'unnamed joint')

        axis = kwargs.pop('axis', vec(1, 0, 0))

        nullspace = np.identity(6)
        kwargs['nullspace'] = nullspace

        compliance = np.zeros( (6, 6) )
        
        compliance[:3, :3] = kwargs.get('compliance', 1) * np.outer(axis, axis)
        compliance[3:, 3:] = 0 * np.identity(3)

        kwargs['compliance'] = compliance
        
        return Joint(*args, **kwargs)
    
    
    
    # bodies
    size = 1

    head_size = kwargs.get('head_size', 1.0)
    trunk_size = kwargs.get('trunk_size', 3.0)
    arm_size = kwargs.get('arm_size', 2.0)
    forearm_size = kwargs.get('forearm_size', 2.0)        
    femur_size = kwargs.get('femur_size', 3.0)
    tibia_size = kwargs.get('tibia_size', 2.0)
    foot_size = kwargs.get('foot_size', 1.5)            
    
    head = body(name = 'head', dim = vec(head_size, head_size, head_size) )
    trunk = body(name = 'trunk', dim = vec(2.0 / 3.0 * trunk_size,
                                           trunk_size,
                                           trunk_size / 3.0) )
    
    arm_dim = vec(arm_size / 4.0, arm_size, arm_size / 4.0)
    larm = body(name = 'larm', dim = arm_dim)
    rarm = body(name = 'rarm', dim = arm_dim )    

    forearm_dim = vec(forearm_size / 4.0, forearm_size, forearm_size / 4.0)
    lforearm = body(name = 'lforearm', dim = forearm_dim)
    rforearm = body(name = 'rforearm', dim = forearm_dim)

    femur_dim = vec(femur_size / 6.0, femur_size, femur_size / 6.0)
    lfemur = body(name = 'lfemur', dim = femur_dim)
    rfemur = body(name = 'rfemur', dim = femur_dim)

    tibia_dim = vec(tibia_size / 4.0, tibia_size, tibia_size / 4.0)
    ltibia = body(name = 'ltibia', dim = tibia_dim)
    rtibia = body(name = 'rtibia', dim = tibia_dim)

    foot_dim = vec(foot_size / 3.0, foot_size, foot_size / 3.0)
    lfoot = body(name = 'lfoot', dim = foot_dim)
    rfoot = body(name = 'rfoot', dim = foot_dim)
    
    
    bodies = [head, trunk,
              larm, rarm,
              lforearm, rforearm,
              lfemur, rfemur,
              ltibia, rtibia,
              lfoot, rfoot]

    
    # joints
    neck = spherical( (trunk, Rigid3(center = vec(0, 3 * trunk.dim[1] / 5, 0))),
                      (head, Rigid3(center = vec(0, -head.dim[1] / 2, 0))),
                      name = 'neck')

    lshoulder = spherical( (trunk, Rigid3(orient = Quaternion.exp( -math.pi / 4 * ez),
                                          center = vec(- 3 * trunk.dim[0] / 5,
                                                       trunk.dim[1] / 2,
                                                       0))),
                           (larm, Rigid3(center = vec(0, larm.dim[1] / 2, 0))),
                           name = 'lshoulder' )

    rshoulder = spherical( (trunk, Rigid3(orient = Quaternion.exp( math.pi / 4 * ez),
                                          center = vec(3 * trunk.dim[0] / 5,
                                                       trunk.dim[1] / 2,
                                                       0))),
                           (rarm, Rigid3(center = vec(0, rarm.dim[1] / 2, 0))),
                           name = 'rshoulder' )

    relbow = hinge( (rarm, Rigid3(orient = Quaternion.exp( -math.pi / 2 * ex),
                                  center = vec(0, -rarm.dim[1] / 2, 0))),
                        (rforearm, Rigid3(center = vec(0, rforearm.dim[1] / 2, 0))),
                        name = 'relbow' )
    
    lelbow = hinge( (larm, Rigid3(orient = Quaternion.exp( -math.pi / 2 * ex),
                                  center = vec(0, -larm.dim[1] / 2, 0))),
                        (lforearm, Rigid3(center = vec(0, lforearm.dim[1] / 2, 0))),
                        name = 'lelbow')


    lhip = spherical( (trunk, Rigid3( center = vec(-trunk.dim[0] / 2,
                                                  -trunk.dim[1] / 2,
                                                  0))),
                      (lfemur, Rigid3(center = vec(0, lfemur.dim[1] / 2, 0))),
                      name = 'lhip')


    rhip = spherical( (trunk, Rigid3( center = vec(trunk.dim[0] / 2,
                                                  -trunk.dim[1] / 2,
                                                  0))),
                      (rfemur, Rigid3(center = vec(0, rfemur.dim[1] / 2, 0))),
                      name = 'rhip')
    
    rknee = hinge( (rfemur, Rigid3(orient = Quaternion.exp( math.pi / 5 * ex),
                                   center = vec(0, -rfemur.dim[1] / 2, 0))),
                   (rtibia, Rigid3(center = vec(0, rtibia.dim[1] / 2, 0))),
                   name = 'rknee' )
    
    lknee = hinge( (lfemur, Rigid3(orient = Quaternion.exp( math.pi / 5 * ex),
                                   center = vec(0, -lfemur.dim[1] / 2, 0))),
                   (ltibia, Rigid3(center = vec(0, ltibia.dim[1] / 2, 0))),
                   name = 'lknee')


    rankle = spherical( (rtibia, Rigid3(orient = Quaternion.exp( -math.pi / 2 * ex),
                                       center = vec(0, -3 * rtibia.dim[1] / 5, 0))),
                       (rfoot, Rigid3(center = vec(0, 2 * rfoot.dim[1] / 5, 0))),
                       name = 'rankle' )

    lankle = spherical( (ltibia, Rigid3(orient = Quaternion.exp( -math.pi / 2 * ex),
                                       center = vec(0, -3 * ltibia.dim[1] / 5, 0))),
                        (lfoot, Rigid3(center = vec(0, 2 * lfoot.dim[1] / 5, 0))),
                        name = 'lankle' )
    

    
    joints = [neck,
              lshoulder, rshoulder,
              relbow, lelbow,
              lhip, rhip,
              lknee, rknee,
              lankle, rankle]



    # constraints
    stiffness = 1e2
    
    c1 = Constraint(lforearm, vec(0, -lforearm.dim[1] / 2, 0),
                    vec(-2, 3, 1),
                    stiffness)


    c2 = Constraint(rforearm, vec(0, -rforearm.dim[1] / 2, 0),
                    vec(2, 3, 1),
                    stiffness)

    c3 = Constraint(lfoot, vec(0, -lfoot.dim[1] / 2, 0),
                    vec(-2, -2, 0),
                    stiffness)


    c4 = Constraint(rfoot, vec(0, -rfoot.dim[1] / 2, 0),
                    vec(2, -2, 0),
                    stiffness)

    
    constraints = [c1, c2, c3, c4]
    
    
    return Skeleton(bodies, joints, constraints)



def solver(skeleton, graph, forward, dt = 1):

    matrix = {}
    vector = {}
    old = {}

    gs = False
    
    while True:

        matrix = {}

        # assemble
        skeleton.fill_matrix(matrix, vector, graph, old, dt, gs = gs)
        skeleton.fill_vector(vector, graph, dt)

        # solve
        factor(matrix, forward)
        solve(vector, matrix, forward)


        # step
        skeleton.step(vector, old, dt)
        
        yield



        
def draw():

    gl.glColor(1, 1, 1)
    skeleton.draw()

    gl.glLineWidth(4)
    gl.glPointSize(6)
    
    
    for c in skeleton.constraints:
        with gl.disable(gl.GL_LIGHTING):

            gl.glColor(1, 0, 0)            
            with gl.begin(gl.GL_POINTS):
                gl.glVertex(c.target)

            gl.glColor(1, 1, 0)
            with gl.begin(gl.GL_LINES):
                start = c.body.dofs(c.local)
                end = c.target

                gl.glVertex(start)
                gl.glVertex(end)
                
    
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


@tool.coroutine
def dragger(c):

    while True:
        pos = yield
        c.target[:] = pos

        
def select(p):
    
    for c in skeleton.constraints:

        # print(c.body.name, norm(p - c.target))
        if norm(p - c.target) < 0.2:
            global on_drag
            on_drag = dragger(c)


def drag(p):
    if on_drag: on_drag.send(p)

    
if __name__ == '__main__':
    on_drag = None


    skeleton = make_skeleton()

    graph = Graph([], [])


    data = skeleton.update( graph )
    forward = graph.orient( data[skeleton.bodies[1]] )


    s = solver(skeleton, graph, forward, 0.5)
    
    viewer.run()
