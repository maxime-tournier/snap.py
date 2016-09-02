
import numpy as np

from collections import namedtuple

from snap.math import *


# TODO joint nullspace (subclass)


class Body(namedtuple('Body', 'mass inertia dim name dofs')):

    def __hash__(self):
        return id(self)

    def __str__(self):
        return 'Body({0})'.format(self.name)

    def inertia_tensor(self):
        return np.identity(6)

    
class Joint(namedtuple('Joint', 'parent child name nullspace')):

    def __hash__(self):
        return id(self)

    def __str__(self):
        return 'Joint({0}, {1})'.format(self.parent[0].name, self.child[0].name)

    def jacobian(self):

        p = self.parent[0].dofs * self.parent[1]
        c = self.child[0].dofs * self.child[1]        
        
        r = p.inv() * c
        
        dp = self.parent[1].inv().Ad()
        dc = self.child[1].inv().Ad()

        return -self.nullspace.dot(r.inv().Ad() * dp), self.nullspace.dot(dc)

    
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
        return '({0}, {1})'.format( str(self.src), str(self.dst) )
    
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

                    v.out_edges.append(e)
                    other.in_edges.append(e)

                    assert len(other.in_edges) == 1, "cyclic graph"
                    res += postfix(other)

            return res + [v]

        res = postfix(start)
        assert len(res) == len(self.vertices)

        return res
    

class Skeleton(namedtuple('Skeleton', 'bodies joints constraints')):

    def draw(self): pass
    
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

            e1 = Edge( data[j.parent[0]], v, j)
            e2 = Edge( v, data[j.child[0]], j)

            graph.edges.append( e1 )
            graph.edges.append( e2 )

            
        for c in self.constraints:
            v = add_vertex(c)
            e = Edge(data[c.body])

            graph.edges.append(e)

        return data


    def fill(self, matrix, graph):
        
        for v in graph.vertices:

            if type(v.data) is Body:
                matrix[v] = v.data.inertia_tensor()

            if type(v.data) is Joint:
                assert len(v.in_edges) + len(v.out_edges) == 2, 'is the graph oriented ?'
                
                edges = v.in_edges + v.out_edges

                p = v.data.parent[0]
                c = v.data.child[0]
                
                if edges[0].dst.data != p:
                    assert edges[1].src.data == p and edges[0].dst.data == c

                    # reorder edges so that it matches (p, c)
                    edges = [edges[1], edges[0]]


                J = v.data.jacobian()

                matrix[v] = np.zeros( (6, 6) )
                
                for i in range(2):
                    matrix[ edges[i] ] = J[i]
            # TODO constraints

def factor(forward, matrix):
    pass

def solve(forward, matrix, vector):
    pass

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
        nullspace[:, :3] = np.identity(3)
        
        kwargs['nullspace'] = nullspace
        return Joint(*args, **kwargs)
    
    
    # bodies
    head = body(name = 'head')
    trunk = body(name = 'trunk')
    
    larm = body(name = 'larm')
    rarm = body(name = 'rarm')    

    lforearm = body(name = 'lforearm')
    rforearm = body(name = 'rforearm')    

    bodies = [head, trunk, larm, rarm, lforearm, rforearm]

    # joints
    neck = joint( (trunk, Rigid3()), (head, Rigid3()), name = 'neck')

    lshoulder = joint( (trunk, Rigid3()), (larm, Rigid3()), name = 'lshoulder' )

    rshoulder = joint( (trunk, Rigid3()), (rarm, Rigid3()), name = 'rshoulder' )

    relbow = joint( (rarm, Rigid3()), (rforearm, Rigid3()), name = 'relbow' )

    lelbow = hinge( (larm, Rigid3()), (lforearm, Rigid3()), name = 'lelbow')
    
    joints = [neck, lshoulder, rshoulder, relbow, lelbow]
    
    return Skeleton(bodies, joints, [])



skeleton = make_skeleton()

graph = Graph([], [])


data = skeleton.update( graph )
matrix = {}


graph.orient( data[skeleton.bodies[2]] )

skeleton.fill(matrix, graph)

for k, v in matrix.items():
    print(str(k), str(v))

# print( str(matrix) )
