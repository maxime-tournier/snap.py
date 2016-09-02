
import numpy as np

from collections import namedtuple

from snap.math import *

class Body(namedtuple('Body', 'mass inertia dim name')):

    def __hash__(self):
        return id(self)

    def __str__(self):
        return 'Body({0})'.format(self.name)

class Joint(namedtuple('Joint', 'parent child')):

    def __hash__(self):
        return id(self)

    def __str__(self):
        return 'Joint({0}, {1})'.format(self.parent[0].name, self.child[0].name)

class Skeleton(namedtuple('Skeleton', 'bodies joints')):

    def draw(self): pass
    
    def forward(self, start = None):
        '''build a postfix traversal of the constraint graph for skeleton'''
        
        marked = set()

        edges =  { }

        for j in self.joints:
            p = j.parent[0]
            c = j.child[0]

            edges[p] = edges.get(p, [])
            edges[c] = edges.get(c, [])            

            edges[p].append(j)
            edges[c].append(j)


        def postfix(b):
            '''generate a postfix traversal of body/joints'''
            
            res = []
            
            # print('edges for', str(b), list(map(str, edges[b])))

            for e in edges[b]:

                if e not in marked:
                    
                    marked.add(e)
                    # TODO remove edge ?

                    other = e.parent[0] if e.parent[0] != b else e.child[0]

                    res += postfix(other) + [e]

            return res + [b]

        start = start or self.bodies[0]
        assert type(start) is Body

        res = postfix(start)
        assert len(res) == len(self.joints) + len(self.bodies), 'skeleton is not connected'

        return res
    
    
def make_skeleton():

    mass = 1
    inertia = np.ones(3)
    dim = np.ones(3)

    def body(**kwargs):
        name = kwargs.get('name', 'unnamed')
        return Body(mass = mass, inertia = inertia, dim = dim, name = name)


    # bodies
    head = body(name = 'head')
    trunk = body(name = 'trunk')
    
    larm = body(name = 'larm')
    rarm = body(name = 'rarm')    

    lforearm = body(name = 'lforearm')
    rforearm = body(name = 'rforearm')    

    bodies = [head, trunk, larm, rarm, lforearm, rforearm]

    # joints
    neck = Joint( (trunk, Rigid3()),
                  (head, Rigid3()) )

    lshoulder = Joint( (trunk, Rigid3()),
                       (larm, Rigid3()) )

    rshoulder = Joint( (trunk, Rigid3()),
                       (rarm, Rigid3()) )

    relbow = Joint( (rarm, Rigid3()),
                    (rforearm, Rigid3()) )

    lelbow = Joint( (larm, Rigid3()),
                    (lforearm, Rigid3()) )

    joints = [neck, lshoulder, rshoulder, relbow, lelbow]
    
    return Skeleton(bodies, joints)


skeleton = make_skeleton()

print( list(map(str, skeleton.forward() )))
