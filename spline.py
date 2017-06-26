from __future__ import print_function
import numpy as np

from numpy.polynomial.polynomial import Polynomial

from snap.math import *
from snap.gl import *

from snap import viewer




def window(nodes, t, n = 4):
    start = np.searchsorted(nodes, t)
    if nodes[start] > t: start = start - 1
    # print('t', t, 'start', start, nodes)

    return slice(start - (n - 1) / 2, start + n / 2 + 1)
    

n = 10

nodes = np.linspace(-100, 100, n)
points = np.random.rand(n, 3)


print(nodes)

# hermite basis
h00 = Polynomial( (1, 0, -3, 2) )
h10 = Polynomial( (0, 1, -2, 1) )
h01 = Polynomial( (0, 0, 3, -2) )
h11 = Polynomial( (0, 0, -1, 1) )


   
    
def pad_nodes( nodes ):
    return np.pad(nodes, 1,
                  'constant',
                  constant_values = ( 2 * nodes[0] - nodes[1],
                                      2 * nodes[-1] - nodes[-2]) )

def pad_values( nodes ):
    return np.pad(nodes, ((1, 1), (0, 0)), 'edge')



def spline(nodes, values, ts):

    nodes = pad_nodes(nodes)
    values = pad_values(values)
    
    w = None

    for t in ts:

        if w is None or not (n[1] <= t < n[2]):
            w = window(nodes, t)

            n = nodes[w] 
            v = values[w]
            
            t0, t1 = n[1], n[2]
            p0, p1 = v[1:3]
            m0, m1 = v[-2] - v[0], v[-1] - v[1]

            dt = float(t1 - t0)
            
        x = (t - t0) / dt
        yield h00(x) * p0 + h01(x) * p1 + h10(x) * m0 + h11(x) * m1


        
sampled_nodes = np.linspace(nodes[0], nodes[-1], 300)
sampled_points = list(spline(nodes, points, sampled_nodes))


def draw():
    m = 100

    glDisable(GL_LIGHTING)
    glPointSize(5)

    
    glBegin(GL_POINTS)

    glColor(1, 0, 0)
    for p in points:
        glVertex(p)
    glEnd()
    
    glBegin(GL_LINE_STRIP)    
    glColor(1, 1, 1)

    
    for p in sampled_points:
        glVertex(p)
            
    glEnd()
    glEnable(GL_LIGHTING)




if __name__ == '__main__':
    viewer.run()




