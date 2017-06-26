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
# nodes = np.arange(n)
points = np.random.rand(n, 3)


print(nodes)

# hermite basis
h00 = Polynomial( (1, 0, -3, 2) )
h10 = Polynomial( (0, 1, -2, 1) )
h01 = Polynomial( (0, 0, 3, -2) )
h11 = Polynomial( (0, 0, -1, 1) )


g0 = -h10 / 2
g1 = h00 - h11 / 2
g2 = h01 + h10 / 2
g3 = h11 / 2



c3 = g3
c2 = g2 + c3
c1 = g1 + c2
c0 = g0 + c1



def pad_nodes( nodes ):
    
    return np.pad(nodes, (1, 2),
                  'linear_ramp',
                  end_values = (2 * nodes[0] - nodes[1],
                                3 * nodes[-1] - 2 * nodes[-2]) )


def pad_values( nodes ):
    return np.pad(nodes, ((1, 2), (0, 0)), 'edge')



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
            m0, m1 = (v[-2] - v[0]) / 2, (v[-1] - v[1]) / 2

            dt = float(t1 - t0)
            
        x = (t - t0) / dt
        yield h00(x) * p0 + h01(x) * p1 + h10(x) * m0 + h11(x) * m1


def spline_points(nodes, values, ts):

    nodes = pad_nodes(nodes)
    values = pad_values(values)
    
    w = None

    for t in ts:

        if w is None or not (n[1] <= t < n[2]):
            w = window(nodes, t)

            n = nodes[w] 
            v = values[w]
            
            t0, t1 = n[1], n[2]
            dt = float(t1 - t0)

        x = (t - t0) / dt
        alpha = np.array([ g0(x), g1(x), g2(x), g3(x) ] )
        
        yield alpha.dot(v)



def spline_cumulative_factors_window(padded_nodes, ts):

    w = None

    for t in ts:

        if w is None or not (n[1] <= t < n[2]):
            w = window(padded_nodes, t)
            n = padded_nodes[w] 

            t0, t1 = n[1], n[2]
            dt = float(t1 - t0)

        x = (t - t0) / dt
        alpha = np.array([ c0(x), c1(x), c2(x), c3(x) ] )

        yield alpha, w


def spline_cumulative(nodes, values, ts):

    nodes = pad_nodes(nodes)
    values = pad_values(values)    

    for alpha, win in spline_cumulative_factors_window(nodes, ts):
        v = values[win]
        
        dv = v[1:] - v[:-1]
        
        yield alpha[0] * v[0] + alpha[1:].dot(dv)

        
sampled_nodes = np.linspace(nodes[0], nodes[-1], 300)

sampled_points = list(spline(nodes, points, sampled_nodes))
sampled_points2 = list(spline_points(nodes, points, sampled_nodes))
sampled_points3 = list(spline_cumulative(nodes, points, sampled_nodes))


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

    glBegin(GL_LINE_STRIP)    
    glColor(1, 1, 0)
    for p in sampled_points2:
        glVertex(p)
    glEnd()


    glBegin(GL_LINE_STRIP)    
    glColor(0, 1, 1)
    for p in sampled_points3:
        glVertex(p)
    glEnd()
    

    
    glEnable(GL_LIGHTING)




if __name__ == '__main__':
    viewer.run()




