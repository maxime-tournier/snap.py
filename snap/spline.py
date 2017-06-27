from __future__ import print_function
import numpy as np

from numpy.polynomial.polynomial import Polynomial

from . import tool
from .math import *

# hermite basis (p0, p1, m0, m1)
h00 = Polynomial( (1, 0, -3, 2) )
h10 = Polynomial( (0, 1, -2, 1) )
h01 = Polynomial( (0, 0, 3, -2) )
h11 = Polynomial( (0, 0, -1, 1) )

# catmull-rom spline basis (p-1, p0, p1, p2)
g0 = -h10 / 2
g1 = h00 - h11 / 2
g2 = h01 + h10 / 2
g3 = h11 / 2


# catmull-rom cumulative basis
c3 = g3
c2 = g2 + c3
c1 = g1 + c2
c0 = g0 + c1

dc3 = c3.deriv()
dc2 = c2.deriv()
dc1 = c1.deriv()
dc0 = c0.deriv()


def window(nodes, t, n = 4):
    start = np.searchsorted(nodes, t)
    if nodes[start] > t: start = start - 1
    # print('window', 't:', t, 'start:', start, 'nodes:', nodes)
    
    return slice( start - int((n - 1) / 2), start + int(n / 2) + 1)


def pad_nodes( nodes ):
    return np.pad(nodes, (1, 2),
                  'linear_ramp',
                  end_values = (2 * nodes[0] - nodes[1],
                                3 * nodes[-1] - 2 * nodes[-2]) )


def pad_values( nodes ):
    return np.pad(nodes, ((1, 2), (0, 0)), 'edge')



class Spline(object):
    '''a general lie group catmull-rom spline

    Based on Kim et al. 1995, A general construction scheme for unit
    quaternion curves with simple high order derivatives'''

    __slots__ = 'group', 'state'
    
    def __init__(self, nodes, values, group):
        
        padded_nodes = pad_nodes(nodes)
        padded_values = pad_values(values)

        self.group = group
        
        assert len(nodes) == len(values)

        @tool.coroutine
        def state():
            result = None
            n = 4
            
            while True:
                x = yield result

                if result is None or not (x0 <= x < x1):
                    # recompute stuff
                    win = window(padded_nodes, x, n)
                    
                    nodes = padded_nodes[win]
                    values = padded_values[win]
                    
                    omega = np.zeros((n, group.dim))
                    omega[0] = group.log( values[0] )

                    for i in range(1, n):
                        omega[i] = group.log( group.prod(group.inv(values[i-1]), values[i]) )

                    x0, x1 = nodes[1], nodes[2]
                    delta = x1 - x0
        
                t = (x - x0) / delta

                result = t, omega

        self.state = state()


        
    def __call__(self, x):
        
        t, omega = self.state.send(x)

        alpha = np.array( [ c0(t), c1(t), c2(t), c3(t) ] )
        dalpha = np.array( [ dc0(t), dc1(t), dc2(t), dc3(t) ] )
        
        result = self.group.identity()

        # spatial velocity
        dresult = np.zeros(self.group.dim)

        for i in range( alpha.size ):
            dresult = dresult + self.group.ad(result, dalpha[i] * omega[i])
            result = self.group.prod(result, self.group.exp( alpha[i] * omega[i] ))
            
        return result, dresult



class VectorSpace(object):
    __slots__ = 'dim', 

    def __init__(self, dim):
        self.dim = dim

    def identity(self): return np.zeros(self.dim)    
    def prod(self, lhs, rhs): return lhs + rhs
    def inv(self, x): return -x

    def ad(self, x, y): return y
    def exp(self, x): return x
    def log(self, x): return x    


class RotationGroup(object):
    __slots__ = ()

    dim = 3
    
    def identity(self): return Quaternion()

    def prod(self, lhs, rhs): 
        return lhs.view(Quaternion) * rhs.view(Quaternion)
    
    def inv(self, x): return x.view(Quaternion).inv()

    def exp(self, x): return Quaternion.exp(x)
    def log(self, x): return x.view(Quaternion).log()

    def ad(self, x, y):
        return x.view(Quaternion)(y)


class TranslationRotationGroup(object):
    __slots = ()
    
    dim = 6
    
    def identity(self): return Rigid3()

    def prod(self, lhs, rhs):
        lhs = lhs.view(Rigid3)
        rhs = rhs.view(Rigid3)
        
        res = Rigid3()
        res.center = lhs.center + rhs.center
        res.orient = lhs.orient * rhs.orient
        return res
    
    def inv(self, x):
        x = x.view(Rigid3)
        res = Rigid3()
        res.center = -x.center
        res.orient = x.orient.conj()
        return res

    def exp(self, x):
        
        x = x.view(Rigid3.Deriv)
        res = Rigid3()
        res.orient = Quaternion.exp(x.angular)
        res.center = x.linear

        return res
    
    def log(self, x):
        x = x.view(Rigid3)
        
        res = Rigid3.Deriv()
        res.angular = x.orient.log()
        res.linear = x.center

        return res


    def ad(self, x, y):
        x = x.view(Rigid3)
        y = y.view(Rigid3.Deriv)
        
        res = Rigid3.Deriv()
        res.angular = x.orient(y.angular)
        res.linear = y.linear

        return res

    


