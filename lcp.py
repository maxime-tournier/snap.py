from snap import viewer, gl, tool

from snap.gl import *
from snap.math import *

import numpy as np


def draw_orthant(size=100):
    with push_matrix():
        glColor(1, 1, 1, 0.7)
        
        with points():
            glVertex(0, 0, 0)

        with lines():
            glVertex(0, 0, 0)
            glVertex(size, 0, 0)

            glVertex(0, 0, 0)
            glVertex(0, size, 0)

            glVertex(0, 0, 0)
            glVertex(0, 0, size)

            
        glTranslate(size, size, size)
        glScale(size, size, size)

        glColor(1, 1, 1, 0.2)            
        gl.cube()            
        


from contextlib import contextmanager

def eigen_frame(M):
    s, v = np.linalg.eigh(M)

    # det check
    if np.linalg.det(v) < 0:
        v = -v

    q = Quaternion.from_matrix(v)
    return s, q
        

        
def draw_ellipsoid(M, c, radius):
    with gl.push_matrix():
        glTranslate(*c)

        s, q = eigen_frame(M)

        gl.rotate(q)
        glScale(*(s ** -0.5))
        
        gl.sphere(radius=radius, slices=128, stacks=64)
    

def lcp(n):
    m = 2 * n
    
    F = np.random.rand(m, n)
    M = F.T.dot(F)

    q = 10 * (np.random.rand(n) - 0.5)

    return (M, q)


def pgs(x, M, q):
    while True:
        for i, qi in enumerate(q):
            x[i] -= (M[:, i].dot(x) + qi) / M[i, i]
            x[i] = max(x[i], 0)

        error = np.linalg.norm(np.minimum(x, M.dot(x) + q))
        
        yield x

        if error <= 1e-10: return
        


def solve(M, q):
    n = 100
    
    for i, x in zip(range(n), pgs(np.zeros(q.size), M, q)):
        pass

    return x


def pgs_debug(x, M, q, eps=1e-10):
    yield x
    
    while True:
        for i, qi in enumerate(q):
            x[i] -= (M[:, i].dot(x) + qi) / M[i, i]
            yield x
            x[i] = max(x[i], 0)
            yield x

        error = np.linalg.norm(np.minimum(x, M.dot(x) + q))
        
        if error <= eps: return
    

@contextmanager
def points():
    with gl.disable(GL_LIGHTING):
        with gl.disable(GL_DEPTH_TEST):
            glPointSize(10)
            with gl.begin(GL_POINTS):
                yield


@contextmanager
def lines():
    with gl.disable(GL_LIGHTING):
        with gl.disable(GL_DEPTH_TEST):
            glLineWidth(4)
            with gl.begin(GL_LINES):
                yield

@contextmanager
def curve():
    with gl.disable(GL_LIGHTING):
        with gl.disable(GL_DEPTH_TEST):
            glLineWidth(4)
            with gl.begin(GL_LINE_STRIP):
                yield
                

                
def draw():

    glBlendFunc(gl.GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    
    with gl.enable(gl.GL_BLEND):
        

        draw_problem()
        glClear(GL_DEPTH_BUFFER_BIT)        
        draw_orthant()
        


if __name__ == '__main__':
    n = 3
    M, q = lcp(n)

    # M = np.identity(3)
    # M = np.diag([3, 4, 5])

    # U = Quaternion.from_vectors(vec(1, 0, 0),
    #                             vec(0, 1, 0)).matrix()
    # M = U.dot(M).dot(U.T)

    Minv = np.linalg.inv(M)
    r = -Minv.dot(q)
    s, v = np.linalg.eigh(M)
    print(v, s)

    x = solve(M, q)
    print(x)

    start = np.random.rand(3)
    traj = [np.copy(y) for i, y in zip(range(20),
                                       pgs_debug(np.copy(start), M, q)) ]
    
    
    def draw_problem():
        # glClear(GL_DEPTH_BUFFER_BIT)
        radius = math.sqrt(-q.dot(r) / 4)

        with points():
            glColor(1, 1, 0)
            glVertex(*(r/2))

        glColor(1, 1, 1, 0.5)
        draw_ellipsoid(M, r/2, radius=radius)

        glClear(GL_DEPTH_BUFFER_BIT)        
        with push_matrix():
            glTranslate(*r)

            s, u = eigen_frame(Minv)
            
            gl.rotate(u)
            glScale(*s)
            gl.rotate(u.conj())
            draw_orthant()

        with points():
            glColor(1, 0, 0)
            glVertex(*x)
                
        with lines():
            glColor(1, 0, 1)
            glVertex(0, 0, 0)
            glVertex(*q)

        with points():
            glColor(0, 1, 0)
            glVertex(*start)
            
        with curve():
            glColor(0, 1, 0)
            for y in traj:
                glVertex(*y)
    
    viewer.run(fullscreen=True)
