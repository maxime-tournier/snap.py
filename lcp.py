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
    

def random_lcp(n, scale):
    m = 2 * n
    
    F = np.random.rand(m, n)
    M = F.T.dot(F)

    q = scale * (np.random.rand(n) - 0.5)

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

def make_original():
    n = 3
    iterations = 20
    scale = 10
    
    class LCP:
        def __init__(self):
            self.M, self.q = random_lcp(n, scale)

            self.Minv = np.linalg.inv(self.M)
            self.r = -self.Minv.dot(self.q)
            self.s, self.v = np.linalg.eigh(self.M)

            self.x = solve(self.M, self.q)

            self.c = self.r / 2

            _, self.quat = eigen_frame(self.M)
            
            self.restart()
            
            
        def restart(self):
            self.start = scale * (np.random.rand(3) - 0.5)
            self.traj = [np.copy(y)
                         for i, y in zip(range(iterations),
                                         pgs_debug(np.copy(self.start),
                                                   self.M, self.q)) ]
        
        def draw(self):
            # glClear(GL_DEPTH_BUFFER_BIT)
            radius = math.sqrt(-self.q.dot(self.r) / 4)

            with points():
                glColor(1, 1, 0)
                glVertex(*self.c)

            glColor(1, 1, 1, 0.5)
            draw_ellipsoid(self.M, self.c, radius=radius)

            glClear(GL_DEPTH_BUFFER_BIT)        
            with push_matrix():
                glTranslate(*self.r)
                
                gl.rotate(self.quat)
                glScale(*(1.0 / self.s))
                gl.rotate(self.quat.conj())
                draw_orthant()

            with points():
                glColor(1, 0, 0)
                glVertex(*self.x)

            with lines():
                glColor(1, 0, 1)
                glVertex(0, 0, 0)
                glVertex(*self.q)

            with points():
                glColor(0, 1, 0)
                glVertex(*self.start)

            with curve():
                glColor(0, 1, 0)
                for y in self.traj:
                    glVertex(*y)

            # 
            glClear(GL_DEPTH_BUFFER_BIT)        
            draw_orthant()

        def draw_normalized(self):
            power = lambda x: self.v.dot(np.diag(self.s ** x)).dot(self.v.T)

            r = -power(0.5).dot(self.q) / 2
            
            glColor(1, 1, 1, 0.5)
            draw_ellipsoid(np.identity(3), np.zeros(3), radius=np.linalg.norm(r))
            
            glClear(GL_DEPTH_BUFFER_BIT)        
            with push_matrix():
                glTranslate(*(-r))
                
                gl.rotate(self.quat)
                glScale(*(self.s ** 0.5))
                gl.rotate(self.quat.conj())
                draw_orthant()


            glClear(GL_DEPTH_BUFFER_BIT)        
            with push_matrix():
                glTranslate(*r)
                
                gl.rotate(self.quat)
                glScale(*(self.s ** -0.5))
                gl.rotate(self.quat.conj())
                draw_orthant()
                
            
    return LCP()

if __name__ == '__main__':

    lcp = make_original()

    class Viewer(viewer.Viewer):
        def draw(self):
            glBlendFunc(gl.GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);    
            with gl.enable(gl.GL_BLEND):
                # lcp.draw()
                lcp.draw_normalized()

        def on_keypress(self, key):
            global lcp
            
            if key == 'r':
                lcp.restart()
                self.update()
                
            elif key == 'n':
                lcp = make_original()
                self.update()

    with viewer.app():
        viewer = Viewer()
        viewer.showFullScreen()
        viewer.show()
