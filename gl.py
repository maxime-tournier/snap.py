

from OpenGL.GL import *
from OpenGL.GLU import *

from .math import *
from contextlib import contextmanager

def sphere(**kwargs):
    
    radius = kwargs.get('radius', 0.5)
    slices = kwargs.get('slices', 16)
    stacks = kwargs.get('stacks', 8)

    gluSphere(sphere.quad, radius, slices, stacks)

sphere.quad = gluNewQuadric()



def cylinder(**kwargs):
    
    radius = kwargs.get('radius', 0.5)

    height = kwargs.get('height', 1)

    slices = kwargs.get('slices', 16)
    stacks = kwargs.get('stacks', 8)

    gluCylinder(cylinder.quad, radius, radius, height, slices, stacks)

cylinder.quad = gluNewQuadric()



def cone(**kwargs):
    radius = kwargs.get('radius', 0.5)

    height = kwargs.get('height', 1)

    slices = kwargs.get('slices', 16)
    stacks = kwargs.get('stacks', 8)

    gluCylinder(cone.quad, radius, 0, height, slices, stacks)

cone.quad = gluNewQuadric()



def arrow(**kwargs):
    radius = kwargs.get('radius', 0.025)

    height = kwargs.get('height', 1)

    # slices = kwargs.get('slices', 16)
    # stacks = kwargs.get('stacks', 8)

    cylinder(radius = radius, height = 0.8 * height)
    with push_matrix():
        glTranslate(0, 0, 0.8 * height)
        cone(radius = 2.0 * radius, height = 0.2 * height)


def rotate(q):
    '''glRotate from a quaternion'''
    
    axis, angle = q.axis_angle()
    if axis is not None:
        glRotate(angle * deg, *axis)

        
@contextmanager
def push_matrix():
    glPushMatrix()
    try:
        yield
    finally:
        glPopMatrix()



@contextmanager
def enable(*args):
    for a in args:
        glEnable(a)
    try:
        yield
    finally:
        for a in args:
            glDisable(a)


@contextmanager
def disable(*args):
    for a in args:
        glDisable(a)
    try:
        yield
    finally:
        for a in args:
            glEnable(a)
            

@contextmanager
def begin(what):

    glBegin(what)
    try:
        yield
    finally:
        glEnd()
        
@contextmanager
def frame(g):
    '''rigid frame context'''

    with push_matrix():
        glTranslate(*g.center)
        rotate(g.orient)
        yield


@contextmanager
def lookat(target, **kwargs):
    view = kwargs.get('view', ez)
    
    with push_matrix():
        rotate( Quaternion.from_vectors(view, target) )
        yield

    

