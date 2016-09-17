

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


def circle(**kwargs):
    
    radius = kwargs.get('radius', 0.5)
    steps = kwargs.get('steps', 16)    

    with begin(GL_LINE_LOOP):
        for i in range(steps):
            alpha = 2 * i * math.pi / steps
            glVertex(radius * math.cos(alpha), radius * math.sin(alpha))



def cube(**kwargs):
    
    mode = kwargs.get('mode', GL_QUADS)
    size = kwargs.get('size', 1)


    def side():
        # top
        glNormal(0, 0, 1)
        glVertex(1, 1, 1)
        glVertex(-1, 1, 1)
        glVertex(-1, -1, 1)
        glVertex(1, -1, 1)        
        
    
    with push_matrix():

        # top
        with begin(mode): side()

        # back
        glRotate(90, 1, 0, 0)
        with begin(mode): side()

        # bottom
        glRotate(90, 1, 0, 0)
        with begin(mode): side()

        # front
        glRotate(90, 1, 0, 0)
        with begin(mode): side()

        # right
        glRotate(90, 0, 1, 0)
        with begin(mode): side()

        # left
        glRotate(180, 0, 1, 0)
        with begin(mode): side()

        
        
def cylinder(**kwargs):
    '''a z-aligned cone'''    
    
    radius = kwargs.get('radius', 0.5)

    height = kwargs.get('height', 1)

    slices = kwargs.get('slices', 16)
    stacks = kwargs.get('stacks', 8)

    gluCylinder(cylinder.quad, radius, radius, height, slices, stacks)

cylinder.quad = gluNewQuadric()



def cone(**kwargs):
    '''a z-aligned cone'''
    radius = kwargs.get('radius', 0.5)

    height = kwargs.get('height', 1)

    slices = kwargs.get('slices', 16)
    stacks = kwargs.get('stacks', 8)

    gluCylinder(cone.quad, radius, 0, height, slices, stacks)

cone.quad = gluNewQuadric()



def arrow(**kwargs):
    '''a z-aligned arrow'''
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

    

