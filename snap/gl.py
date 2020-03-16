

from OpenGL.GL import *
from OpenGL.GLU import *

from .math import *
from contextlib import contextmanager
from .tool import coroutine

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
        with begin(mode):
            side()

        # back
        glRotate(90, 1, 0, 0)
        with begin(mode):
            side()

        # bottom
        glRotate(90, 1, 0, 0)
        with begin(mode):
            side()

        # front
        glRotate(90, 1, 0, 0)
        with begin(mode):
            side()

        # right
        glRotate(90, 0, 1, 0)
        with begin(mode):
            side()

        # left
        glRotate(180, 0, 1, 0)
        with begin(mode):
            side()


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

    cylinder(radius=radius, height=0.8 * height)
    with push_matrix():
        glTranslate(0, 0, 0.8 * height)
        cone(radius=2.0 * radius, height=0.2 * height)


def rotate(q):
    '''glRotate from a quaternion'''

    axis, angle = q.axis_angle()
    if axis is not None:
        glRotate(angle / deg, *axis)


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
        rotate(Quaternion.from_vectors(view, target))
        yield

        
def axis():
    glColor(1, 1, 1)
    sphere(radius=0.025)

    glColor(1, 0.2, 0.2)
    with lookat(ex):
        arrow()

    glColor(0.2, 1, 0.2)
    with lookat(ey):
        arrow()

    glColor(0.2, 0.2, 1)
    with lookat(ez):
        arrow()
        

class Camera:
    __slots__ = ('fov', 'znear', 'zfar', 'target', 'frame', 'width', 'height')

    def __init__(self):
        self.znear = 0.1
        self.zfar = 100.0

        self.fov = 60

        self.target = np.zeros(3)
        self.frame = Rigid3()

        self.width, self.height = None, None
        
        self.frame.center[2] = 1
        
    @property
    def ratio(self):
        return self.width / self.height

    @property
    def view(self):
        return self.frame.orient(-ez)
    
    def projection(self):
        gluPerspective(self.fov, self.ratio, self.znear, self.zfar)

        
    def modelview(self):
        inv = self.frame.inv()

        glTranslate(*inv.center)

        n, theta = inv.orient.axis_angle()
        if n is not None:
            glRotate(theta / deg, *n)

    
    def radius(self, value):
        self.znear = max(0, (self.distance - value) / 2)
        self.zfar = max(1, (self.distance + value) * 2)
        
    radius = property(None, radius)

    def aabb(self, aabb):
        lower, upper = aabb
        radius = norm(upper - lower) / 2

        self.target = (upper + lower) / 2
        self.radius = radius

    aabb = property(None, aabb)
        
    def resize(self, width, height):
        self.width = width
        self.height = height
        glViewport(0, 0, width, height)

    def mouse_win(self):
        raise RuntimeError('unimplemented')
        
    def mouse_cam(self, z=0):
        x, y = self.mouse_win()
        return np.array([2 * (x / self.width - 0.5),
                         2 * (-y / self.height - 0.5),
                         z])
    
    @coroutine
    def mouse_move(self):
        init = self.mouse_cam()
        delta = None
        
        while True:
            yield delta
            current = self.mouse_cam()
            delta = np.array(current) - np.array(init)

    @property
    def distance(self):
        return np.linalg.norm(self.target - self.frame.center)
            
    translate_factor = 1.0
    rotate_factor = 1.0
    zoom_factor = 1.0

    def unproject(self, x, y, z=0):
        return np.array(projection.gluUnProject(x, y, z))
    
    def mouse_translate(self):
        init = self.frame.copy()
        factor = self.translate_factor * self.distance

        for dx in self.mouse_move():
            self.frame.center = init.center - init.orient(factor * dx)
            yield
            
            
    def mouse_trackball(self):
        init = self.frame.copy()

        winx, winy = self.mouse_win()
        p = self.unproject(winx, winy, 0)
        x = self.mouse_cam()
        
        factor = self.rotate_factor / (self.znear + self.distance)
        
        for dx in self.mouse_move():
            omega = np.cross(p - self.target, init.orient(dx))
            quat = Quaternion.exp(-omega * factor)

            twist = (Rigid3.translation(self.target) *
                     Rigid3.rotation(quat) *
                     Rigid3.translation(-self.target))
            
            self.frame = twist * init
            
            yield

    def mouse_rotate(self):
        init = self.frame.copy()

        for dx in self.mouse_move():
            omega = np.cross(init.center - self.target, init.orient(dx))
            quat = Quaternion.exp(-omega * self.rotate_factor)

            self.frame = init * Rigid3.rotation(quat)
            
            yield
        

    def mouse_zoom(self):
        init = self.frame.copy()
        local_view = init.orient.inv()(self.target - init.center)
        
        factor = self.zoom_factor * self.distance
        
        for dx, dy, dz in self.mouse_move():
            self.frame = init * Rigid3.translation(-factor * dy * local_view)
            yield


    wheel_factor = 0.1
            
    def mouse_zoom_wheel(self, amount):
        init = self.frame.copy()
        local_view = init.orient.inv()(self.target - init.center)

        factor = amount * self.wheel_factor
        
        self.frame = init * Rigid3.translation(-factor * local_view)
        yield
            
        
    @contextmanager
    def draw(self):
        try:
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            self.projection()

            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()            
            self.modelview()

            glPushMatrix()
            yield
        finally:
            glPopMatrix()
        

    def draw_axis(self):
        glPushMatrix()
        glTranslate(*self.target)
        axis()
        glPopMatrix()
        
