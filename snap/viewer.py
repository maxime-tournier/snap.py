# MIT license
# tournier.maxime@gmail.com

import OpenGL

from OpenGL.GL import *
from OpenGL.GLU import *

from .math import *
from .tool import *
from .qt import *

from . import gl

import time
import sys


class Camera(object):

    def __init__(self, owner):

        self.owner = owner

        # frame/pivot
        self.frame = Rigid3()
        self.frame.center[2] = 1
        self.pivot = vec(0, 0, 0)

        # frustum
        self.znear = 0.1
        self.zfar = 100.0
        self.vfov = 60.0

        # width / height
        self.ratio = 1.0

        self.rotation_sensitivity = 1
        self.translation_sensitivity = 1
        self.zoom_sensitivity = 1

        # spin/slide
        self.dframe = Rigid3()
        self.damping = 1e-2
        self.stop_velocity = 1e-3

    @property
    def projection(self):
        res = QtGui.QMatrix4x4()

        res.setToIdentity()
        res.perspective(self.vfov, self.ratio, self.znear, self.zfar)
        return res

    @property
    def modelview(self):
        res = QtGui.QMatrix4x4()
        inv = self.frame.inv()

        res.setToIdentity()
        res.translate(*inv.center)
        axis, angle = inv.orient.axis_angle()

        if axis is not None:
            res.rotate(angle / deg, *axis)
        return res

    def pixel_coords(self, x, y):
        '''pixel -> viewport coords'''

        rx = float(x) / float(self.owner.width())
        ry = float(self.owner.height() - 1.0 - y) / float(self.owner.height())

        return rx, ry

    def unproject(self, Pinv, x, y, z=0):
        '''unproject viewport coordinates'''

        d = 2.0 * QtGui.QVector4D(x, y, z, 1.0) - QtGui.QVector4D(1.0, 1.0, 1.0, 1.0)
        res = Pinv.map(d)

        return vec(res.x(), res.y(), res.z()) / res.w()

    def pixel_depth(self, px, py):
        '''read depth under pixel, or None'''
        self.owner.makeCurrent()
        read = glReadPixels(px, self.owner.height() - 1 - py,
                            1, 1,
                            GL_DEPTH_COMPONENT, GL_FLOAT)
        res = read[0][0]
        return res if res < 1.0 else None

    def point_under_pixel(self, x, y):
        '''point under pixel or None'''
        z = self.pixel_depth(x, y)
        if z is None:
            return None

        x, y = self.pixel_coords(x, y)

        Pinv, ok = self.projection.inverted()
        return self.unproject(Pinv, x, y, z)

    @property
    def pivot_distance(self):
        return norm(self.frame.center - self.pivot)

    @coroutine
    def mouse_translate(self, start):
        '''translate camera from mouse move events'''
        start_pos = start.pos()

        start_frame = Rigid3()
        start_frame[:] = self.frame

        Pinv, ok = self.projection.inverted()
        assert ok

        z = self.znear
        z = (z - self.znear) / (self.zfar - self.znear)

        sx, sy = self.pixel_coords(start_pos.x(), start_pos.y())
        s = self.unproject(Pinv, sx, sy, z)

        while True:
            ev = yield

            ex, ey = self.pixel_coords(ev.pos().x(), ev.pos().y())
            e = self.unproject(Pinv, ex, ey, z)

            d = e - s

            scale = self.pivot_distance

            f = Rigid3()
            f.center = scale * 10 * self.translation_sensitivity * d

            next_frame = start_frame * f.inv()

            self.dframe = self.frame.inv() * next_frame
            self.frame = next_frame

    def axis_align(self):
        '''align camera with nearest axis'''

        pos = np.abs(self.frame.center - self.pivot).tolist()

        index = pos.index(max(pos))

        self.frame.center = [self.pivot[i] if i != index else self.frame.center[i]
                             for i in range(3)]

        local_pivot = self.frame.inv()(self.pivot)

        # look at pivot
        q = Quaternion.from_vectors(-ez, local_pivot)
        self.frame.orient = self.frame.orient * q

        # pick axis closest to camera up axis
        cam_up = self.frame.orient(ey)
        proj = np.abs(cam_up).tolist()
        index = proj.index(max(proj))

        up = vec(0, 0, 0)
        up[index] = proj[index] / cam_up[index]  # to get sign right

        q = Quaternion.from_vectors(ey, self.frame.orient.inv()(up))
        self.frame.orient = self.frame.orient * q

    def lookat(self, target, **kwargs):
        '''make camera point at target'''

        local_target = self.frame.inv()(target)
        q = Quaternion.from_vectors(-ez, local_target)

        self.frame.orient = self.frame.orient * q

        # project q.inv() onto geodesic around z
        up = kwargs.get('up', ey)
        qinv = Quaternion.from_vectors(ey, self.frame.orient.inv()(up))

        if math.fabs(q.real) < Quaternion.epsilon:
            return

        # gnomonic projection
        pqinv = qinv / q.real

        r = Quaternion()
        r.imag[2] = pqinv.imag[2]
        r /= norm(r)

        self.frame.orient = self.frame.orient * r

    wheel_direction = 1.0

    @coroutine
    def mouse_zoom(self, start=None):
        '''ajust zoom from mouse'''
        if start is not None:
            last = start.pos()

        while True:
            ev = yield

            if start is None:
                degrees = self.wheel_direction * float(wheel_angle(ev)) / 256.0
            else:
                # dx = ev.pos().x() - start.x()
                dy = ev.pos().y() - last.y()
                last = ev.pos()

                degrees = - self.wheel_direction * (dy / 256.0)

            u = self.frame.inv()(self.pivot)

            dist = norm(u)
            view = u / dist

            delta = (self.zoom_sensitivity * degrees) * dist

            # make sure we dont zoom closer than znear
            delta = min(delta, dist - self.znear)

            f = Rigid3()
            f.center[:] = view * delta

            self.frame = self.frame * f

    @coroutine
    def mouse_rotate(self, start):
        '''rotate camera from mouse move events'''

        start = start.pos()

        start_frame = Rigid3()
        start_frame[:] = self.frame

        Pinv, ok = self.projection.inverted()

        sx, sy = self.pixel_coords(start.x(), start.y())
        s = start_frame(self.unproject(Pinv, sx, sy))

        while True:
            ev = yield

            ex, ey = self.pixel_coords(ev.pos().x(), ev.pos().y())
            e = start_frame(self.unproject(Pinv, ex, ey))

            f = Rigid3()
            f.orient = Quaternion.from_vectors(e - self.pivot,
                                               s - self.pivot)

            scale = norm(self.frame.center - self.pivot)

            f.orient = Quaternion.exp(scale * 16 * self.rotation_sensitivity * f.orient.log())

            t = Rigid3()
            t.center = self.pivot

            g = t * f * t.inv()

            next_frame = g * start_frame

            self.dframe = next_frame * self.frame.inv()
            self.frame[:] = next_frame

    @coroutine
    def mouse_drag(self, start):
        start_pos = start.pos()

        start_frame = Rigid3()
        start_frame[:] = self.frame

        Pinv, ok = self.projection.inverted()

        z = self.pixel_depth(start_pos.x(), start_pos.y())
        sx, sy = self.pixel_coords(start_pos.x(), start_pos.y())

        s = start_frame(self.unproject(Pinv, sx, sy, z))

        while True:
            ev = yield

            ex, ey = self.pixel_coords(ev.pos().x(), ev.pos().y())
            e = start_frame(self.unproject(Pinv, ex, ey, z))

            self.owner.drag(e)

    @coroutine
    def spin(self):
        '''rotate camera around pivot (damped) on each call to next'''

        delta = Rigid3()
        delta[:] = self.dframe

        while True:
            yield

            # 1% damping
            factor = 1.0 - self.damping
            vel = factor * delta.log()
            if norm(vel) < self.stop_velocity:
                break

            delta = Rigid3.Deriv.exp(vel)
            self.frame[:] = delta * self.frame

    @coroutine
    def slide(self):
        '''translate camera with constant local direction (damped) on each call to next'''
        delta = Rigid3()
        delta[:] = self.dframe

        while True:
            yield

            factor = 1.0 - self.damping
            vel = factor * delta.log()
            if norm(vel) < self.stop_velocity:
                break

            delta = Rigid3.Deriv.exp(vel)
            self.frame[:] = self.frame * delta


def draw_axis():

    glColor(1, 1, 1)
    gl.sphere(radius=0.025)

    glColor(1, 0.2, 0.2)
    with gl.lookat(ex):
        gl.arrow()

    glColor(0.2, 1, 0.2)
    with gl.lookat(ey):
        gl.arrow()

    glColor(0.2, 0.2, 1)
    with gl.lookat(ez):
        gl.arrow()


class Viewer(QtWidgets.QOpenGLWidget):

    alt_button = QtCore.Qt.CTRL if sys.platform == 'darwin' else QtCore.Qt.ALT

    def __init__(self, parent=None):
        super(Viewer, self).__init__(parent)

        self.camera = Camera(self)

        #
        self.mouse_move_handler = None
        self.mouse_wheel_handler = self.camera.mouse_zoom()
        self.draw_handler = None

        self.setWindowTitle('Viewer')

        # display flags
        # TODO make these properties and emit update_needed ?
        self.show_axis = True
        self.show_grid = False

        # animation
        self.animation = QtCore.QTimer()

        def on_timeout():

            self.animate()
            self.update()

        connect(self.animation, 'timeout()', on_timeout)
        self.fps = 60

        # a timer to post updateGL events
        self.update_timer = QtCore.QTimer()
        self.update_timer.setSingleShot(True)
        self.update_timer.setInterval(0)

        connect(self.update_timer, 'timeout()', self.update)
        # self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.update)

        # a nice signal to control it
        self.update_needed.connect(self.update_timer.start)


        

        
    update_needed = signal()

    @property
    def fps(self):
        return 1.0 / (self.animation.interval() / 1000.0)

    @fps.setter
    def fps(self, value):
        interval = (1.0 / value) * 1000.0
        self.animation.setInterval(interval)

    def minimumSizeHint(self):
        return QtCore.QSize(100, 300)

    def sizeHint(self):
        rec = QApplication.desktop().screenGeometry()

        # widget height is half screen height
        factor = 1.0 / 2.0

        height = factor * rec.height()

        # 4/3 form factor
        ratio = 4.0 / 3.0
        width = ratio * height

        return QtCore.QSize(width, height)

    def resizeGL(self, w, h):

        glViewport(0, 0, w, h)
        self.camera.ratio = float(w) / float(h if h != 0 else 1.0)

    def init(self): pass

    @property
    def context(self):
        return QtGui.QOpenGLContext.currentContext()
    
    def initializeGL(self):
        bg = QtGui.QColor.fromCmykF(0.39, 0.39, 0.0, 0.0).darker()

        ctx = self.context
        gl = ctx.functions()
        gl.glClearColor(bg.redF(), bg.greenF(), bg.blueF(), bg.alphaF())

        self.resizeGL(self.width(), self.height())

        # some reasonable defaults
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)

        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        glEnable(GL_NORMALIZE)

        self.init()

    # def update(self):
    #     if not self.animation.isActive():
    #         QtWidgets.QOpenGLWidget.update(self)

    def mouseMoveEvent(self, e):

        if self.mouse_move_handler:
            self.mouse_move_handler.send(e)
            self.update()

    def select(self, p): pass

    def drag(self, p): pass

    def mousePressEvent(self, e):
        self.draw_handler = None
        self.camera.dframe = Rigid3()

        if e.button() == QtCore.Qt.LeftButton:
            if e.modifiers() == QtCore.Qt.SHIFT:
                p = self.camera.point_under_pixel(e.pos().x(), e.pos().y())
                if p is not None:
                    self.select(self.camera.frame(p))
                    self.mouse_move_handler = self.camera.mouse_drag(e)

            else:
                self.mouse_move_handler = self.camera.mouse_rotate(e)
                self.update()

        if e.button() == QtCore.Qt.RightButton:
            if e.modifiers() == QtCore.Qt.SHIFT:
                p = self.camera.point_under_pixel(e.pos().x(), e.pos().y())
                if p is not None:
                    self.camera.pivot = self.camera.frame(p)
                    self.update()
            else:
                self.mouse_move_handler = self.camera.mouse_translate(e)
                self.update()

        if e.button() == QtCore.Qt.MiddleButton:
            self.mouse_move_handler = self.camera.mouse_zoom(e)
            self.update()

    def mouseDoubleClickEvent(self, e):

        if e.button() == QtCore.Qt.LeftButton:
            self.camera.axis_align()
            self.update()

    def animate(self): pass

    def reset(self): pass

    def toggle_fullscreen(self):
        if self.isFullScreen():
            self.showNormal()
        else:
            self.showFullScreen()

    def on_keypress(self, key): pass

    def keyPressEvent(self, e):
        if e.key() == QtCore.Qt.Key_Return:
            if e.modifiers() == Viewer.alt_button:
                self.toggle_fullscreen()
            else:
                if self.animation.isActive():
                    self.animation.stop()
                else:
                    self.animation.start()

        if e.key() == QtCore.Qt.Key_Escape:
            self.close()

        if e.key() == QtCore.Qt.Key_Backspace:
            self.reset()

        if e.text() == 'a':
            self.show_axis = not self.show_axis
            self.update()

        self.on_keypress(e.text())

    spin_threshold = 0.05
    slide_threshold = 0.04

    def mouseReleaseEvent(self, e):
        self.mouse_move_handler = None

        if e.button() == QtCore.Qt.LeftButton:

            if norm(self.camera.dframe.log()) > self.spin_threshold:
                self.draw_handler = self.camera.spin()
                self.update()

        if e.button() == QtCore.Qt.RightButton:

            if norm(self.camera.dframe.log()) / (1.0 + self.camera.pivot_distance) > self.slide_threshold:
                self.draw_handler = self.camera.slide()
                self.update()

    def wheelEvent(self, e):
        self.mouse_wheel_handler.send(e)
        self.update()

    def draw(self):
        pass

    def paintGL(self):
        glMatrixMode(GL_PROJECTION)
        glLoadMatrixd(self.camera.projection.data())

        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixd(self.camera.modelview.data())
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glPushMatrix()

        # axis/grid
        if self.show_axis:
            draw_axis()

        if self.draw_handler:
            try:
                next(self.draw_handler)
                self.update_needed.emit()
            except StopIteration:
                self.draw_handler = None

        self.draw()

        # or glFlush ?
        glPopMatrix()
        glFinish()


from contextlib import contextmanager

def exit_on_sigint(app):
    timeout = 100

    # install signal handler
    def handler(sig, frame):
        if sig == signal.SIGINT:
            app.quit()

    import signal
    signal.signal(signal.SIGINT, handler)
    
    # note: we need to nudge python interpreter every so often so that our
    # handler can run
    timer = QtCore.QTimer(app)
    timer.setInterval(timeout)
    timer.timeout.connect(lambda: None)
    timer.start()
    


def run(**kwargs):

    import sys

    main = sys.modules['__main__'].__dict__

    init = main.get('init', None)
    draw = main.get('draw', None)
    animate = main.get('animate', None)
    reset = main.get('reset', None)
    keypress = main.get('keypress', None)
    select = main.get('select', None)
    drag = main.get('drag', None)

    fullscreen = kwargs.get('fullscreen')
    
    class SimpleViewer(Viewer):

        def init(self):
            if init:
                init()

        def draw(self):
            if draw:
                draw()

        def select(self, p):
            if select:
                select(p)

        def reset(self):
            if reset:
                reset()
                self.update()

        def drag(self, p):
            if drag:
                drag(p)

        def animate(self):
            if animate:
                animate()

        def keyPressEvent(self, e):
            if keypress and keypress(e.text()):
                self.update()
            else:
                Viewer.keyPressEvent(self, e)

    with app() as self:

        w = SimpleViewer()
        w.show()

        if fullscreen:
            w.showFullScreen()

        exit_on_sigint(self)
