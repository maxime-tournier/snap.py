# MIT license
# tournier.maxime@gmail.com

from PySide import QtCore, QtGui, QtOpenGL

import OpenGL

from OpenGL.GL import *
from OpenGL.GLU import *

from .math import *
from .tool import *

from . import gl

import time

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
        self.vfov = 60.0;

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
        res.translate( *inv.center )
        
        axis, angle = inv.orient.axis_angle()
        if axis is not None:
            res.rotate(angle * deg, *axis)
            
        return res


    def normalize(self, x, y):
        '''normalize mouse coodinates'''
        
        P = self.projection
        v = QtGui.QVector4D(1, -1, -self.frame.center[2], 1)

        u = P.map(v)

        rx = float(x) / float(self.owner.width())
        ry = float(y) / float(self.owner.height())

        ry = 1.0 - ry

        return rx, ry


    def unproject(self, Pinv, x, y, z = 0):

        d = 2.0 * QtGui.QVector4D(x, y, z, 1.0) - QtGui.QVector4D(1, 1, 1, 1)
        res = Pinv.map(d)
        
        return vec(res.x(), res.y(), res.z())
    

    @coroutine
    def translate(self, start):

        start_pos = start.pos()

        start_frame = Rigid3()
        start_frame[:] = self.frame

        unprojection, ok = self.projection.inverted()
        assert ok

        z = self.znear
        z = (z - self.znear) / (self.zfar - self.znear)

        sx, sy = self.normalize(start_pos.x(), start_pos.y())
        s = self.unproject(unprojection, sx, sy, z)

        while True:
            ev = yield

            ex, ey = self.normalize(ev.pos().x(), ev.pos().y())
            e = self.unproject(unprojection, ex, ey, z)

            d = e - s

            scale = norm(self.frame.center - self.pivot)

            f = Rigid3()
            f.center = scale * self.translation_sensitivity * d 

            next_frame = start_frame * f.inv()

            self.dframe = self.frame.inv() * next_frame
            self.frame = next_frame


    def axis_align(self):

        pos = np.abs(self.frame.center - self.pivot).tolist()
        
        index = pos.index( max(pos) )

        self.frame.center = [ self.pivot[i] if i != index else self.frame.center[i]
                              for i in range(3)]
        
        local_pivot = self.frame.inv()(self.pivot)

        # look at pivot
        q = Quaternion.from_vectors(-ez, local_pivot)
        self.frame.orient = self.frame.orient * q


        # pick axis closest to camera up axis
        cam_up = self.frame.orient(ey)
        proj = np.abs(cam_up).tolist()
        index = proj.index( max(proj) )
        
        up = vec(0, 0, 0)
        up[index] = proj[index] / cam_up[index] # to get sign right
        
        q = Quaternion.from_vectors(ey, self.frame.orient.inv()(up))
        self.frame.orient = self.frame.orient * q
        
        
    def lookat(self, target, **kwargs):

        local_target = self.frame.inv()(target)
        q = Quaternion.from_vectors(-ez, local_target)
        
        self.frame.orient = self.frame.orient * q
        
        # project q.inv() onto geodesic around z
        up = kwargs.get('up', ey)
        qinv = Quaternion.from_vectors(ey, self.frame.orient.inv()(up))

        if math.fabs(q.real) < 1e-5: return

        # gnomonic projection
        pqinv = qinv / q.real
        
        r = Quaternion()
        r.imag[2] = pqinv.imag[2]
        r /= norm(r)
        
        self.frame.orient  = self.frame.orient * r
        
        
    @coroutine
    def zoom(self):

        while True:
            ev = yield
            degrees = float(ev.delta()) / 256.0

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
    def rotate(self, start):

        start_pos = start.pos()

        start_frame = Rigid3()
        start_frame[:] = self.frame

        Pinv, ok = self.projection.inverted()

        sx, sy = self.normalize(start_pos.x(), start_pos.y())
        s = start_frame( self.unproject(Pinv, sx, sy) )

        
        while True:
            ev = yield

            ex, ey = self.normalize(ev.pos().x(), ev.pos().y())
            e = start_frame( self.unproject(Pinv, ex, ey) )

            f = Rigid3()
            f.orient = Quaternion.from_vectors(e - self.pivot,
                                               s - self.pivot)

            scale = norm(self.frame.center - self.pivot)
            
            f.orient = Quaternion.exp( scale * self.rotation_sensitivity * f.orient.log() )
            
            t = Rigid3()
            t.center = self.pivot

            g = t * f * t.inv()

            next_frame = g * start_frame

            self.dframe = next_frame * self.frame.inv()
            self.frame[:] = next_frame




    @coroutine
    def spin(self):

        delta = Rigid3()
        delta[:] = self.dframe 

        while True:
            yield

            # 1% damping
            factor = 1.0 - self.damping
            vel = factor * delta.log()
            if norm(vel) < self.stop_velocity: break
            
            delta = Rigid3.exp( vel )
            self.frame[:] = delta * self.frame



    @coroutine            
    def slide(self):
        delta = Rigid3()
        delta[:] = self.dframe 

        while True:
            yield

            factor = 1.0 - self.damping
            vel = factor * delta.log()
            if norm(vel) < self.stop_velocity: break
            
            delta = Rigid3.exp( vel )
            self.frame[:] = self.frame * delta


            
class Viewer(QtOpenGL.QGLWidget):
    
    def __init__(self, parent=None):
        super(Viewer, self).__init__(parent)

        self.camera = Camera(self)

        # 
        self.mouse_move_handler = None
        self.mouse_wheel_handler = self.camera.zoom()
        self.draw_handler = None
        
        self.setWindowTitle('Viewer')

        # display flags
        # TODO make these properties and emit update_needed
        self.show_axis = True
        self.show_grid = False
        

        # animation
        self.animation = QtCore.QTimer()

        def on_timeout():

            self.animate()
            self.updateGL()
            
        self.connect(self.animation, QtCore.SIGNAL("timeout()"), on_timeout)
        self.fps = 60

        # a timer to post updateGL events
        self.update_timer = QtCore.QTimer()
        self.update_timer.setSingleShot( True )
        self.update_timer.setInterval(0)
        self.connect(self.update_timer, QtCore.SIGNAL("timeout()"), self.update)

        # a nice signal to control it
        self.update_needed.connect(self.update_timer.start)
        
    update_needed = QtCore.Signal()
    
    @property
    def fps(self):
        return 1.0 / (self.animation.interval() / 1000.0)
    
    @fps.setter
    def fps(self, value):
        interval = (1.0 / value) * 1000.0
        self.animation.setInterval( interval )

    def minimumSizeHint(self):
        return QtCore.QSize(100, 300)

    def sizeHint(self):
        return QtCore.QSize(400, 400)


    def resizeGL(self, w, h):

        glViewport(0, 0, w, h)
        self.camera.ratio = float(w) / float( h if h != 0 else 1.0 )
        
    def init(self): pass
    
    def initializeGL(self):
        bg = QtGui.QColor.fromCmykF(0.39, 0.39, 0.0, 0.0).darker()
        
        self.qglClearColor(bg)

        self.resizeGL(self.width(), self.height())

        # some reasonable defaults
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)

        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)

        glEnable(GL_NORMALIZE)

        self.init()


    def update(self):
        if not self.animation.isActive():
            self.updateGL()

    def mouseMoveEvent(self, e):
        
        if self.mouse_move_handler:
            self.mouse_move_handler.send( e )
            self.update()
            
    def mousePressEvent(self, e):
        self.draw_handler = None
        self.camera.dframe = Rigid3()
        
        if e.button() == QtCore.Qt.LeftButton:
            self.mouse_move_handler = self.camera.rotate(e)
            self.update()
            
        if e.button() == QtCore.Qt.RightButton:
            self.mouse_move_handler = self.camera.translate(e)
            self.update()


    def mouseDoubleClickEvent(self, e):

        if e.button() == QtCore.Qt.LeftButton:
            self.camera.axis_align()
            self.update()


    def animate(self): pass



    def keyPressEvent(self, e):

        if e.key() == QtCore.Qt.Key_Return:
            if self.animation.isActive(): self.animation.stop()
            else: self.animation.start()

        if e.key() == QtCore.Qt.Key_Escape:
            self.close()
        
    def mouseReleaseEvent(self, e):
        self.mouse_move_handler = None

        if e.button() == QtCore.Qt.LeftButton:
            
            if norm(self.camera.dframe.log()) > 0.1:
                self.draw_handler = self.camera.spin()
                self.update()
                
        if e.button() == QtCore.Qt.RightButton:
            
            if norm(self.camera.dframe.log()) > 0.1:
                self.draw_handler = self.camera.slide()
                self.update()
                

    def wheelEvent(self, e):
        self.mouse_wheel_handler.send(e)
        self.update()


    def draw(self):
        pass


    def draw_axis(self):

        glColor(1, 1, 1)
        gl.sphere(radius = 0.025)
        
        glColor(1, 0.2, 0.2)
        with gl.lookat(ex):
            gl.arrow()            
        
        glColor(0.2, 1, 0.2)
        with gl.lookat(ey):
            gl.arrow()            
        
        glColor(0.2, 0.2, 1)
        with gl.lookat(ez):
            gl.arrow()            

    
    def paintGL(self):
        glMatrixMode(GL_PROJECTION)
        glLoadMatrixd(self.camera.projection.data())
        
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixd(self.camera.modelview.data())
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glPushMatrix()

        # axis/grid
        if self.show_axis: self.draw_axis()
        
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

        

        


    
