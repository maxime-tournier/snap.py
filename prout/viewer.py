from PySide import QtCore, QtGui, QtOpenGL

import OpenGL

from OpenGL.GL import *
from OpenGL.GLU import *

from .math import *
from .tool import *

class Camera(object):
    
    def __init__(self, owner):

        self.owner = owner
        
        self.frame = Rigid3()
        self.pivot = vec(0, 0, -2)
        
        self.znear = 0.1
        self.zfar = 100.0
        self.vfov = 60.0;

        # width / height
        self.ratio = 1.0

        self.rotation_sensitivity = 1
        self.translation_sensitivity = 1
        self.zoom_sensitivity = 1
        
        self.cb = None
        
        
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
        if angle > 1e-5:
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

            self.frame = start_frame * f.inv()


    def clamp_to_axis(self):

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
            
            self.frame[:] = g * start_frame

            
class Viewer(QtOpenGL.QGLWidget):
    
    def __init__(self, parent=None):
        super(Viewer, self).__init__(parent)

        self.camera = Camera(self)

        self.mouse_move_handler = None
        self.mouse_wheel_handler = self.camera.zoom()

        self.setWindowTitle('Viewer')

        
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
        self.init()


    def mouseMoveEvent(self, ev):
        if self.mouse_move_handler:
            self.mouse_move_handler.send( ev )
            self.updateGL()


    def mousePressEvent(self, ev):
        if ev.button() == QtCore.Qt.LeftButton:
            self.mouse_move_handler = self.camera.rotate(ev)
        if ev.button() == QtCore.Qt.RightButton:
            self.mouse_move_handler = self.camera.translate(ev)


    def mouseDoubleClickEvent(self, ev):

        if ev.button() == QtCore.Qt.LeftButton:
            self.camera.clamp_to_axis()

        self.updateGL()
        
    def mouseReleaseEvent(self, ev):
        self.mouse_move_handler = None

    def wheelEvent(self, ev):
        self.mouse_wheel_handler.send(ev)
        self.updateGL()


    def draw(self):
        pass
        
    def paintGL(self):
        glMatrixMode(GL_PROJECTION)
        glLoadMatrixd(self.camera.projection.data())
        
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixd(self.camera.modelview.data())
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glPushMatrix()
        
        self.draw()

        # debug
        if self.camera.cb: self.camera.cb()
        
        # or glFlush ?
        glPopMatrix()
        glFinish()



    
