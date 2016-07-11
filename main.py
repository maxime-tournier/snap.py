import youpy

from PySide import QtGui

import sys

from threading import Thread

import OpenGL

from OpenGL.GL import *
from OpenGL.GLU import *

from youpy import gl
import numpy as np

from youpy.math import *

class Viewer(youpy.Viewer):
    

    def draw_cross(self):

        glColor(0.7, 0.7, 0.7)
        
        with gl.push_matrix():
            glScale(0.2, 0.2, 0.2)
            with gl.disable(GL_LIGHTING), gl.begin(GL_LINES):
                glVertex(-1, 0, 0)
                glVertex(1, 0, 0)

                glVertex(0, -1, 0)
                glVertex(0, 1, 0)

                glVertex(0, 0, -1)
                glVertex(0, 0, 1)


    def test(self):
        self.target = 2 * (2 * np.random.rand(3) - np.ones(3))


        self.camera.frame.center[2] = -5
        self.target = vec(0, 0, 1)        
        self.camera.lookat( self.target )
        

    def init(self):
        self.test()

    def keyPressEvent(self, e):
        youpy.Viewer.keyPressEvent(self, e)

        self.test()
        self.updateGL()

    def animate(self):
        import time
        print('animate', time.time())
        
    def draw(self):
        print('draw')
        glLineWidth(3.0)

        # glColor(0.4, 0.4, 1.0)
        # gl.sphere()

        with gl.push_matrix():
             glTranslate(*self.target)
             glColor(0, 0, 0)
             self.draw_cross()

        
if __name__ == '__main__':
    
    app = QtGui.QApplication(sys.argv)

    def console(local):
        
        import code
        
        code.interact(local = local)
        app.quit()
        
    thread = Thread(target = console, args = (locals(), ))
    thread.daemon = True
    thread.start()

    w = Viewer()

    
    w.show()


    
    def cleanup():
        import os
        os.system('stty sane')
        
    app.aboutToQuit.connect( cleanup )
    
    sys.exit(app.exec_())


