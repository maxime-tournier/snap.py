import prout

from PySide import QtGui

import sys

from threading import Thread

import OpenGL

from OpenGL.GL import *
from OpenGL.GLU import *


class Viewer(prout.Viewer):

    def draw_frame(self):
        glDisable(GL_LIGHTING)
        glBegin(GL_LINES)

        glColor(1, 0, 0)
        glVertex(0, 0, 0)
        glVertex(1, 0, 0)

        glColor(0, 1, 0)
        glVertex(0, 0, 0)
        glVertex(0, 1, 0)

        glColor(0, 0, 1)        
        glVertex(0, 0, 0)
        glVertex(0, 0, 1)
        glEnd()
        glEnable(GL_LIGHTING)


    def draw_cross(self):
        glDisable(GL_LIGHTING)
        glBegin(GL_LINES)

        glVertex(-1, 0, 0)
        glVertex(1, 0, 0)

        glVertex(0, -1, 0)
        glVertex(0, 1, 0)

        glVertex(0, 0, -1)
        glVertex(0, 0, 1)
        glEnd()
        glEnable(GL_LIGHTING)
        


    def init(self):
        glEnable(GL_DEPTH_TEST)
        


    def animate(self):
        import time
        print('animate', time.time())
        
    def draw(self):
        
        glLineWidth(3.0)
        self.draw_frame()

        
        glPushMatrix()
        glTranslate(*self.camera.pivot)
        glColor(0, 0, 0)
        self.draw_cross()
        glPopMatrix()

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
    w.camera.frame.center[2] = 5

    
    def cleanup():
        import os
        os.system('stty sane')
        
    app.aboutToQuit.connect( cleanup )
    
    sys.exit(app.exec_())


