import snap

from PySide import QtGui

import sys

from threading import Thread

import OpenGL

from OpenGL.GL import *
from OpenGL.GLU import *

from snap import gl
import numpy as np

from snap.math import *

class Viewer(snap.Viewer):
    

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


    def init(self):
        pass

    def animate(self):
        import time
        print('animate', time.time())
        
    def draw(self):
        with gl.push_matrix():
            glTranslate(*self.camera.pivot)
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


