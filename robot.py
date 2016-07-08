import prout

from PySide import QtGui

import sys

from threading import Thread

import OpenGL

from OpenGL.GL import *
from OpenGL.GLU import *

from prout import gl
from prout.math import *

import time

class RigidBody(object):

    def __init__(self):

        self.mass = 1.0
        self.inertia = [1, 1, 1]
        self.name = 'unnamed body'

        # dimensions for drawing
        self.dim = [1, 1, 1]
        
        # body dofs
        self.frame = Rigid3()
        
class Joint(object):

    def __init__(self):

        self.parent_index = 0
        self.parent_frame = Rigid3()
        
        self.child_index = 0
        self.child_frame = Rigid3()

        # joint dofs
        self.dofs = Rigid3()
        
        self.name = 'unnamed joint'

class Robot(object):

    def __init__(self):


        # an array of 3 rigid bodies
        self.body = [ RigidBody() for i in range(3) ]

        # body properties
        self.body[0].name = 'basis'
        self.body[1].name = 'link1'
        self.body[2].name = 'link2'


        # configuration
        self.body[0].frame.center[2] = 2
        self.body[0].frame.center[1] = 0.5        
        self.body[0].dim = [0.1, 1, 0.1]

        
        self.body[1].dim = [0.1, 2, 0.1]
        self.body[2].dim = [0.1, 2, 0.1]        
        
        
        
        # an array of 2 joints
        self.joint = [ Joint() for i in range(2) ]

        self.joint[0].name = 'basis-link1'

        self.joint[0].parent_index = 0
        self.joint[0].child_index = 1

        # joint coordinates
        self.joint[0].parent_frame.center = [0, 0.5, 0]
        self.joint[0].child_frame.center = [0, -1, 0]        

        
        self.joint[1].name = 'link1-link2'

        self.joint[1].parent_index = 1
        self.joint[1].child_index = 2

        self.joint[1].parent_frame.center = [0, 1, 0]
        self.joint[1].child_frame.center = [0, -1, 0]        

        # root body
        self.root = 0
        
        # joint traversal order for forward kinematics (TODO compute
        # automatically)
        self.forward = [0, 1]

        
    def update(self):
        '''update body dofs from joint dofs (forward kinematics)'''

        for i in self.forward:
            j = self.joint[i]

            c = self.body[j.child_index]
            p = self.body[j.parent_index]            
            
            c.frame = p.frame * j.parent_frame * j.dofs * j.child_frame.inv()

            
    def draw(self):
        glColor(1, 1, 1)
        
        for b in self.body:
            # in body frame, aligned with y axis
            with gl.frame(b.frame):
                glScale(*b.dim)
                glTranslate(0, -0.5, 0)

                with gl.lookat(ey):
                    gl.cylinder()
                

class Viewer(prout.Viewer):

    def init(self):
        self.robot = Robot()
        self.animation.start()
        self.start_time = time.time()
        
    def animate(self):
        t = time.time() - self.start_time
        
        self.robot.joint[0].dofs.orient = Quaternion.exp( math.pi/4 * math.sin(t) * ez )
        self.robot.joint[1].dofs.orient = Quaternion.exp( math.pi/4 * math.sin(2 * t) * ez )               
        self.robot.update()
    
    def draw(self):
        self.robot.draw()
        
if __name__ == '__main__':
    
    app = QtGui.QApplication(sys.argv)

    w = Viewer()
    w.show()
    
    w.camera.frame.center[2] = 5

    sys.exit(app.exec_())


