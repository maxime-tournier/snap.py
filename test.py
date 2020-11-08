import gui
import Sofa as sofa
from Compliant import api
api.visual_model_classname = 'OglModel'

from snap.gl import *

class Scene(sofa.PythonController):
    def __init__(self, node):
        node.create_object('LDLTSolver')
        node.create_object('CompliantImplicitSolver')
        
        rigid = api.RigidBody.from_mesh(node, 'box',
                                        filename='mesh/raptor_8kp.obj')

    def onEndAnimationStep(self, dt):
        # print('time', self.context.time)
        pass

def load_module(name):
    try:
        sofa.load_plugins(name)
    except RuntimeError as e:
        pass
    

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    # parser.add_argument('filename')
    args = parser.parse_args()

    load_module('SofaGeneral')
    simu = sofa.DAGSimulation()
    
    with gui.init():
        scene = Scene(simu.root)

        simu.init()
        simu.root.dt = 1e-5
        # simu.root.init_visual()

        with gui.console(locals()):
            for events in gui.loop():
                simu.animate()
                draw = simu.async_draw()
                draw()
                
                glEnable(GL_LIGHTING)
                glEnable(GL_COLOR_MATERIAL)                
                axis()
                
