from __future__ import print_function
import numpy as np

from numpy.polynomial.polynomial import Polynomial

from snap.math import *
from snap.gl import *

from snap import viewer, tool, spline




    

n = 10

nodes = np.linspace(-100, 100, n)
points = np.random.rand(n, 3)

m = 1000       
sampled_nodes = np.linspace(nodes[0], nodes[-1], m)


quats = np.random.rand(n, 4) - 0.5

for i in range(n):
    quats[i].view(Quaternion).normalize()

frame = 0



quat_spline = spline.Spline(nodes, quats, spline.RotationGroup() )
vec3_spline = spline.Spline(nodes, points, spline.VectorSpace(3))

sampled_quats = [quat_spline(x)[0] for x in sampled_nodes]
sampled_omegas = [quat_spline(x)[1] for x in sampled_nodes]



class State(object): pass
state = State()

state.q = Quaternion()
state.dq = np.zeros(3)

state.p = np.zeros(3)
state.dp = np.zeros(3)

state.frame = 0

sampled_points = [vec3_spline(x)[0] for x in sampled_nodes]
sampled_dpoints = [vec3_spline(x)[1] for x in sampled_nodes]


def animate():

    state.frame = (state.frame + 1) % m

    state.q[:] = sampled_quats[state.frame]
    state.dq[:] = sampled_omegas[state.frame]

    state.p[:] = sampled_points[state.frame]
    state.dp[:] = sampled_dpoints[state.frame]
    
    
def draw():
    m = 100

    glDisable(GL_LIGHTING)
    glPointSize(5)

    # interpolated point
    glBegin(GL_POINTS)    
    glColor(1, 0, 1)
    glVertex(state.p)
    glEnd()


    # interpolated tangent
    glBegin(GL_LINES)    
    glColor(1, 0, 1)
    glVertex(state.p - state.dp / 2)
    glVertex(state.p + state.dp / 2)    
    glEnd()

    

    # curve + control points
    glBegin(GL_POINTS)

    glColor(1, 0, 0)
    for p in points:
        glVertex(p)
    glEnd()
    
    glBegin(GL_LINE_STRIP)    
    glColor(1, 1, 1)
    
    for p in sampled_points:
        glVertex(p)
    glEnd()

    
    
    glEnable(GL_LIGHTING)

    # interpolated frame
    with push_matrix():
        rotate(state.q)
        viewer.draw_axis()

    # # angular velocity
    with lookat(state.dq):
        glColor(1, 0, 1)
        arrow(height = norm(state.dq))        
            

        
if __name__ == '__main__':
    viewer.run()




