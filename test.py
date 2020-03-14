import pygame as pg

from snap.gl import *
from snap.math import *
from snap.tool import coroutine

from OpenGL.GLU import projection

import numpy as np

from contextlib import contextmanager
from itertools import repeat

import logging
log = logging.getLogger()
import traceback


default_size = (1024, 768)

def init():
    pg.display.init()
    pg.display.set_mode(default_size, pg.DOUBLEBUF | pg.OPENGL | pg.RESIZABLE)
    
    print('init')


    
def loop():
    def handle(ev):
        if ev.type == pg.QUIT:
            pg.quit()
            quit()
        return ev
            
    while True:
       glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
       
       yield filter(handle, pg.event.get())
       
       pg.display.flip()
       pg.time.wait(10)


def draw_axis():

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

        self.width, self.height = default_size

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
            glRotate(-theta / deg, *n)

        
    def resize(self, width, height):
        self.width = width
        self.height = height
        glViewport(0, 0, width, height)

    def mouse_win(self):
        return pg.mouse.get_pos()
        
    def mouse_cam(self, z=0):
        x, y = self.mouse_win()
        return np.array([2 * (x / self.width - 0.5), 2 * (-y / self.height - 0.5), z])
    
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
    rotate_factor = 2.0
    zoom_factor = 1.0

    def unproject(self, x, y, z=0):
        return np.array(projection.gluUnProject(x, y, z))
    
    def mouse_translate(self):
        init = self.frame.copy()
        factor = self.translate_factor * self.distance

        for dx in self.mouse_move():
            self.frame.center = init.center - init.orient(dx)
            yield
            
            
    def mouse_rotate(self):
        init = self.frame.copy()

        win = self.mouse_win()
        p = self.unproject(*win)
        
        x = self.mouse_cam()
        factor = self.rotate_factor / (1.0 + self.distance)

        for dx in self.mouse_move():
            omega = np.cross(p - self.target, init.orient(dx))
            quat = Quaternion.exp(omega * self.rotate_factor)

            twist = Rigid3.translation(self.target) * Rigid3.rotation(quat) * Rigid3.translation(-self.target)
            
            self.frame = twist * init
            yield
            # self.frame = Rigid3.rotation(Quaternion.exp(dx * ey)) * init
            # print(self.frame.center, self.view)


    def mouse_zoom(self):
        init = self.frame.copy()
        factor = self.zoom_factor * self.distance
        
        for dx, dy in self.mouse_move():
            self.frame = init * Rigid3.translation(-factor * dy * ez)
            yield

    def mouse_zoom_wheel(self, amount):
        init = self.frame.copy()
        factor = self.zoom_factor * self.distance

        self.frame = init * Rigid3.translation(-factor * amount * ez)
        yield
            
    def mouse_left(self):
        yield from self.mouse_rotate()

    def mouse_right(self):
        yield from self.mouse_translate()

    def mouse_middle(self):
        yield from self.mouse_zoom()

    wheel_factor = 0.1
    
    def mouse_wheel(self, up):
        amount = self.wheel_factor if up else -self.wheel_factor
        yield from self.mouse_zoom_wheel(amount)
        
        
    @coroutine
    def mouse(self, button):
        print('button:', button)
        if button == 1:
            yield from self.mouse_left()
        if button == 2:
            yield from self.mouse_middle()
        elif button == 3:
            yield from self.mouse_right()
        elif button == 4:
            yield from self.mouse_wheel(up=True)
        elif button == 5:
            yield from self.mouse_wheel(up=False)            
        else:
            yield from repeat(None)

            
            
def camera_loop():
    camera = Camera()

    class State: pass
    state = State()
    state.mouse = None

    def handle(event):
        if event.type == pg.VIDEORESIZE:
            camera.resize(event.w, event.h)
        elif event.type == pg.MOUSEBUTTONDOWN:
            state.mouse = camera.mouse(event.button)
        elif event.type == pg.MOUSEBUTTONUP:
            state.mouse = None
        elif event.type == pg.MOUSEMOTION:                
            if state.mouse: next(state.mouse)
        else:
            return event

    for events in loop():
        events = filter(handle, events)
        try:
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            camera.projection()

            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()            
            camera.modelview()

            if state.mouse:
                draw_axis()
            
            glPushMatrix()
            yield events
        finally:
            glPopMatrix()
       
init()
glClearColor(1, 0, 0, 1)
glEnable(GL_LIGHTING)
glEnable(GL_DEPTH_TEST)
glEnable(GL_LIGHTING)
glEnable(GL_LIGHT0)
glEnable(GL_NORMALIZE)
glEnable(GL_COLOR_MATERIAL)

def reload():
    import sys
    import os
    os.execl(sys.executable, __file__, *sys.argv)

for events in camera_loop():
    try:
        for ev in events:
            if ev.type == pg.KEYDOWN:
                print('key down:', ev.key)
                if ev.key == ord('r'):
                    reload()
                if ev.key == pg.K_ESCAPE:
                    pg.quit()
                    quit()

        glTranslate(0, 0, 0)
        s = 0.1
        glScale(s, s, s)
        glColor(1, 1, 1)
        cube()
    except:
        log.error(traceback.format_exc())
