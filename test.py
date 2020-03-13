import pygame as pg

from snap.gl import *
from snap.math import *
from snap.tool import coroutine

import numpy as np

from contextlib import contextmanager
from itertools import repeat

default_size = (1024, 768)


def init():
    pg.display.init()
    pg.display.set_mode(default_size, pg.DOUBLEBUF | pg.OPENGL | pg.RESIZABLE)
    
    print('init')


    
def loop():
    def forward(events):
        for ev in events:
            if ev.type == pg.QUIT:
                pg.quit()
            else:
                yield ev
    
    while True:
       glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
       
       yield forward(pg.event.get())
       pg.display.flip()
       pg.time.wait(10)

       

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

    def mouse_pos(self):
        return pg.mouse.get_pos()
        
    @coroutine
    def mouse_move(self):
        init = self.mouse_pos()

        delta = None
        
        while True:
            current = yield delta
            dx, dy = np.array(current) - np.array(init)
            delta = dx / self.width, -dy / self.height

    @property
    def distance(self):
        return np.linalg.norm(self.target - self.frame.center)
            
    translate_factor = 1.0
    rotate_factor = 2.0
    zoom_factor = 1.0
    
    def mouse_translate(self):
        init = self.frame.copy()
        mouse_move = self.mouse_move()

        factor = self.translate_factor * self.distance
        
        while True:
            info = yield None
            dx, dy = mouse_move.send(info)
            self.frame.center = init.center - init.orient(vec(factor * dx, factor * dy, 0))

            
    def mouse_rotate(self):
        init = self.frame.copy()
        mouse_move = self.mouse_move()

        factor = self.rotate_factor / (1.0 + self.distance)

        while True:
            info = yield None
            dx, dy = mouse_move.send(info)
            dv = vec(dx, dy, 0)

            omega = np.cross(init.center - self.target, init.orient(dv))
            quat = Quaternion.exp(omega * self.rotate_factor)

            twist = Rigid3.translation(self.target) * Rigid3.rotation(quat) * Rigid3.translation(-self.target)
            
            self.frame = twist * init
            # self.frame = Rigid3.rotation(Quaternion.exp(dx * ey)) * init
            # print(self.frame.center, self.view)


    def mouse_zoom(self):
        init = self.frame.copy()
        mouse_move = self.mouse_move()

        factor = self.zoom_factor * self.distance
        
        while True:
            info = yield None
            dx, dy = mouse_move.send(info)

            self.frame = init * Rigid3.translation(-factor * dy * ez)

            
    def mouse_left(self):
        yield from self.mouse_rotate()

    def mouse_right(self):
        yield from self.mouse_translate()

    def mouse_middle(self):
        yield from self.mouse_zoom()

        
    @coroutine
    def mouse(self, button):
        print('button:', button)
        if button == 1:
            yield from self.mouse_left()
        if button == 2:
            yield from self.mouse_middle()
        elif button == 3:
            yield from self.mouse_right()
        else:
            yield from repeat(None)

            
def camera_loop():
    camera = Camera()

    mouse = None
    
    for events in loop():
        for event in events:
            if event.type == pg.VIDEORESIZE:
                camera.resize(event.w, event.h)
            if event.type == pg.MOUSEBUTTONDOWN:
                mouse = camera.mouse(event.button)
            if event.type == pg.MOUSEBUTTONUP:
                mouse = None
            if event.type == pg.MOUSEMOTION:                
                if mouse: mouse.send(pg.mouse.get_pos())
                
            
        try:
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            camera.projection()

            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()            
            camera.modelview()

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
        
for events in camera_loop():
    for ev in events:
        if ev.type == pg.KEYDOWN:
            if ev.key == pg.K_ESCAPE:
                pg.quit()
                quit()
            
    glTranslate(0, 0, 0)
    s = 0.1
    glScale(s, s, s)
    cube()
