import pygame as pg

from snap import gl, tool
from itertools import repeat

import logging
logging.basicConfig(level=logging.INFO)
log = logging.getLogger('gui')

import traceback

default_size = (1024, 768)

def init():
    pg.display.init()
    pg.display.set_mode(default_size, pg.DOUBLEBUF | pg.OPENGL | pg.RESIZABLE)

    bg = 0.2
    
    gl.glClearColor(bg, bg, bg, 1)
    gl.glEnable(gl.GL_LIGHTING)
    gl.glEnable(gl.GL_DEPTH_TEST)
    gl.glEnable(gl.GL_LIGHTING)
    gl.glEnable(gl.GL_LIGHT0)
    gl.glEnable(gl.GL_NORMALIZE)
    gl.glEnable(gl.GL_COLOR_MATERIAL)

    log.info('initialized')

class Quit(Exception): pass
    
def event_loop():
    def handle(ev):
        if ev.type == pg.QUIT:
            raise Quit()
        return ev
            
    while True:
       gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
       
       yield filter(handle, pg.event.get())
       
       pg.display.flip()
       pg.time.wait(10)

       

class Camera(gl.Camera):

    def __init__(self):
        gl.Camera.__init__(self)
        self.resize(*default_size)
    
    def mouse_win(self):
        return pg.mouse.get_pos()

    def mouse_left(self):
        yield from self.mouse_trackball()

    def mouse_right(self):
        yield from self.mouse_translate()

    def mouse_middle(self):
        yield from self.mouse_zoom()

    def mouse_wheel(self, up):
        amount = 1.0 if up else -1.0
        yield from self.mouse_zoom_wheel(amount)
        
    @tool.coroutine
    def mouse(self, button):

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

        
@tool.coroutine    
def viewer_loop():
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

    events = None
    while True:
        events = yield events
        events = filter(handle, events)
        with camera.draw():
            if state.mouse:
                camera.draw_axis()

def reload():
    import sys
    import os
    os.execl(sys.executable, __file__, *sys.argv)


def loop():
    init()

    def handle(event):
        if event.type == pg.KEYDOWN:
            log.debug('key down:', event.key)
            if event.key == ord('r'):
                reload()
            if event.key == pg.K_ESCAPE:
                raise Quit()
        return event
            

    for events in tool.compose(event_loop(), viewer_loop()):
        try:
            yield list(filter(handle, events))
        except Quit:
            pg.quit()
            quit()
        except:
            log.error(traceback.format_exc())
    
            
if __name__ == '__main__':    
    for events in loop():

        gl.glTranslate(0, 0, 0)
        s = 0.1
        gl.glScale(s, s, s)
        gl.glColor(1, 1, 1)
        gl.cube()
