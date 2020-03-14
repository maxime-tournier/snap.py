import gui
from snap.gl import *

if __name__ == '__main__':    
    for events in gui.loop():
        glTranslate(0, 0, 0)
        s = 0.1
        glScale(s, s, s)
        glColor(1, 1, 1)
        cube()
