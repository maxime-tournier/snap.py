from PySide2 import QtCore, QtGui, QtOpenGL, QtWidgets
from PySide2.QtWidgets import QApplication

def connect(src, sig, dst):
    src.connect(src, QtCore.SIGNAL(sig), dst)

def signal():
    return QtCore.Signal()

def wheel_angle(ev):
    return ev.delta()
    
        
from contextlib import contextmanager
import sys

@contextmanager
def app():
    res = QApplication(sys.argv)
    
    try:
	    yield res
    finally:
	    res.exec_()

