from __future__ import print_function, absolute_import 

try:
    from PySide import QtCore, QtGui, QtOpenGL
    from PySide.QtGui import QApplication
    
    def connect(src, sig, dst):
	    src.connect(src, QtCore.SIGNAL(sig), dst)
        
    def signal():
	    return QtCore.Signal()

    def wheel_angle(ev):
        return ev.delta()
    
except ImportError as e:
    import PyQt5
    from PyQt5 import QtCore, QtGui, QtOpenGL, QtWidgets
    from PyQt5.QtWidgets import QApplication

    def connect(src, sig, dst):
        name = sig.split('(')[0]
        getattr(src, name).connect(dst)

    def signal():
        return QtCore.pyqtSignal()

    def wheel_angle(ev):
        return ev.angleDelta().y()
    
        
from contextlib import contextmanager
import sys

@contextmanager
def app():
    res = QApplication(sys.argv)
    
    try:
	    yield res
    finally:
	    res.exec_()

