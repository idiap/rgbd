"""
Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch/
Written by Kenneth Funes <kenneth.funes@idiap.ch>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>
"""

from PySide import QtCore, QtOpenGL
from viewer import Viewer

class InteractiveViewer(Viewer):
    """
    Interactive extension of the viewer widget

    This class contains an extension of the viewer class, in which a group of meshes/vertices/models are displayed and
    and their position are all modified simultaneously using input from the keyboard and mouse

    """
    """ Signals for the parent class """
    xRotationChanged = QtCore.Signal(int)
    yRotationChanged = QtCore.Signal(int)
    zRotationChanged = QtCore.Signal(int)

    def __init__(self, parent=None):
        """ The constructor of the class """
        super(InteractiveViewer, self).__init__(parent)
        self.len = 1.0
        self.lastPos = QtCore.QPoint()

    def setXRotation(self, angle):
        """ Rotates the view from the given angle along x-axis """
        angle = self.normalizeAngle(angle)
        if angle != self.pose[3]:
            self.pose[3] = angle
            self.xRotationChanged.emit(angle)
            self.updateGL()

    def setYRotation(self, angle):
        """ Rotates the view from the given angle along y-axis """
        angle = self.normalizeAngle(angle)
        if angle != self.pose[4]:
            self.pose[4] = angle
            self.yRotationChanged.emit(angle)
            self.updateGL()

    def setZRotation(self, angle):
        """ Rotates the view from the given angle along z-axis """
        angle = self.normalizeAngle(angle)
        if angle != self.pose[5]:
            self.pose[5] = angle
            self.zRotationChanged.emit(angle)
            self.updateGL()

    def mouseMoveEvent(self, event):
        """ Detects mouse movements """
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()

        if event.buttons() & QtCore.Qt.LeftButton:
            self.setXRotation(self.pose[3] + 0.5 * dy)
            self.setYRotation(self.pose[4] + 0.5 * dx)
        elif event.buttons() & QtCore.Qt.RightButton:
            self.setXRotation(self.pose[3] + 0.5 * dy)
            self.setZRotation(self.pose[5] + 0.5 * dx)
        self.lastPos = event.pos()

    def keyPressEvent(self, event):
        """ Detects keys pressed  """
        key = event.key()
        if key == QtCore.Qt.Key_L:
          self.renderMode = 'L'
        if key == QtCore.Qt.Key_F:
          self.renderMode = 'F'
        if key == QtCore.Qt.Key_P:
          self.renderMode = 'P'
        if key == QtCore.Qt.Key_T:
          self.renderMode = 'T'
        if key == QtCore.Qt.Key_Up:
          self.pose[1] += (1.0/150)
        elif key == QtCore.Qt.Key_Down:
          self.pose[1] -= (1.0/150)
        elif key == QtCore.Qt.Key_Left:
          self.pose[0] -= (1.0/150)
        elif key == QtCore.Qt.Key_Right:
          self.pose[0] += (1.0/150)
        else:
          QtOpenGL.QGLWidget.keyPressEvent(self,event)
        self.updateGL()

    def mousePressEvent(self, event):
        """ Detects mouse pressed """
        self.setFocus()
        self.lastPos = event.pos()
        
    def wheelEvent(self, event):
       """ Detects wheel scrolled """
       delt = event.delta()
       if delt>=0:
         self.pose[6] = self.pose[6]*1.05
       else:
         self.pose[6] = self.pose[6]/1.05
       #self.updateMesh()
       #self.objects[0].updateVertices()
       self.updateGL()

    def normalizeAngle(self, angle):
        """ Normalized an angle so it is between 0 and 360 degrees """
        while angle < 0:
            angle += 360 * 16
        while angle > 360 * 16:
            angle -= 360 * 16
        return angle

