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
from interactiveViewer import InteractiveViewer
from PySide import QtCore, QtGui

class RGBDViewer(QtGui.QMainWindow):
    """
    Class to visualize and render a stream of RGB-D data
    """
    updateSignal = QtCore.Signal()
    def __init__(self, qtApp, close_callback = None, render3D=True, showRGB=True):
        super(RGBDViewer, self).__init__()
        self.mainLayout = QtGui.QHBoxLayout()
        self.qtApp = qtApp
        self.rgbLabel = QtGui.QLabel()
        # A widget where there is some control over what is being displayed.
        self.control_widget = QtGui.QWidget()
        self.control_layout = QtGui.QVBoxLayout()
        # An example of such control is the "pauseButton"
        self.pauseButton = QtGui.QPushButton('Pause/Unpause')
        # Whi is now added to the control widget
        self.control_layout.addWidget(self.pauseButton)
        self.control_widget.setLayout(self.control_layout)
        self.mainLayout.addWidget(self.control_widget)

        # Create the widget which allows to do OpenGL rendering
        self.glWidget = InteractiveViewer()
        self.glWidget.setFixedSize(600, 600)
        self.glWidget.ambient = (0.5,0.5,0.5,1.0)
        self.glWidget.position = (1.0,1.0,2.0)
        self.glWidget.diffuse = (1.0,1.0,1.0,1.0)
        self.glWidget.qtApp = qtApp
        # Make the connection of the update signal
        self.updateSignal.connect(self.glWidget.updateGL)
        widget = QtGui.QWidget()
        widget.setLayout(self.mainLayout)
        self.setCentralWidget(widget)
        # Whenever this "main window" is closed, then this function will be called
        self.close_callback = close_callback
        self.meshId = None

        self.showRGB = showRGB
        self.render3D = render3D

        self.rgbLabel.setMinimumSize(QtCore.QSize(640, 480))
        self.rgb_size = 400, 400

        self.mainLayout.addWidget(self.rgbLabel)

        self.glWidget.show()
        if not render3D:
            self.glWidget.hide()
            #self.mainLayout.addWidget(self.glWidget)

    def setNewData(self, frameData, frameIndex):
        frameMesh = frameData[4]
        if self.meshId is not None:
            self.glWidget.removeObject(self.meshId)
            self.meshId = None
        if self.render3D:
            self.meshId = self.glWidget.addObject(frameMesh)
        if self.showRGB:
            rgb_numpy = frameData[0]
            image = QtGui.QImage(rgb_numpy, rgb_numpy.shape[1], rgb_numpy.shape[0], QtGui.QImage.Format_RGB888)
            self.rgbLabel.setPixmap(QtGui.QPixmap.fromImage(image))
            if rgb_numpy.shape[1] != self.rgb_size[0] or rgb_numpy.shape[0] != self.rgb_size[1]:
                self.rgb_size = rgb_numpy.shape[1], rgb_numpy.shape[0]
                self.resize(QtCore.QSize(self.rgb_size[0]+200, self.rgb_size[1]))
                self.rgbLabel.resize(QtCore.QSize(self.rgb_size[0], self.rgb_size[1]))
        self.updateSignal.emit()

    def closeEvent(self, ev):
        """ Closes event"""
        self.glWidget.close()
        if self.close_callback is not None:
            self.close_callback()