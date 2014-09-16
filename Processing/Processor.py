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
from rgbd.Streaming.RGBDStreamer import RGBDStreamer
from rgbd.Rendering.RGBDViewer import RGBDViewer
from PySide import QtCore, QtGui
import sys

class Processor(QtCore.QObject):
    """
    Class to handle a stream of RGB-D data and to display it

    """
    newFrameSignal = QtCore.Signal()
    def __init__(self, rendering=True):
        super(Processor, self).__init__()
        self.connected=False
        self.viewer=None
        self.streamer=None
        self.frame_callback=None
        self.rendering = rendering
        self.count = 0
        self.app = None

    def __del__(self):
        self.stop()

    def createGUI(self):
        # Creates the widget where to visualize the RGB-D data
        self.viewer = RGBDViewer(self.app, render3D=self.rendering, close_callback=self.stop)
        self.viewer.show()
        self.connect(self.viewer.pauseButton, QtCore.SIGNAL("clicked()"), self.pause)
        self.newFrameSignal.connect(self.processFrame)

    def pause(self):
        """
        Toggle the pause status
        """
        self.streamer.pause(not self.streamer.paused)

    def run(self, source=0, calibrationFile=None, frame_callback =None):
        # Sets the function to be called each time a new frame data is available
        self.frame_callback=frame_callback
        # Creates the RGB-D data streaming class
        self.streamer=RGBDStreamer(frame_callback=self.newFrameSignal.emit, connection_callback=self.connectionUpdate, calibrate = True)  # frame_callback=self.newFrame
        self.streamer.connect(source, calibrationFile)
        # Now create the Qt Application (basically for the Qt events loop)
        self.app = QtGui.QApplication(sys.argv)
        # Creates the necessary GUI
        self.createGUI()
        # Puts the streamer to run freely
        self.streamer.pause(False)
        # Runs the Qt Loop
        sys.exit(self.app.exec_())

    def stop(self):
        """
        Stops the process of data generation
        """
        if self.streamer is not None:
            self.streamer.pause(True)
            self.streamer.disconnect()
            self.streamer.kill()
            self.streamer=None

    def connectionUpdate(self, state=False):
        self.connected = state

    def processFrame(self):
        """
        This function is called within the Qt events loop, as response tot he newFrameSignal activation
        """
        if self.streamer is None:
            return
        data = self.streamer.popFrame()
        if data is not None:
            self.frame, self.frameIndex = data
            self.newFrameAvailable = False
            if self.frame is not None:
                if self.frame_callback is not None:
                    self.frame_callback(self.frame, self.frameIndex)
                self.viewer.setNewData(self.frame, self.frameIndex)
            else:
                self.frameMesh, self.state = None, None




