"""
Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch/
Written by Kenneth Funes <kenneth.funes@idiap.ch>

This file contains definitions to retrieve data from different sources of RGBD data while doing the relevant data
calibration.

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
from threading import Thread, Lock
from RGBDCamera import RGBDCamera
from RGBDSources import *
from rgbd.Common.depthMesh import DepthMesh
import rgbd.config as config
from collections import deque
import time
from RGBDProcessing import *

class RGBDStreamer:
    """
    This class is capable of browsing through an RGB-D data stream (videos, devices) while calibrating the data

    """
    pclStreamer = RGBDCalibration()  # Creates a single static copy of the PCL based streamer object

    def __init__(self, frame_callback=None, connection_callback=None, calibrate=True, state_function=None, blocking=True):
        """
        Starts the RGBD streamer object

        The parameters are the following:

        - frame_callback: a callback function this object will call whenever a new calibrated RGBD frame is available
        - connection_callback: this is called whenever there is an update in the connection.
        - calibrate: Whether to calibrate the RGB-D data or not. If not, then the only thing it's retrieved are the
                     RGB and Depth frames.
        - stateFunction: This is a function which this module calls as soon as a new RGB-D frame is read. It is useful
                     to capture external information with timing as close as possible to the timing a frame is read
        """
        # Configuring the behavior
        self.frame_callback = frame_callback
        self.connection_callback = connection_callback
        self.calibrate = calibrate
        self.state_function = state_function
        self.blocking = blocking  # Blocking behavior for the caller application
        # Inner states
        self.streamLoopRunning = True
        self.paused = False
        self.delta  = False

        # The source object for recorded RGB-D data. Devices data sources are handled inside pclStreamer
        self.recordedSource = None
        # The object with the relevant calibration data
        self.KDH = RGBDCamera()
        # By providing the references to the PCLStreamer of the cameras, it should be enough to do the calibration
        self.pclStreamer.setCameras(self.KDH.rgb_camera, self.KDH.depth_camera)
        self.connected = False
        self.N_frames = 100
        # The fifo of frames
        self.FIFO = deque()
        self.frameLock = Lock()
        self.recordingsLock = Lock()
        self.frameIndex = -1

        # Inner thread which is retrieving calibrated frames
        self.streamThread = Thread(target=self.streamLoop)
        self.streamThread.start()

        # Inner thread which is reading recorded RGB-D data (e.g. video files). Emulates a behavior like OpenNI but,
        # due to python restrictions, it would still run in the same CPU as the main thread
        self.RGBDReadThread = Thread(target=self.readRecordedLoop)
        self.RGBDReadThread.start()

    def kill(self):
        self.disconnect()
        self.streamLoopRunning = False# This should eventually kill the listening thread
        if self.streamThread.is_alive():
            self.streamThread.join(0.5)
            if self.streamThread.is_alive():
                self.streamThread.terminate()
        if self.RGBDReadThread.is_alive():
            self.RGBDReadThread.join(0.5)
            if self.RGBDReadThread.is_alive():
                self.RGBDReadThread.terminate()

    def __del__(self):
        self.kill()

    def setStateFunction(self, state_function=None):
        self.state_function = state_function

    def toggleConnect(self, devFile=0, calFile=None):
        if self.connected:
            self.disconnect()
        else:
            self.connect(devFile, calFile)

    def connect(self, source_description=0, calFile=None):
        """
        According to source_description this will establish a connection to the specified source of RGBD data

        The options of sources of data are the diverse. An integer refers to a the connected camera with that same index.
        A string normally refers to the path to recorded data. According to the extension we'll decide on the source
        object which will be created to manage it.

        We can also specify a calibration file. Even though OpenNI is handling some calibration (registration between
        depth and rgb), we still have the option to override that by specifying the calibration file to be used.
        """
        if self.connected:
            self.disconnect()
        self.pause(True)
        self.pclStreamer.setCalibration(self.calibrate)
        if source_description.__class__ is int:
            # If we are connecting to a device
            if config.CONNECTED_DEVICE_SUPPORT:
                self.pclStreamer.connectDevice(source_description)
                if self.calibrate:
                    if calFile is not None:
                        self.KDH.setCalibration(calFile)
                    else:
                        if config.PCL_FOUND:
                            self.KDH.setOpenNIParameters(self.pclStreamer)
                        pass
                self.recordedSource = None
            else:
                return
        else:
            self.recordedSource = RGBD_VideoSource(source_description)
            self.N_frames = self.recordedSource.N_frames
            if self.calibrate:
                if calFile is None:
                    self.KDH.setCalibration(None)
                else:
                    print '<<<<<<<<< LOADING PARAMETERS FILE ', calFile, ">>>>>>>>>>>>>>>>"
                    self.KDH.setCalibration(calFile)
        self.connected = True
        if self.connection_callback is not None:
            self.connection_callback(self.connected)

    def disconnect(self):
        """
        This function kills the data source object
        """
        if self.recordedSource is not None:
            self.recordedSource.close()
            self.recordedSource = None
        else:
            self.pclStreamer.disconnectDevice()
        self.connected = False
        if self.connection_callback is not None:
            self.connection_callback(self.connected)

    def isConnected(self):
        return self.connected

    def jump(self, frameIndex):
        """
        Jump to the indicated frame index. It is not relevant for a connected device
        """
        if self.recordedSource is not None:
            self.recordingsLock.acquire()
            self.delta = True  # To capture the next frame
            self.recordedSource.jump(frameIndex)
            self.recordingsLock.release()

    def pause(self, p=True):
        self.paused = p
        self.pclStreamer.setPause(p)

    def next(self):
        """
        When connected and paused, this function allows to retrieve the next frame without unpausing
        """
        if self.connected and self.paused:
            self.delta=True

    def popFrame(self):
        theFrame = None
        self.frameLock.acquire()# newFrame is used also in another thread
        if len(self.FIFO)>0:
          theFrame = self.FIFO.pop()# Take the newest frame
          self.FIFO.clear()# Not to have an explosion of memory
        self.frame_callback_called = False
        self.frameLock.release()
        return theFrame

    def streamLoop(self):
        """
        Infinite loop which is retrieving frames when needed. Calling the broadcast function when a frame is
        available to the user of this class.
        """
        prev_time = time.time()
        read_count = 0
        anRGBDFrame = RGBDContainer()
        self.frame_callback_called = False
        while self.streamLoopRunning:
            if self.connected and not self.frame_callback_called:
                frameSuccess = self.pclStreamer.getFrameData(anRGBDFrame)
                if frameSuccess:  # The frame was well taken
                    state = None
                    if self.state_function is not None:
                        state = self.state_function() # Gets the state of the caller app ASAP, before further processing or waiting for response
                        state = state, time.time()
                    if self.calibrate:
                        dM = DepthMesh(anRGBDFrame, self.KDH)
                        newFrame = (dM.texture, dM.depth, dM.vcs, dM.txcoord, dM     , state), anRGBDFrame.frameIndex
                    else:
                        rgb, depth, vcs, norms, valid_points = anRGBDFrame.getCloudArrays()
                        newFrame = (rgb       , depth   , None  , None      , anRGBDFrame, state), anRGBDFrame.frameIndex
                    self.frameLock.acquire()
                    self.FIFO.clear()
                    self.FIFO.append(newFrame)
                    self.frameIndex = 0
                    if self.frame_callback is not None:
                        self.frame_callback()
                        self.frame_callback_called = self.blocking
                    self.frameLock.release()
                    anRGBDFrame = RGBDContainer()
                    read_count+=1
                time.sleep(0.001)
                if time.time()-prev_time > 1.0:
                    if not self.paused:
                        print '  >>>>>> Working at ',read_count,' fps  <<<<<'
                    read_count, prev_time = 0, time.time()
            else:
                time.sleep(0.001)

    def readRecordedLoop(self):
        """
        Infinite loop retrieving recorded frames and dumping them into the RGBD Calibration

        The reason this was designed in this way is that the main logic for calibration and data streaming will be
        independent of whether the data stream comes from video files or from a device. Therefore the loop in
        streamLoopRunning does not care about this, besides, the actual calibration can be handled in a different
        thread in the C++ side, which allows to allocate the calibration logic in another CPU than the main thread.
        """
        while self.streamLoopRunning:
            if self.recordedSource is not None and (not self.paused or self.delta) and self.pclStreamer.RGBDFrameProcessed():
                self.delta = False
                self.recordingsLock.acquire()  # To protect the flow of data when there are jumps
                depth, rgb, frameIndex = self.recordedSource.getData()
                if depth is not None and rgb is not None:
                    self.pclStreamer.processPyRGBD(depth, rgb, frameIndex)
                else:
                    self.disconnect()  # We have reached the end of the file, or it tried to do something unusual
                self.recordingsLock.release()
            time.sleep(0.001)






