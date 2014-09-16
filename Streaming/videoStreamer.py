"""
Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch/
Written by Kenneth Funes <kenneth.funes@idiap.ch>
 
This file contains the definition of a class able of loading and recording
video files. It is mostly based on opencv and ffmpeg.

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
import sys
import cv2
import subprocess
from threading import Thread, Lock
import numpy as np
import time
cv = cv2.cv

class VideoStreamer():
    """
    Reader and recorder class for video files.
    
    If write is True, then the video is opened for writing. If it is False, then
    it is being open for reading.
    
    callback is a callable object. Valid only when reading videos. Once an instance
    of this class is created for reading a file, if callback is given, then a
    separate thread will retrieve the frames. Each time a frame becomes available
    the callback function will be invoked.
    
    In the case callback is None, and the class is used for reading videos (write=False).
    Then no Thread will be created and the getFrame function will do the waiting
    for the new frame.
    
    All videos will be assumed to be RGB and the compression will be minimal. The
    Codec is Zlib, is a lossless compressor, and as backend for recording it is
    using ffmpeg (or avconv as young people is calling it nowadays :) ). Opencv is used for
    reading the videos and there is no restriction regarding the codec (as long
    as ffmpeg can handle it, which is the backend of opencv). Of course, zlib is
    included into those codecs.
    
    WARNING: LARGE VIDEOS ARE GENERATED
    
    The variable map provides a mechanism to skip frames when reading a video.
    Here map is assumed to be of size M, where M is the length of the video
    in frames. map[i] indicates whether the frame i is used (True) or ignored
    (False). This is intended to synchronize videos with lost frames, so it is
    expected for map to be almost full of "Trues", i.e. only a few frames
    are missing, otherwise this can become quite inefficient.
    
    """
    def __init__(self, filename, callback = None, write = False, width = 100, height = 60, fps = 29.7, lossy = False, map = None, keep_CV_BGR = False):
        """ Constructor """
        self.filename = filename
        self.keep_CV_BGR = keep_CV_BGR # OpenCV reads as BGR, whether to give back the frames like this, or as RGB (default)
        self.write = write
        self.callback = callback
        self.frameAvailable = False
        self.frame = None
        self.frameIndex = -1
        self.map = map
        if not write:
            self.streamer = cv2.VideoCapture(filename)
            if not self.streamer.isOpened() and self.callback:
                self.callback()# Signalling there is a new frame, but not is given is
                # the way to indicate it has finished
            if self.callback is not None:
                self.lock = Lock()
                self.reading = True
                self.readThread = Thread(target=self.readLoop)
                self.readThread.start()
            self.width    = int(self.streamer.get(cv.CV_CAP_PROP_FRAME_WIDTH ))
            self.height   = int(self.streamer.get(cv.CV_CAP_PROP_FRAME_HEIGHT))
            self.fps      = int(self.streamer.get(cv.CV_CAP_PROP_FPS         ))
            print 'Opening Video ', self.width,'x',self.height, ' at ', self.fps, 'fps'
            self.N_frames = 0
            self.updateMap(self.map)
        else:
            self.width = width
            self.height = height
            self.fps = fps
            self.N_frames = -1
            args = ['ffmpeg', '-f', 'rawvideo', '-r', str(fps), '-y', '-s', str(width)+'x'+str(height), '-pix_fmt', 'rgb24', '-v', '2','-i', 'pipe:0', '-an',  '-vcodec']
            if lossy:
                args = args + ['mpeg4', '-qscale','1']
            else:
                args = args + ['zlib',]
            args.append(filename)

            print '------ STARTING TO WRITE VIDEO -------'
            print args

            self.streamer = subprocess.Popen(args, stdin=subprocess.PIPE, stdout=None, stderr=None)
    
    def updateMap(self, map = None):
        self.N_frames = int(self.streamer.get(cv.CV_CAP_PROP_FRAME_COUNT ))
        self.map = map
        if self.map is not None:
            # Transform from the index understood outside, to the actual index of the video
            self.mapDirect = np.arange(0,len(self.map))
            self.mapDirect = self.mapDirect[self.map]
            # Transforms from the index seen in the video file, to the index understood outside (a -1 indicates the frame is not included)
            self.mapInverse = np.cumsum(self.map)-1
            self.mapInverse[~self.map] = -1
            # The number of frames, as seen from the caller is the following
            self.N_frames = len(self.mapDirect)
    
    def jump(self, frame_idx_in):
        """
        Does a jump to the requested frame. Actually to the one before, such that 
        the next read is the desired frame
        """
        if self.callback:
            self.lock.acquire()
        frame_idx_in = min(frame_idx_in, self.N_frames - 1)
        # This is an inefficient thing to do, but it is a safe jump, as opencv/ffmpeg
        # is sometimes not very accurate when jumping to a frame, so we will go a few
        # frames before what it's actually asked and then start asking for frames
        # until we get to the desired position
        frame_idx = frame_idx_in
        if self.map is not None:
            frame_idx = self.mapDirect[frame_idx_in]

        cv_frametojump = frame_idx-10# max(frame_idx-10, -1)
        count_tries = 0
        while True:
                #print 'Trying to jump to ',cv_frametojump
                self.streamer.set(cv.CV_CAP_PROP_POS_FRAMES, cv_frametojump)
                self.frameIndex  = int(self.streamer.get(cv.CV_CAP_PROP_POS_FRAMES ) - 1)
                #print 'frameIndex is ',self.frameIndex
                if self.frameIndex <= frame_idx_in - 1:  # This means that if the current frame is still passing the desired position, we try again
                        break
                cv_frametojump -= 10
                count_tries += 1
                if count_tries > 5:
                        print 'failed to jump as current frame index is ',self.frameIndex, ' and wanted to go to ',frame_idx
                        raise
        count_tries = 0
        #print 'Before calling readFrame ' , int(self.streamer.get(cv.CV_CAP_PROP_POS_FRAMES ) - 1)
        ok, frame = self.streamer.read()
        #print 'After calling readFrame ' , int(self.streamer.get(cv.CV_CAP_PROP_POS_FRAMES ) - 1)
        # Goes to the frame before frame_idx because if we have or not a callback, the
        # next time it reads a frame will be the desired (frame_idx)
        while self.frameIndex < frame_idx_in - 1:
            self.readFrame()
            count_tries += 1
            if count_tries> 10*self.fps: # 10 seconds max of reading
                print 'failed to jump as current frame index is ',self.frameIndex, ' and wanted to go to ',frame_idx
                raise
        if self.callback:
            self.frameAvailable = False
            self.lock.release()
    
    def readLoop(self):
        if self.callback is not None:
            self.alive = True
            while self.reading:
                if not self.frameAvailable:
                    self.lock.acquire()
                    self.readFrame()
                    self.frameAvailable = True
                    self.callback()
                    self.lock.release()
                else:
                    time.sleep(0.00001)
    
    def stop(self):
        if not self.write:
            if self.callback:
                self.reading = False
                self.readThread.join()
            self.streamer.release()
        else:
            self.streamer.stdin.flush()
            time.sleep(1.0)
            self.streamer.stdin.close()
            
    def readFrame(self):
        new_frame = None
        ok, frame = self.streamer.read()
        if ok:
            #cv2.imwrite('/idiap/temp/kfunes/tmp/video_cv.png', frame)
            if self.keep_CV_BGR:
                    new_frame = frame
            else:
                    new_frame = cv2.cvtColor(frame, cv.CV_BGR2RGB)
        self.frameIndex  = int(self.streamer.get(cv.CV_CAP_PROP_POS_FRAMES ) - 1) # Frame just read
        if self.map is not None:
            while self.mapInverse[self.frameIndex] == -1:# If there are frames to ignore
                print 'missing frame ',self.frameIndex
                ok, frame = self.streamer.read()
                if frame is not None:
                    new_frame = cv2.cvtColor(frame, cv.CV_BGR2RGB)
                self.frameIndex  = int(self.streamer.get(cv.CV_CAP_PROP_POS_FRAMES ) - 1) # Frame just read
            self.frameIndex = self.mapInverse[self.frameIndex]#  Translate the index into what is understood from the caller
        self.frame = new_frame, self.frameIndex
    
    def writeFrame(self, frame, header = None):
        if self.write:
            #self.streamer.stdin.write(frame.tostring())
            if header is not None:
                    drawHeaderString(frame, header)
            self.streamer.stdin.write(np.getbuffer(frame))
            self.streamer.stdin.flush()
    
    def getFrame(self):
        """
            This function will take the available frame and give it if is working with
            the callback functionality, or it will read it and give it if it isn't
        """
        if not self.write:
            if self.callback is None:
                self.readFrame()# When there is no callback, it reads the file here
            else:
                self.lock.acquire()
            frame = self.frame
            self.frame = None
            self.frameAvailable = False
            if self.callback is not None:
                self.lock.release()
            return frame
