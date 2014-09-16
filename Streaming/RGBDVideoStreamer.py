"""
Copyright (c) 2011-2014 Idiap Research Institute, http://www.idiap.ch/
Written by Laurent Nguyen <laurent.nguyen@idiap.ch>
Modified by Kenneth Funes <kenneth.funes@idiap.ch>

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

import numpy as np
from videoStreamer import VideoStreamer

class RGBDVideoStreamer():
    """
    A class that simultaneously read two video files, one for depth and other for the RGB. It allows for a shift

    """
    def __init__(self, rgb_mov_f, depth_mov_f, shift = 2, depth_format = 0):
        """
        This class the main functionality of interfacing with RGB-D video files. Whether for record or for reading from the corresponding files
        """
        self.framesRGBD = None
        self.depth_mov_f = depth_mov_f
        self.rgb_mov_f = rgb_mov_f
        self.depth_format = depth_format
        self.RGBVideo    = VideoStreamer(rgb_mov_f  )
        self.DepthVideo  = VideoStreamer(depth_mov_f)
        self.frame = 0  # The current frame index, it actually points to the true frame index of the depth video
        self.frame_time = 0.0
        (self.fps, self.N_frames, self.height, self.width) = self.RGBVideo.fps, self.RGBVideo.N_frames, self.RGBVideo.height, self.RGBVideo.width
        self.shape = (self.width, self.height)
        self.depth = 0
        self.rgb = 0
        self.shift = shift
        self.recentIndex = True
        self.jump(0)
    
    def __del__(self):
        self.stop()

    def stop(self):
        if self.RGBVideo is not None:
            self.RGBVideo.stop()
            self.RGBVideo = None
        if self.DepthVideo is not None:
            self.DepthVideo.stop()
            self.DepthVideo = None
            self.depth = None

    def getData(self):
        self.__extractFrame()
        if self.imgRGB is not None and self.imgDepth is not None:
            self.depth = np.uint16(self.imgDepth)
            if self.depth_format == 0:
                # EYEDIAP format
                # For the depth the blue channel contains the lowest 8 bits, the green
                # channel has the 3 highest bits, but coded in the highest 3 bits, here
                # it is shifted 3 bits to complete a total 8 bits shift. Finally a simple
                # add will do the trick here
                self.depth = self.depth[:, :, 2]+np.left_shift(self.depth[:, :, 1], 3)
            elif self.depth_format == 1:
                # LSB in the Blue channel and MSB in the Green Channel
                self.depth = self.depth[:, :, 2]+np.left_shift(self.depth[:, :, 1], 8)
            else:
                # Red: LSB, Green: MSB and blue: MSB
                self.depth = self.depth[:, :, 0]+np.left_shift(self.depth[:, :, 1], 8)
            #self.depth = self.depth[:,:,2]+np.left_shift(self.depth[:,:,1], 3)
            self.rgb   = self.imgRGB.copy()
            return (self.depth, self.rgb)
        else:
            return None, None

    def dataAvailable(self):
        return not ((self.frame < 0) or (self.frame > self.N_frames-1))


    def jump(self, frame):
        """
        Sets the pointer of the reader to the indicated frame number
        """
        if self.shift > 0:# For negative shifts the depth video is ahead
            frame = frame - self.shift
            frame = max(frame,0)
        elif self.shift < 0:
            frame = min(frame, self.N_frames -1 + self.shift)
        else:
            frame = min(max(frame, 0), self.N_frames -1)
        self.DepthVideo.jump(frame)
        self.RGBVideo.jump(frame)
        self.recentIndex = True
        self.frame = frame - 1
  
    def __extractFrame(self):
        if self.recentIndex:  # A recent index flag means apply the shift to the lack of phase between the videos
            if self.shift<0:
                for i in range(1-self.shift):
                    self.imgDepth, frameIdx = self.DepthVideo.getFrame()
                self.imgRGB, self.frame = self.RGBVideo.getFrame()
            else:
                self.imgDepth, frameIdx = self.DepthVideo.getFrame()
                for i in range(1+self.shift):
                    self.imgRGB, self.frame = self.RGBVideo.getFrame()
            self.recentIndex = False
        else:
            if self.frame - self.shift >= 0 and self.frame-self.shift < self.N_frames:
                self.imgDepth, frameIdx = self.DepthVideo.getFrame()
                self.imgRGB, self.frame = self.RGBVideo.getFrame()
            else:
                self.imgRGB, self.imgDepth = None, None

