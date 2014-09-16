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
import os
from RGBDVideoStreamer import RGBDVideoStreamer

class RGBDBIWI(object):
    def __init__(self):
        super(RGBDBIWI, self).__init__()

class RGBD_VideoSource(object):
    def __init__(self, source):
        super(RGBD_VideoSource, self).__init__()
        self.rgb_video_file = source[0]
        self.depth_video_file = source[1]
        self.shift = 0
        self.depth_format = 0
        if len(source)>2:
            self.shift = source[2]
        if len(source)>3:
            self.depth_format = source[3]
        self.reader = RGBDVideoStreamer(self.rgb_video_file, self.depth_video_file, self.shift, self.depth_format)
        self.N_frames = self.reader.N_frames
        self.frameIndex = 0

    def jump(self, index):
        if index > -1 and index < self.N_frames:
            self.reader.jump(index)

    def getData(self):
        depth, rgb = self.reader.getData()
        self.frameIndex = self.reader.frame
        self.depth= depth
        self.video = rgb
        return depth, rgb, self.frameIndex

    def close(self):
        self.reader.stop()

    def __del__(self):
        pass

