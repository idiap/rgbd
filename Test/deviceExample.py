"""
  Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch/
  Written by Kenneth Funes <kenneth.funes@idiap.ch>

  Example script on how to start streaming from a connected device while visualizing the 3D data
"""
from rgbd.Processing.Processor import Processor

count = 0

def frameCallback(frameData, frameIndex):
    """
    A callback function to process RGB-D data
    """
    global count
    if frameData is not None:
        rgb = frameData[0]
        depth = frameData[1]
        mesh = frameData[4]  # An instance of Common/depthMesh.py
        count += 1

source, calibration = 0, None

pr = Processor(rendering=True)
pr.run(source, calibration, frame_callback=frameCallback)

