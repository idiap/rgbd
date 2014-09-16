"""
  Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch/
  Written by Kenneth Funes <kenneth.funes@idiap.ch>

  Example script on how to stream from recorded data (the EYEDIAP dataset) while visualizing
"""
from rgbd.Processing.Processor import Processor
from rgbd.Test import data
import os.path
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

# Selects whether to visualize the EYEDIAP dataset or the sample video provided in our website
useEYEDIAP = False

if useEYEDIAP:
    EYEDIAP_session = '1_A_FT_M'
    source, calibration = data.getEYEDIAPData(session=EYEDIAP_session, hd=False)
else:
    source = os.path.join(data.sample_data, 'rgb_vga.mov'), os.path.join(data.sample_data, 'depth.mov'), 0, 1
    calibration = os.path.join(data.sample_data, 'calibration.npz')

if not (os.path.exists(source[0]) and os.path.exists(source[1])):
    print 'ERROR: One of the following files do not exist:'
    print source[0]
    print source[1]
    exit()

pr = Processor(rendering=True)
pr.run(source, calibration, frame_callback=frameCallback)

