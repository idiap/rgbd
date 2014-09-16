import os

#    The EYEDIAP database path. You can retrieve it from:
#
#        https://www.idiap.ch/dataset/eyediap
#
#    IMPORTANT: Edit this path according to where you have the EYEDIAP database in your system
EYEDIAP_database_path = '/path_to_eyediap/EYEDIAP/'

#    Alternatively you can download a short sample video from our website:
#
#        https://www.idiap.ch/software/rgbd/
#
#    IMPORTANT: Edit this path to the location of the sample video in your system
sample_data = '/path_to_sample/sample/'

def getEYEDIAPData(session='1_A_FT_M', hd=False):
    """
    Returns the corresponding data and calibration files for the given session of the EYEDIAP database
    """
    session_data = os.path.join(EYEDIAP_database_path, 'Data', session)
    # --------------------------------------------------------------------------
    if hd:
        rgb_video_file = os.path.join(session_data, 'rgb_hd.mov')
        rgb_calibration_file = os.path.join(session_data, 'rgb_hd_calibration.txt')
    else:
        rgb_video_file = os.path.join(session_data, 'rgb_vga.mov')
        rgb_calibration_file = os.path.join(session_data, 'rgb_vga_calibration.txt')
    depth_video_file = os.path.join(session_data, 'depth.mov')
    calibration_files = rgb_calibration_file, os.path.join(session_data, 'depth_calibration.txt')
    rgbd_shift = 0
    depth_format = 0  # For the EYEDIAP dataset
    source, calibration = (rgb_video_file, depth_video_file, rgbd_shift, depth_format), calibration_files
    return source, calibration

def getDeviceData():
    """
    Returns the corresponding "data" and calibration for an input device
    """
    source = 0
    calibration = None
    return source, calibration