/**
  @file Helper class for Point Cloud Library streamer

Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch
Written by Kenneth Funes <kenneth.funes@idiap.ch>,
Matthieu Duval <matthieu.duval@idiap.ch>

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
*/

#ifndef __RGBD_CALIBRATION_H__
#define __RGBD_CALIBRATION_H__

#include <Python.h>
#include "config.h"

#ifdef PCL_OPENNI_DEVICE_SUPPORT
    #include "pcl/point_types.h"
    #include "pcl/point_cloud.h"
    #include "pcl/io/pcd_io.h"
    #ifdef USE_OPENNI2
        #include <pcl/io/openni2_grabber.h>
        #include "pcl/io/openni2/openni.h"
    #else
        #include <pcl/io/openni_grabber.h>
        #include <pcl/io/grabber.h>
    #endif /* USE_OPENNI2 */
#endif /*PCL_OPENNI_DEVICE_SUPPORT*/

#include "RGBDContainer.h"

#include <boost/thread/mutex.hpp>
#include <sys/time.h>
#include <boost/thread.hpp>
#include "camera.h"
#include <cv.h>

#include "arrayobject.h"

#define _BAD_POINTS_NANS_

class RGBDCalibration {
/**
Class in charge of getting RGB-D frames and calibrating them into PCL meshes with Python/Numpy compatibility.
*/
public:
    RGBDCalibration();
    ~RGBDCalibration();

    /**
    Creates a connection using OpenNI to a connected RGB-D camera. By default it's going to be paused after a connection
    */
    void connectDevice (int device_index);

    /**
    Disconnect from the given device
    */
    void disconnectDevice();

    /**
    When connected to a device, this function retrieves the camera parameters.

    As it is using OpenNI to read data, then the depth map is registered to the RGB map, meaning that a single
    intrinsics matrix is sufficient and the depth is given directly in milimeters.
    */
    void getCameraIntrinsics (PyObject *intrinsics) const;

    /**
    When connected to a device, this function retrieves the dimensions of rgb images and the depth maps.
    */
    bool getDimensions (PyObject * dimensions);

    /**
    Set the streaming state.
    */
    void setPause(bool pause);

    /***
    This functions takes the data from the latest frame and stores it into the RGBDContainer object. The given RGBDContainer will
    then behave like a container of the calibrated data.

    WARNING: this function can only be called from the main python thread.

    @return A flag indicating whether the process went fine (True:OK).
    */
    bool getFrameData(RGBDContainer & container);

    /**
    Enable or disable the data calibration functionality

    If enabled, then it will process the RGB-D frame by removing non-linear distorsions, creating a point cloud and by
    keeping the mapping between RGB and Depth data.
    */
    void setCalibration(bool enable){calibration_enabled=enable;}

    /**
    Returns whether the previously given RGB-D frame has been calibrated and taken by the rest of the pipeline. This
    indicates that it is ready to calibrate more data.
    */
    bool RGBDFrameProcessed();

    /**
    A caller application can give an RGB-D frame to be processed
    */
    void processPyRGBD(PyObject * depth, PyObject * rgb, int frameIndex);

    /**
    Assign the configuration descriptions, i.e. the cameras, extrinsincs and depth computation

    By giving only the camera objects, then the user application has direct access to the calibration parameters and
    geometric capabilities
    */
    void setCameras(camera & rgb_camera_in, depth_camera & depth_camera_in);

private:
    /**
    This function contains an infinite loop which is constantly monitoring for RGBD data to be calibrated
    */
    void calibrationThread();

    /**
    This routine takes the rgb and depth buffers and calibrate the data
    */
    RGBDContainer::PointCloudPtr convertDepthToPointCloud(const unsigned short * depth_buffer) const;

#ifdef PCL_OPENNI_DEVICE_SUPPORT
    /**
    Callback function for the grabber
    */
    void processOpenNIRGBD( const RGBDContainer::openni_image_ptr & image,
                            const RGBDContainer::openni_depth_ptr & depth, float constant_in);
//    #ifdef USE_OPENNI2
//        void processOpenNIRGBD( const pcl::io::openni2::Image::Ptr & image ,
//                                const pcl::io::openni2::DepthImage::Ptr & depth, float rf);
//    #else
//        void processOpenNIRGBD( const boost::shared_ptr<openni_wrapper::Image>     & image ,
//                                const boost::shared_ptr<openni_wrapper::DepthImage> & depth,  float constant_in);
//    #endif /* USE_OPENNI2 */

#endif /* PCL_OPENNI_DEVICE_SUPPORT */

    /**
    Rotates and translates a point cloud. Here it's made to use the same memory for the output (contrary to PCL)
    */
    void transformPointCloud(RGBDContainer::PointCloudPtr cloud, PyArrayObject * R, PyArrayObject * T);

    /**
    Some devices provide data that it's mirrored. This function mirrors it back to normal
    */
    void mirrorDepth(unsigned short * depth_buffer);
    void mirrorRGB(unsigned char* image);

#ifdef PCL_OPENNI_DEVICE_SUPPORT
    /**
       This is a solution to access some protected methods from the OpenNIGrabber
    */
    #ifdef USE_OPENNI2
        class myOpenNIGrabber : public pcl::io::OpenNI2Grabber{
        public:
            myOpenNIGrabber (const std::string& device_id = "", const Mode& depth_mode = OpenNI_Default_Mode, const Mode& image_mode = OpenNI_Default_Mode) {}
    #else
        class myOpenNIGrabber : public pcl::OpenNIGrabber{
        public:
            myOpenNIGrabber (const std::string &device_id = "", const Mode &depth_mode=OpenNI_Default_Mode, const Mode &image_mode=OpenNI_Default_Mode): pcl::OpenNIGrabber::OpenNIGrabber(device_id, depth_mode, image_mode){}
    #endif /* USE_OPENNI2 */
            virtual ~myOpenNIGrabber() throw () {}

            void myGetDimensions (int &rgb_width, int &rgb_height, int &depth_width, int &depth_height) const;

            void myGetRGBCameraIntrinsics (double &rgb_focal_length_x, double &rgb_focal_length_y, double &rgb_principal_point_x, double &rgb_principal_point_y) const;

            void myGetDepthCameraIntrinsics (double &depth_focal_length_x, double &depth_focal_length_y, double &depth_principal_point_x, double &depth_principal_point_y) const;
    };
    // Streaming class with slight modifications to access protected members
    myOpenNIGrabber * interface;

#endif /* PCL_OPENNI_DEVICE_SUPPORT */

    // The calibration thread
    boost::shared_ptr<boost::thread> ptr_thread;

    int count;
    boost::mutex mutex;  // Mutex to protect the calibrated data

    timeval  start,end;
    double timeDiffms(timeval &start, timeval &end);

    // ---------- Funcionality configuration -----------
    bool calibration_enabled;  // whether to generate calibrated date (undistorted, generate point cloud, etc)
    bool mirrorDeviceData;  // whether to mirror or not the input data from the connected device

    // -----  Variables for the calibration thread -----
    boost::mutex RGBD_to_calibrate_mutex;
    bool data_to_calibrate_available;
    int frameIndex_to_calibrate;

    // When using numpy data (from recorded arrays)
    PyObject * py_depth_to_calibrate;
    PyObject * py_rgb_to_calibrate;

#ifdef PCL_OPENNI_DEVICE_SUPPORT
    // When using openni data (from a device)
    RGBDContainer::openni_image_ptr openni_rgb_to_calibrate;
    RGBDContainer::openni_depth_ptr openni_depth_to_calibrate;
#endif /* PCL_OPENNI_DEVICE_SUPPORT */

    // ----- Post calibration variables ---------
    bool calibrated_data_available;
    int frameIndex_calibrated;
    // When using numpy data (from recorded arrays)
    PyObject * py_calibrated_depth;
    PyObject * py_calibrated_rgb;
#ifdef PCL_OPENNI_DEVICE_SUPPORT
    // When using openni data (from a device)
    RGBDContainer::openni_image_ptr openni_calibrated_rgb;
    RGBDContainer::openni_depth_ptr openni_calibrated_depth;
#endif /* PCL_OPENNI_DEVICE_SUPPORT */
    // Additional rgb buffer. Might be generated from openni_wrapper::Image, e.g. when the camera is not RGB, but YUV422
    unsigned char * calibrated_rgb_buffer;
    bool malloc_calibrated_rgb_buffer;  // indicates whether the buffer had to be allocated using malloc
    bool * calibrated_valid_buffer;
    bool calibrated_data_registered;
    bool calibrated_data_normals_computed;
    // The point cloud generated from the calibration
    RGBDContainer::PointCloudPtr calibrated_point_cloud;
    // The camera objects contain most parameters to calibrate the data
    camera * rgb_cam;
    depth_camera * depth_cam;
    //
    bool fromDevice;

    // Control of the flow of recorded data
    bool frameTaken;

    // threads control
    bool continue_calibration_thread;

    bool pause_state;

    bool compute_normals;

};

#endif