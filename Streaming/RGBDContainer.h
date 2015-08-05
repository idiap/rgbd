/**
 @file Helper class to contain a single calibrated frame given by the RGBDCalibration class

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

#ifndef __RGBD_CONTAINER_H__
#define __RGBD_CONTAINER_H__

#include <Python.h>
#include "config.h"

#ifdef PCL_OPENNI_DEVICE_SUPPORT
    #include "pcl/point_types.h"
    #include "pcl/point_cloud.h"
    #include "pcl/io/pcd_io.h"
    #ifdef USE_OPENNI2
        #include "pcl/io/openni2/openni.h"
        #include <pcl/io/openni2_grabber.h>
    #else
        #include <pcl/io/openni_camera/openni_image.h>
        #include <pcl/io/openni_camera/openni_depth_image.h>
    #endif /* USE_OPENNI2 */
#else /*PCL_OPENNI_DEVICE_SUPPORT*/
    // If there is no PCL/OpenNI installation, for consistency we define a set of types which are useful to maintain PCL
    // compatibility making the code "look" like pcl
    #include <vector>
    #include <boost/shared_ptr.hpp>

#endif /*PCL_OPENNI_DEVICE_SUPPORT*/

#include "arrayobject.h"
#include <boost/math/special_functions/fpclassify.hpp>

class RGBDContainer{
    /**
    This class is somewhat of a container of data coming from the PCLStreamer

    Data given from the RGBDCalibration class is intended to be stored in this class. Then this object becomes the OWNER of all
    data. This includes numpy arrays, OpenNI/PCL images, point cloud, etc. The user can then retrieve all data on the
    python side using getCloudArrays() but the internal buffers would still belong to this class. The user still can
    modify the data as long as there are no changes in the buffers structures, but only in their content.
    */

    friend class RGBDCalibration;  // For the RGBDCalibration to be able to replace the data inside this class
public:
#ifdef PCL_OPENNI_DEVICE_SUPPORT
    typedef pcl::PointNormal PType;
    typedef pcl::PointCloud<PType> PointCloud;
    typedef pcl::PointCloud<PType>::Ptr PointCloudPtr;
    #ifdef USE_OPENNI2
        typedef pcl::io::openni2::Image::Ptr openni_image_ptr;
        typedef pcl::io::openni2::DepthImage::Ptr openni_depth_ptr;
    #else
        typedef boost::shared_ptr<openni_wrapper::Image> openni_image_ptr;
        typedef boost::shared_ptr<openni_wrapper::DepthImage> openni_depth_ptr;
    #endif /* USE_OPENNI2 */
#else /*PCL_OPENNI_DEVICE_SUPPORT*/
    struct PointNormal{
        float x;
        float y;
        float z;
        float normal_x;
        float normal_y;
        float normal_z;
    };
    typedef PointNormal PType;
    typedef std::vector<PType> PointCloud;
    typedef boost::shared_ptr<PointCloud> PointCloudPtr;
#endif /*PCL_OPENNI_DEVICE_SUPPORT*/

    RGBDContainer ();
    RGBDContainer (PyObject * vertices, PyObject * normals);// Starts from numpy data
    RGBDContainer (PointCloudPtr pointcloud);// Starts from PCL data
    ~RGBDContainer();

    void updateCloud(PyObject * vertices, PyObject * normals);
    void updateCloud(PointCloudPtr pointcloud);//{ _cloud = pointcloud;}


    /**
     Returns numpy arrays wrapping the data contained in the point clouds
     */
    PyObject * getCloudArrays();

    bool isRegistered();

    bool normalsComputed();

    void test_function();

    bool lost;  // flag of success or not for after running ICP

    int frameIndex;  // The frame index number

private:
    void reassignImg(PyObject * img);

    void cleanData();

    // ---- The data this class is holding -----
    PointCloudPtr _cloud;

    PyObject * img;  //The numpy reference to the image
    PyObject * dpt;  //The numpy reference to the depth
    PyObject * vcs;  //The numpy reference to the point cloud vertices
    PyObject * vnorms; //The numpy reference to the point cloud normals
    PyObject * valid; //The numpy reference to the list of valid points within the point cloud

    // The flags indicating which of the points are valid or not (e.g. are not NANs)
    bool * valid_buffer;
    // The rgb buffer, which might have been allocated separately from img and image (due to inconvenient input formats, e.g. YUV224)
    // That only happens when the origin is actually image. So, when assuming to be referring to "image" we can use rgb_buffer instead
    unsigned char * rgb_buffer;
    bool malloc_rgb_buffer;

    // if the RGB-D is actually registered, meaning there is one-to-one correspondence between depth and rgb pixels
    bool registered;
    bool normals_computed;

#ifdef PCL_OPENNI_DEVICE_SUPPORT
    openni_image_ptr image;
    openni_depth_ptr depth;
#endif /* PCL_OPENNI_DEVICE_SUPPORT */
};


#endif  /* __RGBD_CONTAINER_H__ */