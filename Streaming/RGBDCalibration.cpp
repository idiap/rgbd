/**
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

#include "RGBDCalibration.h"

RGBDCalibration::RGBDCalibration() 
{
    import_array();
    interface = NULL;
    // Auxiliary
    count = 0;
    mirrorDeviceData = false;
    calibration_enabled = true;
    data_to_calibrate_available = false;
    calibrated_data_available = false;
    rgb_cam = NULL;
    depth_cam = NULL;
    fromDevice = false;
    calibrated_point_cloud.reset();
    continue_calibration_thread = true;

    compute_normals = false;

    ne.setNormalEstimationMethod (ne.AVERAGE_DEPTH_CHANGE);
    ne.setMaxDepthChangeFactor(0.1f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setDepthDependentSmoothing(false);

    //boost::shared_ptr<boost::thread> aux_ptr_thread(new boost::thread(this->calibrationThread));
    boost::shared_ptr<boost::thread> aux_ptr_thread(new boost::thread(boost::bind(&RGBDCalibration::calibrationThread, this)));
    ptr_thread=aux_ptr_thread;
    frameTaken=true;
    pause_state=true;
}

RGBDCalibration::~RGBDCalibration()
{
    setPause(true);
    continue_calibration_thread=false;
    ptr_thread->join();  // wait until the thread finishes
    if(interface!=NULL){
    delete interface;
    interface = NULL;
    }
    //{
    //  boost::mutex::scoped_lock lock(mutex);
    openni_depth_to_calibrate.reset(); // Cleans everything it can
    openni_rgb_to_calibrate.reset();

    openni_calibrated_depth.reset();
    openni_calibrated_rgb.reset();

    if(py_depth_to_calibrate!=NULL)Py_DECREF(py_depth_to_calibrate);
    if(py_rgb_to_calibrate!=NULL)Py_DECREF(py_rgb_to_calibrate);
    if((calibrated_rgb_buffer!=NULL) && malloc_calibrated_rgb_buffer)free(calibrated_rgb_buffer);
}

void RGBDCalibration::setCameras(camera & rgb_camera_in, depth_camera & depth_camera_in)
{
    rgb_cam = &rgb_camera_in;
    depth_cam = &depth_camera_in;
}

bool RGBDCalibration::RGBDFrameProcessed()
{
    /**
    The logic for recorded data is that we won't ask to process more data until the previous frame has been calibrated.
    Even though this is not very "efficient" as the calibration thread will be useless for a while, it should be still
    fine as the calling application will be busy processing the previously calibrated frame
    */
    return frameTaken;
}

void RGBDCalibration::processPyRGBD(PyObject * depth, PyObject * rgb, int frameIndex)
{
    // Function called from the python side requesting to calibrate the given RGB-D frame
    {
        boost::mutex::scoped_lock lock(RGBD_to_calibrate_mutex);
        //std::cout<<"Received a frame to calibrate"<<std::endl;
        if(!frameTaken){
            std::cout<<"WARNING: The RGBDCalibration has not finished with the previous frame"<<std::endl;
            // We clean the previous data that was waiting to be calibrated
            Py_DECREF(py_depth_to_calibrate);
            Py_DECREF(py_rgb_to_calibrate);
        }
        frameTaken=false;
        // Increment the python reference counters, indicating this class is also keeping a reference of the data
        Py_INCREF(depth);
        Py_INCREF(rgb)  ;
        // Assigns the new data to the queue for calibration
        py_depth_to_calibrate = depth;
        py_rgb_to_calibrate   = rgb  ;
        frameIndex_to_calibrate = frameIndex;
        // Signal there is a new frame waiting
        data_to_calibrate_available = true  ;
    }
}

void RGBDCalibration::processOpenNIRGBD( const boost::shared_ptr<openni_wrapper::Image>      & image,
                                 const boost::shared_ptr<openni_wrapper::DepthImage> & depth,
                                 float constant_in)
{
    if(!pause_state){
        {
         // When streaming from a device we always put the newest obtained frame to be calibrated ASAP
         // thus replacing the previous one
         boost::mutex::scoped_lock lock(RGBD_to_calibrate_mutex);
         // Clean the previous data (if there was any)
         openni_depth_to_calibrate.reset();
         openni_rgb_to_calibrate.reset();
         // Assigns the newly obtained data
         openni_depth_to_calibrate = depth;
         openni_rgb_to_calibrate = image;
         // The frame index doesn't say much for the device streaming, but we'll set the count
         frameIndex_to_calibrate=count;
         data_to_calibrate_available = true;
         count += 1;
        }
    }
}

void RGBDCalibration::calibrationThread()
{
    //  This function has a loop wchich indefinitely calibrating RGB-D data, only if needed.
    //  Note: here we can't create numpy arrays, as it is running in a different thread
    std::cout<<"Running the calibration thread! "<<std::endl;
    // Auxiliary variables to take hold on the data to be calibrated
    PyObject * py_depth;
    PyObject * py_rgb;
    boost::shared_ptr<openni_wrapper::DepthImage> openni_depth;
    boost::shared_ptr<openni_wrapper::Image> openni_rgb;
    // Pointers to the buffers data
    unsigned short * depth_buffer;
    unsigned int depth_bytes_per_line;
    unsigned char * rgb_buffer;
    unsigned int rgb_bytes_per_line;
    bool rgb_malloc=false;
    int frameIndex;
    bool registered=false;

    bool * valid_buffer=NULL;

    pcl::PointCloud<pcl::PointNormal>::Ptr auxPC;
    // Infinite loop retrieving data
    while(continue_calibration_thread){
        if((data_to_calibrate_available) && ((fromDevice&&frameTaken)||(!fromDevice))){
            //std::cout<<"Took a frame to be calibrated"<<std::endl;
            {
                boost::mutex::scoped_lock lock(RGBD_to_calibrate_mutex);
                // We first "take" ownership of the the buffers and other objects to be calibrated
                if(fromDevice){
                openni_depth=openni_depth_to_calibrate;
                openni_rgb  = openni_rgb_to_calibrate;
                openni_depth_to_calibrate.reset();
                openni_rgb_to_calibrate.reset();
                py_depth = NULL;
                py_rgb = NULL;
                py_depth_to_calibrate = NULL;
                py_rgb_to_calibrate = NULL;
                frameTaken=false;
                }else{
                //std::cout<<"Assigning the local python variables"<<std::endl;
                py_depth = py_depth_to_calibrate;
                py_rgb = py_rgb_to_calibrate;
                py_depth_to_calibrate = NULL;
                py_rgb_to_calibrate = NULL;
                }
                // signal openni, or the caller application (from recorded data), that it is "available" to
                // receive more RGB-D data
                frameIndex = frameIndex_to_calibrate;
                data_to_calibrate_available = false;

            }
            // From the given objects we just need the pointers to the raw data
            if(fromDevice){
                depth_buffer = (unsigned short *)openni_depth->getDepthMetaData ().Data ();
                // If the format of the input image is not RGB, we'll need to create a buffer with RGB data
                rgb_malloc = openni_rgb->getEncoding() != openni_wrapper::Image::RGB;
                if (rgb_malloc){
                    rgb_buffer = (unsigned char *) malloc(openni_rgb->getHeight()*openni_rgb->getWidth()*3);
                    openni_rgb->fillRGB (openni_rgb->getWidth(), openni_rgb->getHeight(), rgb_buffer);
                }else{
                    rgb_buffer = (unsigned char * )openni_rgb->getMetaData ().RGB24Data ();
                }
                if(mirrorDeviceData){
                    mirrorDepth(depth_buffer);
                    mirrorRGB  (rgb_buffer  );
                }
                // Assume data is continuous
                rgb_bytes_per_line = openni_rgb->getWidth()*3;
                depth_bytes_per_line = openni_depth->getWidth()*2;
            }else{
                depth_buffer = (unsigned short*)((PyArrayObject*)py_depth)->data;
                rgb_buffer   = (unsigned char*)((PyArrayObject*)py_rgb)->data;
                rgb_bytes_per_line   = ((PyArrayObject*)py_rgb)->strides[0];
                depth_bytes_per_line = ((PyArrayObject*)py_depth)->strides[0];
                rgb_malloc = false;
            }
            if(calibration_enabled){
                 // Remove non-linear radial and tangential distortion (if needed)
                 rgb_cam->undistort_src_src  ( (void*)rgb_buffer     , rgb_bytes_per_line  , CV_MAKETYPE(CV_8U,3));
                 // Depth undistorsion introduce outliers due to interpolation around boundaries (better not...)
                 //depth_cam->undistort_src_src( (void*)depth_buffer   , depth_bytes_per_line, CV_MAKETYPE(CV_16U, 1));
                 // Transform depth to millimeters (if needed)
                 depth_cam->mapToMillimeters(depth_buffer);
                 // Generate the point cloud from the depth map
                 auxPC = convertDepthToPointCloud<pcl::PointNormal>(depth_buffer);
                 // Normals computation
                 if(compute_normals){
                     ne.setInputCloud(auxPC);
                     ne.compute(*auxPC);
                 }
                 // Transform the point cloud to the World coordinate system
                 // By design, the rotation and translation of the depth camera is the inverse. That means, that it
                 // transforms points in its coordinate system to the world coordinate system
                 if( (depth_cam->R!=NULL)|| (depth_cam->T!=NULL)){
                    transformPointCloud(auxPC, depth_cam->R, depth_cam->T);
                    registered=false;
                 }else{
                    registered=true;
                 }
                 valid_buffer=(bool*)malloc(auxPC->size()*sizeof(bool));
                 auxPC->is_dense = false;
                 for (size_t i = 0; i< auxPC->size(); i ++){  // The normals are computed outside this function
                    pcl::PointNormal & p = auxPC->points[i];
                    valid_buffer[i] = !((p.z > 1.1)||(p.z < -0.5)||(p.z!=p.z));
                    // Fixes a problem with inconsistent normals and points which are NaNs, this is to prevent a bug in PCL
                    if(compute_normals) valid_buffer[i] = valid_buffer[i] && (p.normal_z == p.normal_z);// If the normal has a NAN
                    if(!valid_buffer[i]){
                        p.x = p.y = p.z = std::numeric_limits<float>::quiet_NaN ();
                        p.normal_x=0.0; p.normal_y=0.0; p.normal_z = -1.0;
                    }
                 }
            }else{
                 auxPC.reset();
                 valid_buffer=NULL;
            }
            // "saves" the data to be taken by the user application
            {
                boost::mutex::scoped_lock lock(mutex);
                //std::cout<<"Calibrated a frame. Waiting for it to be taken"<<std::endl;
                if(calibrated_data_available){
                    if(fromDevice){
                        // If the calibration is faster than the receiving application we clean the previous calibrated data.
                        // This is fine for devices, but it should not happen from recorded data, in which we are interested in not missing frames.
                        openni_calibrated_depth.reset();
                        openni_calibrated_rgb.reset();
                        if((calibrated_rgb_buffer!=NULL) && malloc_calibrated_rgb_buffer)free(calibrated_rgb_buffer);
                        calibrated_rgb_buffer = NULL;
                        malloc_calibrated_rgb_buffer = false;
                    }else{
                        std::cout<<"WARNING: The RGBDCalibration over producing calibrated frames. MEMORY is leaking!"<<std::endl;
                    }
                }
                // Now we store the calibrated data such that the caller application can retrieve it
                openni_calibrated_depth = openni_depth;
                openni_calibrated_rgb = openni_rgb;
                py_calibrated_depth = py_depth;
                py_calibrated_rgb = py_rgb;
                // when the rgb buffer needs to be regenerated, here it's stored as well
                malloc_calibrated_rgb_buffer = rgb_malloc;
                calibrated_rgb_buffer = rgb_buffer;
                //
                frameIndex_calibrated = frameIndex;
                // the generated point cloud
                calibrated_point_cloud = auxPC;
                // the nans
                calibrated_valid_buffer = valid_buffer;
                // other status about the data
                calibrated_data_registered = registered;
                calibrated_data_normals_computed = compute_normals;
                // "Cleans" the local temporary pointers
                openni_depth.reset();
                openni_rgb.reset();
                auxPC.reset();
                py_depth=NULL;
                py_rgb=NULL;
                rgb_buffer=NULL;
                calibrated_data_available=true;
            }
            boost::this_thread::sleep(boost::posix_time::microseconds(100));  // 100us
         }else{
            boost::this_thread::sleep(boost::posix_time::microseconds(1000));  // 1ms
         }
    }
    if((calibrated_rgb_buffer!=NULL) && malloc_calibrated_rgb_buffer)free(calibrated_rgb_buffer);
    calibrated_rgb_buffer=NULL;
    malloc_calibrated_rgb_buffer=false;
}

void RGBDCalibration::transformPointCloud(pcl::PointCloud<pcl::PointNormal>::Ptr cloud, PyArrayObject * PyR, PyArrayObject * PyT)
{
    if(PyR != NULL){
        float * Rs = (float*)((PyArrayObject*)PyR)->data;
        int RStr = ((PyArrayObject*)PyR)->strides[0]/4;//Four bytes per value
        float R[9];
        R[0]=Rs[0];R[1]=Rs[1];R[2]=Rs[2];
        R[3]=Rs[RStr];R[4]=Rs[RStr+1];R[5]=Rs[RStr+2];
        R[6]=Rs[2*RStr];R[7]=Rs[2*RStr+1];R[8]=Rs[2*RStr+2];
        float v[3];
        for (size_t i = 0; i< cloud->size(); i ++){
            pcl::PointNormal & p = cloud->points[i];
            // We rotate the points
            v[0] = R[0]*p.x+R[1]*p.y +R[2]*p.z;
            v[1] = R[3]*p.x+R[4]*p.y +R[5]*p.z;
            v[2] = R[6]*p.x+R[7]*p.y +R[8]*p.z;
            // Assign the rotated values to the point cloud
            p.x = v[0]; p.y = v[1]; p.z = v[2];
            // Rotate the normal vector
            v[0] = R[0]*p.normal_x+R[1]*p.normal_y +R[2]*p.normal_z;
            v[1] = R[3]*p.normal_x+R[4]*p.normal_y +R[5]*p.normal_z;
            v[2] = R[6]*p.normal_x+R[7]*p.normal_y +R[8]*p.normal_z;
            // Assign the rotated values to the point cloud
            p.normal_x = v[0]; p.normal_y = v[1]; p.normal_z = v[2];
        }
    }
    if(PyT != NULL){
        float * t = (float*)((PyArrayObject*)PyT)->data;
        for (size_t i = 0; i < cloud->size(); i++){
            pcl::PointNormal & p = cloud->points[i];
            p.x += t[0];
            p.y += t[1];
            p.z += t[2];
        }
    }
}

bool RGBDCalibration::getFrameData(RGBDContainer & container)
{
    // Delete all previous data stored in the given container (if any)
    container.cleanData();
    if(!calibrated_data_available)
    return false;
    {
        boost::mutex::scoped_lock lock(mutex);
        // Gives the data to the container
        container._cloud = calibrated_point_cloud;
        container.image = openni_calibrated_rgb;
        container.depth = openni_calibrated_depth;
        container.img = py_calibrated_rgb;
        container.dpt = py_calibrated_depth;
        container.malloc_rgb_buffer = malloc_calibrated_rgb_buffer;
        container.rgb_buffer = calibrated_rgb_buffer;
        container.registered = calibrated_data_registered;
        container.normals_computed = calibrated_data_normals_computed;
        container.frameIndex = frameIndex_calibrated;
        container.valid_buffer = calibrated_valid_buffer;
        frameTaken=true;

     // clean all data indicating it can receive more calibrated data
        calibrated_point_cloud.reset();
        openni_calibrated_rgb.reset();
        openni_calibrated_depth.reset();
        py_calibrated_rgb=NULL;
        py_calibrated_depth=NULL;
        malloc_calibrated_rgb_buffer=false;
        calibrated_rgb_buffer=NULL;
        calibrated_data_available=false;
    }
    return true;
}

template <typename PointT> typename pcl::PointCloud<PointT>::Ptr
RGBDCalibration::convertDepthToPointCloud(const unsigned short * depth_map) const
{
    /**
    This code was adapted from the PCL function pcl::OpenNIGrabber::convertToXYZPointCloud
    */
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud <PointT>);
    cloud->height = depth_cam->height;
    cloud->width = depth_cam->width;
    cloud->is_dense = false;
    // Allocates the memory for the point cloud
    cloud->points.resize (cloud->height * cloud->width);
    // Get intrinsic parameters
    register float constant_x = 1.0f / depth_cam->getFocalLengthX();
    register float constant_y = 1.0f / depth_cam->getFocalLengthY();
    register float centerX = depth_cam->getCenterX();
    register float centerY = depth_cam->getCenterY();
    // Do something better with this id? TODO
    cloud->header.frame_id = std::string("an id");

    float bad_point;
    if(cloud->is_dense){
    bad_point = 0.0f;
    }else{
    bad_point = std::numeric_limits<float>::quiet_NaN ();
    }
    register int depth_idx = 0;
    for (int v = 0; v < depth_cam->height; ++v){
        for (register int u = 0; u <  depth_cam->width; ++u, ++depth_idx){
            PointT& pt = cloud->points[depth_idx];
            // Check for invalid measurements
            if (depth_map[depth_idx] == 0 ){
            pt.x = pt.y = pt.z = bad_point;
            continue;
            }else{
            pt.z = depth_map[depth_idx] * 0.001f;
            if( (pt.z>10.0) && (pt.z<0.05)){
                pt.x = pt.y = pt.z = bad_point;
            }else{
                pt.x = (static_cast<float> (u) - centerX) * pt.z * constant_x;
                pt.y = (static_cast<float> (v) - centerY) * pt.z * constant_y;
            }
            }
        }
    }
    cloud->sensor_origin_.setZero ();
    cloud->sensor_orientation_.w () = 1.0f;
    cloud->sensor_orientation_.x () = 0.0f;
    cloud->sensor_orientation_.y () = 0.0f;
    cloud->sensor_orientation_.z () = 0.0f;
    return (cloud);
}

void RGBDCalibration::mirrorDepth (unsigned short * depth_buffer)
{
    register int width = depth_cam->width;
    register int height = depth_cam->height;
    unsigned short aux_depth;
    for (int v = 0; v < height; ++v){  // row by row we fix the data
        for (register int u = 0; u < width/2; ++u){
            aux_depth = depth_buffer[v*width + u];
            depth_buffer[v*width + u] = depth_buffer[v*width + (width - 1 - u )];
            depth_buffer[v*width + (width - u - 1)] = aux_depth;
        }
    }
}

void RGBDCalibration::mirrorRGB (unsigned char* rgb_data){
    register int width = rgb_cam->width;
    register int height = rgb_cam->height;
    unsigned char aux_rgb;
    for (int v = 0; v < height; ++v){  // row by row we fix the data
        for (register int u = 0; u < width/2; ++u){
            for (register int c = 0; c < 3; c ++){
                aux_rgb = rgb_data[3*(v*width + u)+c];
                rgb_data[3*(v*width + u) +c] = rgb_data[3*(v*width + (width - 1 - u) ) + c];
                rgb_data[3*(v*width + (width - 1 - u) ) + c] = aux_rgb;
            }
        }
    }
}

void RGBDCalibration::connectDevice (int device_index) 
{
    // Openni/PCL configuration
    fromDevice = true;
    pause_state = true;
    if(interface==NULL){
        interface = new myOpenNIGrabber();
        boost::function<pcl::OpenNIGrabber::sig_cb_openni_image_depth_image> f3 = boost::bind (&RGBDCalibration::processOpenNIRGBD, this, _1, _2, _3);
        interface->registerCallback (f3);
        // Configuration of the normals computation
        interface->start();
        interface->stop ();//We stop it here due to a strange behavior from the Carmine camera
    }
}

void RGBDCalibration::disconnectDevice()
{
    setPause(true);
    if(interface!=NULL){
    // Try to free the connected cameras TODO
    }
    fromDevice = false;
}

void RGBDCalibration::setPause(bool pause)
{
    if(interface==NULL)return;
    pause_state = pause;
    if(pause_state){
        interface->stop ();
    }else{
        frameTaken=true;
        interface->start ();
    }
}

void RGBDCalibration::getCameraIntrinsics (PyObject * depthIntrinsics) const
{
    float * data = (float*)((PyArrayObject*)depthIntrinsics)->data;
    double fx, fy, cx, cy;
    interface->myGetDepthCameraIntrinsics (fx, fy, cx, cy);
    data[0] = (float)fx;
    data[2] = (float)cx;
    data[4] = (float)fy;
    data[5] = (float)cy;
    data[6] = data[7] = data[1] = data[3] = 0.0;
    data[8] = 1.0;
}

bool RGBDCalibration::getDimensions(PyObject * dimensions)
{
    if(interface==NULL){
        return false;
    }else{
        int * data = (int*)((PyArrayObject*)dimensions)->data;
        int rgb_width, rgb_height, depth_width, depth_height;
        interface->myGetDimensions(rgb_width, rgb_height, depth_width, depth_height);
        data[0] = rgb_width;
        data[1] = rgb_height;
        data[2] = depth_width;
        data[3] = depth_height;
        return true;
    }
}

double RGBDCalibration::timeDiffms(timeval & start_t, timeval & end_t)
{
    return ((double)1000000*(end_t.tv_sec-start_t.tv_sec) + (double) (end_t.tv_usec - start_t.tv_usec))/1000.0;
}



void RGBDCalibration::myOpenNIGrabber::myGetDepthCameraIntrinsics (double &depth_focal_length_x, double &depth_focal_length_y, double &depth_principal_point_x, double &depth_principal_point_y) const
{
    float constant_x = device_->getDepthFocalLength (depth_width_);
    float constant_y = device_->getDepthFocalLength (depth_width_);
    float centerX = ((float)depth_width_ - 1.f) / 2.f;
    float centerY = ((float)depth_height_ - 1.f) / 2.f;

    if (pcl_isfinite (depth_focal_length_x_))
        constant_x =  static_cast<float> (depth_focal_length_x_);

    if (pcl_isfinite (depth_focal_length_y_))
        constant_y =  static_cast<float> (depth_focal_length_y_);

    if (pcl_isfinite (depth_principal_point_x_))
        centerX =  static_cast<float> (depth_principal_point_x_);

    if (pcl_isfinite (depth_principal_point_y_))
        centerY =  static_cast<float> (depth_principal_point_y_);

    depth_focal_length_x = constant_x;
    depth_focal_length_y = constant_y;
    depth_principal_point_x = centerX;
    depth_principal_point_y = centerY;
}

void RGBDCalibration::myOpenNIGrabber::myGetDimensions (int &rgb_width, int &rgb_height, int &depth_width, int &depth_height) const
{
    rgb_width = (int)image_width_;
    rgb_height = (int)image_height_;
    depth_width = (int)depth_width_;
    depth_height = (int)depth_height_;
}
