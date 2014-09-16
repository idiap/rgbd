/**
Copyright (c) 2011-2014 Idiap Research Institute, http://www.idiap.ch/
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
 */
#include "camera.h"

camera::camera(){
  import_array();//IMPORTANT, if numpy arrays need to be created
  intr  = NULL;
  dist = NULL;
  R = NULL;
  T = NULL;
  width = 0;
  height = 0;
  aux_undistorted_buffer = NULL;
  aux_undistorted_buffer_size=0;
}

camera::camera(int width_in, int height_in, PyObject * intr_in, PyObject * dist_in, PyObject * R_in, PyObject * T_in){
  import_array();//IMPORTANT, if numpy arrays need to be created
  intr  = NULL;
  dist = NULL;
  R = NULL;
  T = NULL;
  width = 0;
  height = 0;
  aux_undistorted_buffer = NULL;
  aux_undistorted_buffer_size=0;
  setParameters(width_in, height_in, intr_in, dist_in, R_in, T_in);
}

camera::~camera() {
  releaseParameters();
}

void camera::move(PyObject * R_in, PyObject * T_in){
  if(R_in!=Py_None)Py_INCREF(R_in);
  if(T_in!=Py_None)Py_INCREF(T_in);  
  if(R)Py_DECREF(R);
	if(T)Py_DECREF(T);
  R = NULL;
  T = NULL;
  if(R_in!=Py_None)R = (PyArrayObject*)R_in;
  if(T_in!=Py_None)T = (PyArrayObject*)T_in;
}

void camera::releaseParameters(){
  if(intr)Py_DECREF(intr);
	if(dist)Py_DECREF(dist);
	if(R)Py_DECREF(R);
	if(T)Py_DECREF(T);
  intr  = NULL;
  dist = NULL;
  R = NULL;
  T = NULL;
  if(aux_undistorted_buffer!=NULL)
    free(aux_undistorted_buffer);
}

void camera::setParameters(int width_in, int height_in, PyObject * intr_in, PyObject * dist_in, PyObject * R_in, PyObject * T_in){
  width = width_in;
  height = height_in;
  // We'll keep using the numpy arrays for the parameters...so...
  Py_INCREF(intr_in);
  if(dist_in!=Py_None)Py_INCREF(dist_in);
  if(R_in!=Py_None)Py_INCREF(R_in);
  if(T_in!=Py_None)Py_INCREF(T_in);
  // If there were parameters before, remove them
  releaseParameters();
  // Assign the new ones
  intr = (PyArrayObject*)intr_in;
  if(dist_in!=Py_None)dist  = (PyArrayObject*)dist_in;
  if(R_in!=Py_None)R = (PyArrayObject*)R_in;
  if(T_in!=Py_None)T = (PyArrayObject*)T_in;
  // Recreate the undistorsion mappings
  if(dist)createUndistorsionMaps();
}

void camera::createUndistorsionMaps(){
  mapx = cv::Mat(height,width,CV_32FC1);
  mapy = cv::Mat(height,width,CV_32FC1);
	//Creates opencv headers for the parameters data
	//Note: step for opencv is in terms of bytes, as well as in PyArrayObject the strides
  cv::Mat intr_cv = cv::Mat(3,3, CV_32FC1, (void*)intr->data, intr->strides[0]);
  cv::Mat dist_cv = cv::Mat(1,5, CV_32FC1, (void*)dist->data, dist->strides[0]);
  // Create a dummy identity Matrix
  cv::Mat identity = cv::Mat::eye(3,3, CV_32FC1);
  // Creates mappings for the RGB images
  cv::initUndistortRectifyMap(intr_cv, dist_cv, identity, intr_cv, cv::Size(width,height), CV_32FC1, mapx, mapy);
}

PyObject* camera::project3Dto2D(PyObject* vcs_3D){
  int vcs_q = ((PyArrayObject*)vcs_3D)->dimensions[0];
  npy_intp dims[2];
  dims[0] = vcs_q;
  dims[1] = 2;
  PyObject* vcs_2D = PyArray_SimpleNew(2, dims, PyArray_FLOAT);
  project3Dto2D(vcs_3D,vcs_2D);
  return vcs_2D;
}

// This function could be way faster with a GPU. Each point is independent
void camera::project3Dto2D(PyObject* vcs_3D, PyObject* vcs_2D){
    float * vcs = (float*)((PyArrayObject*)vcs_3D)->data;
    int vcsStr = ((PyArrayObject*)vcs_3D)->strides[0]/4;//4 bytes per value
    int vcs_q = ((PyArrayObject*)vcs_3D)->dimensions[0];
    float * vcs2D = (float*)((PyArrayObject*)vcs_2D)->data;
    int vcs2DStr = ((PyArrayObject*)vcs_2D)->strides[0]/4;//4 bytes per value
    // ----- Takes the intrinsic parameters ------
    float * intr_p = (float*)((PyArrayObject*)intr)->data;
    int intrStr = ((PyArrayObject*)intr)->strides[0]/4;//Four bytes per value
    float cx = *(intr_p + 2);
    float cy = *(intr_p + intrStr + 2);
    float fx = *intr_p;
    float fy = *(intr_p + intrStr + 1);
    // Auxiliary variables
    float v[3];
    float * vp;
    float * v2D;
    // ----- If there are extrinsic parameters, then we'll have to use them
    float r[9];
    float t[3];
    if(R){
      float * Rs = (float*)((PyArrayObject*)R)->data;
      int RStr = ((PyArrayObject*)R)->strides[0]/4;//Four bytes per value
      // Gets r to be the transposed rotation matrix
      r[0]=Rs[0];r[1]=Rs[RStr  ];r[2]=Rs[2*RStr  ];
      r[3]=Rs[1];r[4]=Rs[RStr+1];r[5]=Rs[2*RStr+1];
      r[6]=Rs[2];r[7]=Rs[RStr+2];r[8]=Rs[2*RStr+2];
    }
    if(T){
        float * T_data = (float*)((PyArrayObject*)T)->data;
        t[0]=-T_data[0]; t[1]=-T_data[1]; t[2]=-T_data[2];
        if(R){  // Applies the inverse rotation to T
            v[0] = r[0]*t[0]+r[1]*t[1]+r[2]*t[2];
            v[1] = r[3]*t[0]+r[4]*t[1]+r[5]*t[2];
            v[2] = r[6]*t[0]+r[7]*t[1]+r[8]*t[2];
            t[0] = v[0];t [1]=v[1]; t[2]=v[2];
        }
        // Now "t" should contain:   - R.transpose x T
    }
    // std::cout<<" Making the 3D to 2D projection for which "<<width<<"x"<<height<<std::endl;
    for(int i=0;i<vcs_q;i++){
        vp = vcs + i*vcsStr;
        v[0]=vp[0];v[1]=vp[1];v[2]=vp[2];
        // Applies the extrinsic transformation
        if(R){// Use the (inverse) rotation and translation matrix to transform the points into the camera's own coordinate system
          v[0] = r[0]*vp[0]+r[1]*vp[1]+r[2]*vp[2];
          v[1] = r[3]*vp[0]+r[4]*vp[1]+r[5]*vp[2];
          v[2] = r[6]*vp[0]+r[7]*vp[1]+r[8]*vp[2];
        }
        if(T){
          v[0] = v[0]+t[0];
          v[1] = v[1]+t[1];
          v[2] = v[2]+t[2];
        }
        v2D = vcs2D + i*vcs2DStr;
        v2D[0] = (fx*v[0]/v[2] + cx);
        v2D[1] = (fy*v[1]/v[2] + cy);
        // Clip according to image boundaries
        v2D[0] = std::max(v2D[0], 0.0f);
        v2D[0] = std::min(v2D[0], (float)width-1);
        //
        v2D[1] = std::max(v2D[1], 0.0f);
        v2D[1] = std::min(v2D[1], (float)height-1);
    }
}

PyObject* camera::project2Dto3D(PyObject* vcs_2D){
  int vcs_q = ((PyArrayObject*)vcs_2D)->dimensions[0];
  npy_intp dims[2];
  dims[0] = vcs_q;
  dims[1] = 3;
  PyObject* vcs_3D = PyArray_SimpleNew(2, dims, PyArray_FLOAT);
  project2Dto3D(vcs_2D,vcs_3D);
  return vcs_3D;
}

void camera::project2Dto3D(PyObject* vcs_2D, PyObject* vcs_3D){
    // input data
    float * vcs2D = (float*)((PyArrayObject*)vcs_2D)->data;
    int vcs2DStr = ((PyArrayObject*)vcs_2D)->strides[0]/4;//4 bytes per value
    // output information
    int vcs_q = ((PyArrayObject*)vcs_3D)->dimensions[0];
    float * vcs = (float*)((PyArrayObject*)vcs_3D)->data;
    int vcsStr = ((PyArrayObject*)vcs_3D)->strides[0]/4;//4 bytes per value
    // ----- Takes the intrinsic parameters ------
    float * intr_p = (float*)((PyArrayObject*)intr)->data;
    int intrStr = ((PyArrayObject*)intr)->strides[0]/4;//Four bytes per value
    float cx = *(intr_p + 2);
    float cy = *(intr_p + intrStr + 2);
    float fx = *intr_p;
    float fy = *(intr_p + intrStr + 1);
    // Auxiliary variables
    float v[3];
    float * vp;// pointer used for the 3D data
    float * v2D;// pointer used for the 2D data
    // ----- If there are extrinsic parameters, then we'll have to use them, here only the rotation matrix plays a role
    // the it would be assumed that the returned vectors are rays coming from T (the camera's center)
    float r[9];
    if(R){
      float * Rs = (float*)((PyArrayObject*)R)->data;
      int RStr = ((PyArrayObject*)R)->strides[0]/4;//Four bytes per value
      r[0]=Rs[0];r[1]=Rs[1];r[2]=Rs[2];
      r[3]=Rs[RStr];r[4]=Rs[RStr+1];r[5]=Rs[RStr+2];
      r[6]=Rs[2*RStr];r[7]=Rs[2*RStr+1];r[8]=Rs[2*RStr+2];
    }
    for(int i=0;i<vcs_q;i++){
        v2D = vcs2D + i*vcs2DStr;
        vp = vcs + i*vcsStr;
        v[0] = (v2D[0]-cx)/fx;
        v[1] = (v2D[1]-cy)/fy;
        v[2] = 1.0;
        if(R){
          // Uses the rotation matrix to transform the rays, now referred to this camera coordinate system,
          // into the world coordinate system. The user should know that the new vectors come out from the camera's
          // center (T)
          vp[0] = r[0]*v[0]+r[1]*v[1]+r[2]*v[2];
          vp[1] = r[3]*v[0]+r[4]*v[1]+r[5]*v[2];
          vp[2] = r[6]*v[0]+r[7]*v[1]+r[8]*v[2];
        }else{
          vp[0]=v[0];vp[1]=v[1];vp[2]=v[2];
        }
    }
}


void camera::undistort_src_dst(void * src, void * dst, int src_line_bytes, int dst_line_bytes, int dataType){
  if(dist==NULL){
     // If there is no distortion, then we'll just copy the data
     if(src_line_bytes==dst_line_bytes){
        memcpy(dst, src, src_line_bytes*height);
     }else{
        // This assumes the pixel format is the same! we simply take care of the case in which rows might not be continuous
        unsigned char* src_char = (unsigned char*)src;
        unsigned char* dst_char = (unsigned char*)dst;
        for(int v=0; v<height; v++){
          src_char+=src_line_bytes;
          dst_char+=dst_line_bytes;
          memcpy(dst_char, src_char, src_line_bytes);  // copy row by row
        }
     }
  }else{
    // Represent as opencv data
    cv::Mat src_cv    = cv::Mat(height, width , dataType, src, src_line_bytes);
    cv::Mat result_cv = cv::Mat(height, width , dataType, dst, dst_line_bytes);
    // Undistort the image
    cv::remap(src_cv, result_cv, mapx, mapy, cv::INTER_LINEAR);
  }
}


void camera::undistort_src_src(void * src, int src_line_bytes, int dataType){
   // calls for undistortion, but keeps the data in the same array
   if(dist==NULL){// If there is no distortion, then it doesn't do anything. The src data is unchanged
     //std::cout<<"No undistorsion "<<std::endl;
     return;
   }
   int new_size = height*src_line_bytes;  // the size in bytes of the array
   // We need an auxiliary array to first store the distortion correction result, which might already be allocated
   if(aux_undistorted_buffer_size!=src_line_bytes){
      if(aux_undistorted_buffer!=NULL)
        free(aux_undistorted_buffer);
      aux_undistorted_buffer = malloc(new_size);
   }
   // Makes the distortion correction
   undistort_src_dst(src, aux_undistorted_buffer, src_line_bytes, src_line_bytes, dataType);
   // Copy the corrected data into the input array
   memcpy(src, aux_undistorted_buffer, new_size);
}


PyObject* camera::undistort(PyObject* src){
    if(dist==NULL){
        Py_INCREF(src);  // we increment the array reference count, as it's equivalent to have created another object
        return src;
    }
	PyArrayObject* src_array = (PyArrayObject*)src;
	int nd = (int)src_array->nd;
	npy_intp idims[5];//For compatibility, depending on the system, npy_intp might have different size (64 vs 32 bits)
	for(int i=0;i<nd;i++)idims[i] = (int)src_array->dimensions[i];
	PyObject* result = PyArray_SimpleNew(nd, idims, src_array->descr->type_num);
	PyArrayObject* result_array = (PyArrayObject*)result;
	// Calls for the undistortion function
    undistort_src_dst((void*)src_array->data, (void*)result_array->data, src_array->strides[0], result_array->strides[0], cvType(src_array));
	return result;
}

// assumes it is an image... i.e. it can have 2 or 3 dimensions
int camera::cvType(PyArrayObject * src_array){
  int nd = (int)src_array->nd;
  npy_intp * dims = src_array->dimensions;
  int channels = 1;
  if(nd==3){
      channels = dims[2];
  }
  switch(src_array->descr->type_num){
      case NPY_UINT8: return CV_MAKETYPE(CV_8U,channels);
      case NPY_UINT16: return CV_MAKETYPE(CV_16U,channels);
      case NPY_INT8: return CV_MAKETYPE(CV_8S,channels);
      case NPY_INT16: return CV_MAKETYPE(CV_16S,channels);
      case NPY_INT32: return CV_MAKETYPE(CV_32S,channels);
      case NPY_FLOAT32: return CV_MAKETYPE(CV_32F,channels);
      case NPY_FLOAT64: return CV_MAKETYPE(CV_64F,channels);
      default: std::cerr<<"Numpy type not recognized for opencv"<<std::endl;return -1;
  }
}

depth_camera::depth_camera(): camera(){
  beta = NULL;
  alpha = NULL;
  k_coeff= NULL;
  depthMM = false;
}

depth_camera::depth_camera(int width_in, int height_in, PyObject * intr_in, PyObject * dist_in, PyObject * R_in,
            PyObject * T_in, PyObject * k_coeff_in, PyObject * alpha_in, PyObject * beta_in, bool depthMM_in):
            camera(width_in, height_in ,intr_in, dist_in, R_in, T_in){
  beta = NULL;
  alpha = NULL;
  k_coeff= NULL;
  depthMM = false;
  setDepthParameters(k_coeff_in, alpha_in, beta_in, depthMM_in);
}

depth_camera::~depth_camera(){
  if(k_coeff)Py_DECREF(k_coeff);
  if(alpha)Py_DECREF(alpha);
  if(beta)Py_DECREF(beta);
  k_coeff = NULL;
  alpha =NULL;
  beta = NULL;
}

void depth_camera::setDepthParameters(PyObject * k_coeff_in, PyObject * alpha_in, PyObject * beta_in, bool depthMM_in){
  // First increment the reference counting for the input arrays, indicating this class will be holding a reference.
  if(k_coeff_in!=Py_None)Py_INCREF(k_coeff_in);
  if(alpha_in!=Py_None)Py_INCREF(alpha_in);
  if(beta_in!=Py_None)Py_INCREF(beta_in);
  depthMM = depthMM_in;
  // Clean the previous objects, if there were some
  if(k_coeff)Py_DECREF(k_coeff);
  if(alpha)Py_DECREF(alpha);
  if(beta)Py_DECREF(beta);
  k_coeff = NULL;
  alpha =NULL;
  beta = NULL;
  // Assigns the parameters then
  if(k_coeff_in!=Py_None)k_coeff = (PyArrayObject*)k_coeff_in;
  if(alpha_in!=Py_None)alpha = (PyArrayObject*)alpha_in;
  if(beta_in!=Py_None)beta = (PyArrayObject*)beta_in;
}

void depth_camera::mapToMillimeters(unsigned short* depth_buffer){
    if(depthMM){// If the input data is already in mm, then there is nothing to do
      //std::cout<<"Not mapping to millimeters... already like that"<<std::endl;
      return;
    }
    // Take values for the distortion correction (if there is one)
    float * ks = (float*)((PyArrayObject*)k_coeff)->data;
    float * beta_start=NULL;
    float * betas=NULL;
    float * alphas=NULL;
    float d_temp;
    int betaStr=0;
    if(beta){
      alphas= (float*)alpha->data;
      beta_start = (float*)beta->data;
      betaStr = beta->strides[0]/4;
    }
    unsigned short * d;  //Currently pointed depth
    unsigned short d_min;
    unsigned short d_max;
    bool no_example = true;

    for(int r=0; r<height;r++){
        if(beta)betas = beta_start + r*betaStr;  //point to the corresponding row
        d = depth_buffer + r*width;  // point to the next row
        for(int c=0;c<width; c++, d++, betas++){
            if((*d<2047)&&(*d>0)){
                d_temp = (float)*d;
                if(beta)d_temp = d_temp + (*betas)*exp(alphas[0] - alphas[1]*(d_temp));
                *d = (unsigned short)(1000.0*depthToMetersInv(d_temp, ks[0], ks[1]));
                d_min = std::min(d_min, *d);
                d_max = std::max(d_max, *d);
            }else{
                d_temp = 0;
                *d = 0;
            }
        }
    }
    //std::cout<<"The range of depth "<<d_min<<"  to "<<d_max<<std::endl;
}

void depth_camera::py_mapToMillimeters(PyObject* depth_in){
  if(depthMM) // If it's already in mm, then there is nothing to do
    return;
}

// These parameters should be adjusted to each device with the calibration
inline float depth_camera::depthToMetersTan(unsigned short depth, float k1, float k2 , float k3){
    if(depth<2047){
        return k3*tan(((float)depth)/k2 + k1);
    }else{
        return 0.0;
    }
}

// These parameters should be adjusted to each device with the calibration
// This function uses the equation from the paper by Daniel Herrera Castro
inline float depth_camera::depthToMetersInv(float depth, float k1, float k2){
    return 1.0 / (k2*depth + k1);
}

float depth_camera::getCenterX(){
  if(intr!=NULL){
    return ((float*)intr->data)[2];
  }else{
    return 100.0;
  }
}

float depth_camera::getCenterY(){
  if(intr!=NULL){
    return ((float*)intr->data)[5]; //assumes intr has continuous data, it should be the case
  }else{
    return 100.0;
  }
}

float depth_camera::getFocalLengthX(){
  if(intr!=NULL){
    return ((float*)intr->data)[0]; //assumes intr has continuous data, it should be the case
  }else{
    return 100.0;
  }
}
float depth_camera::getFocalLengthY(){
  if(intr!=NULL){
    return ((float*)intr->data)[4]; //assumes intr has continuous data, it should be the case
  }else{
    return 100.0;
  }
}

