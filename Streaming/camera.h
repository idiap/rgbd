/**
 * @file camera.h
 * 
 * A C++ class intended to be interfaced with Python to handle camera information
 * which include calibration data and members to handle data projection.
 * 

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

#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <Python.h>
#include "arrayobject.h"
#include <string>
#include <iostream>
#include <cv.h>
#include <algorithm>


class camera{
/**
 * A class to handle camera data. It has functions for camera related geometric
 * calculations:
 *  - Projecting 3D points to 2D
 *  - Taking 2D points and projecting back to 3D (returning a ray)
 *  - Undistorting images

    Something important is that the rotation and translation represent the pose of the camera with respect to the world
    coordinate system. That is, if p_w is a point referred to the world coordinate system and p_c is a point referred to
    the camera's coordinate system, then the transformations are:

        p_w = R p_c + T
        p_c = R^{\top} p_w - R^{\top} T


 */
 friend class RGBDCalibration;
 public:
  /**
  */
  camera();
  
  /**
   * Constructor initializing the camera parameters
   * @param width_in
   * @param height_in
   * @param intr_in
   * @param dist_in
   * @param R_in
   * @param T_in
   */
  camera(int width_in, int height_in, PyObject * intr_in, PyObject * dist_in, PyObject * R_in, PyObject * T_in);

  ~camera();
  
  /**
   * This function takes an input image and applies a remapping, generating a new
   * undistorted image. This depends on the distorsion parameters given before
   * @param src The input image
   * @return The undistorted image
   */
  PyObject* undistort(PyObject* src);

  /**
    Function which takes an image as a data buffer, applies the distortion correction and store the result in the same
    buffer
  */
  void undistort_src_src(void * src, int src_line_bytes, int dataType);

  /**
    Similar to the previous function, but it stores the results in another buffer
  */
  void undistort_src_dst(void * src, void * dst, int src_line_bytes, int dst_line_bytes, int dataType);
  /**
   * This function moves the camera, i.e. redefines the extrinsics
   * @param R_in The rotation matrix, with respect to the world coordinate system
   * @param T_in The translation
   */
  void move(PyObject * R_in, PyObject * T_in);
  
  /**
   * Set the calibration parameters of the camera. All of the values are supposed
   * to be numpy arrays (this is how they will be treated), although some of them
   * can be None (explicitely). For example, when no distorsion is specified, or
   * no extrinsic information (the world coordinate system is the camera's focal point).
   * IMPORTANT: This class will hold a reference to these numpy arrays, so they
   * should NOT be modified outside. For rotation and translation, if one is given,
   * it will be assumed the other is given as well
   * @param width_in
   * @param height_in
   * @param intr_in Intrinsic parameters
   * @param dist_in Distorsion parameters
   * @param R_in Rotation with respect to the world coordinate system
   * @param T_in Translation with respect to the world coordinate system
   */
  void setParameters(int width_in, int height_in, PyObject * intr_in, PyObject * dist_in, PyObject * R_in, PyObject * T_in);
  
  /**
   * This function takes a numpy array of points in 2D and projects them in 3D
   * returning vectors v_w = R^T.[x,y,1]^T. It means that the ray is computed and also
   * transformed to the world coordinate system. Care has to be taken into account
   * when computing a 3D point from these rays, as they are floating (vectors),
   * however they are actually coming from the origin of the camera. Therefore, if the
   * z coordinate (with *reference* to the camera coordinate system is known), then
   * the corresponding 3D point is given as p_w = z*v_w - R^T* T. Useful for depth
   * cameras, which give the value of z at the position of every pixel.
   * the 3D data.
   * @param vcs_2D the array with the 2D points to project back.
   * @param vcs_3D the 3D array where to store the 3D rays
   */
  void project2Dto3D(PyObject* vcs_2D, PyObject* vcs_3D);
  
  /**
   * The same as the previous function, but it allocates the numpy array for the
   * 3D projections
   * @param vcs_2D the array with the 2D points to project back.
   * @return the array with the corresponding 3D vectors
   */
  PyObject* project2Dto3D(PyObject* vcs_2D);
  
  /**
   * This function takes a numpy array of points in 3D and projects them into
   * the camera. It uses a preallocated numpy array for the 2D data. Those points
   * which fall outside of the image will be given coordinates (0.0, 0.0)
   * @param vcs_array the cloud of points to be projected
   * @param vcs_2D the 2D array where to store the projections
   */
  void project3Dto2D(PyObject* vcs_array, PyObject* vcs_2D);
  
  /**
   * The same as the previous function, but it allocates the numpy array for the
   * 2D projections
   * @param vcs_array the cloud of points to be projected
   * @return the array with the corresponding 2D coordinates
   */
  PyObject* project3Dto2D(PyObject* vcs_array);

  int width;
  int height;
protected:
   // References to the current calibration parameters

   PyArrayObject * intr;
   PyArrayObject * dist;
   PyArrayObject * R;
   PyArrayObject * T;
   // The mappings used to remove non-linear distortions
   cv::Mat mapx;
   cv::Mat mapy;

   void * aux_undistorted_buffer;
   unsigned int aux_undistorted_buffer_size;

   /**
    * This function decrements the counters of the python objects used inside
    * this class, and sets the pointers to NULL. Important for consistent memory
    * management.
    */
   void releaseParameters();
   
   //determines the opencv type according to tests in the numpy array
   int cvType(PyArrayObject * src_array);
   /**
    * The way to handle undistorsions is through maps (in the opencv formalism),
    * this function creates them, given that distorsion parameters have been given
    */
   void createUndistorsionMaps();
};

class depth_camera : public camera{
public:
  depth_camera();

  /**
   * Constructor taking the camera parameters, but also taking the depth mapping parameters
   * @param width_in
   * @param height_in
   * @param intr_in
   * @param dist_in
   * @param R_in
   * @param T_in
   */
  depth_camera(int width_in, int height_in, PyObject * intr_in, PyObject * dist_in, PyObject * R_in, PyObject * T_in,
        PyObject * k_coeff_in, PyObject * alpha_in, PyObject * beta_in, bool depthMM_in);

  ~depth_camera();

  /**
  Set the parameters
  */
  void setDepthParameters(PyObject * k_coeff_in, PyObject * alpha_in, PyObject * beta_in, bool depthMM_in);

  /**
  This function takes an input depth map which is then transformed to millimeters according to the parameters
  */
  void mapToMillimeters(unsigned short* depth_buffer);

  /**
  Same as the previous function, but it handles python array objects.
  */
  void py_mapToMillimeters(PyObject* depth_in);

  // ----- Necessary for a caller to know how to transform from a depth map to a 3D mesh ----------
  float getFocalLengthX();
  float getFocalLengthY();
  float getCenterX();
  float getCenterY();
private:
  // The additional objects needed to transform depth data
  bool depthMM;  // If this is true, then input depth data is already in millimeters. The other parameters are ignored
  PyArrayObject * beta;//The matrix which fixes the distortion values in the disparity images (from Daniel Herrera's calibration)
  PyArrayObject * alpha;
  PyArrayObject * k_coeff;

  // ***** Transformations from depth integer ***********
  float depthToMetersTan(unsigned short depth, float k1, float k2 , float k3);

  float depthToMetersInv(float depth, float k1, float k2);
};
#endif
