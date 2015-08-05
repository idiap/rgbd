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

#include "config.h"
#include "RGBDContainer.h"

#ifdef PCL_OPENNI_DEVICE_SUPPORT
using namespace pcl;
#endif /*PCL_OPENNI_DEVICE_SUPPORT*/

RGBDContainer::RGBDContainer ()
{
    import_array();//IMPORTANT, if it needs to create numpy arrays
    dpt = NULL;
    img = NULL;
    vcs = NULL;
    vnorms = NULL;
    rgb_buffer=NULL;
    valid_buffer=NULL;
    valid=NULL;
    malloc_rgb_buffer=false;
    registered=false;
}

RGBDContainer::RGBDContainer (PyObject * vertices, PyObject * normals) 
{
    import_array();  //IMPORTANT, if I need to create numpy arrays
    dpt = NULL;
    img = NULL;
    vcs = NULL;
    vnorms = NULL;
    rgb_buffer=NULL;
    valid_buffer=NULL;
    valid=NULL;
    malloc_rgb_buffer=false;
    registered=false;
    updateCloud(vertices, normals);
}

RGBDContainer::RGBDContainer (PointCloudPtr pointcloud)
{
    import_array();//IMPORTANT, if I need to create numpy arrays
    dpt = NULL;
    img = NULL;
    vcs = NULL;
    vnorms = NULL;
    rgb_buffer=NULL;
    valid_buffer=NULL;
    valid=NULL;
    malloc_rgb_buffer=false;
    registered=false;
    _cloud = pointcloud;
}

RGBDContainer::~RGBDContainer () 
{
    cleanData();
}

void RGBDContainer::cleanData()
{
    if((rgb_buffer!=NULL) && malloc_rgb_buffer)free(rgb_buffer);  // whenever this buffer was allocated separately
    if(valid!=NULL)Py_DECREF(valid);
    if(valid_buffer!=NULL)free(valid_buffer);
    if(dpt!=NULL)Py_DECREF(dpt);
    if(img!=NULL)Py_DECREF(img);
    if(vcs!=NULL)Py_DECREF(vcs);
    if(vnorms!=NULL)Py_DECREF(vnorms);

    valid = NULL;
    valid_buffer=NULL;
    rgb_buffer = NULL;
    malloc_rgb_buffer=false;
    dpt = NULL;
    img = NULL;
    vcs = NULL;
    vnorms = NULL;
    _cloud.reset();
#ifdef PCL_OPENNI_DEVICE_SUPPORT
    image.reset();
    depth.reset();
#endif /* PCL_OPENNI_DEVICE_SUPPORT */
}

bool RGBDContainer::isRegistered()
{
        return registered;
}

bool RGBDContainer::normalsComputed()
{
        return normals_computed;
}

void RGBDContainer::test_function()
{
        std::cout<<"printing"<<std::endl;
}

void RGBDContainer::updateCloud(PyObject * vertices, PyObject * normals)
{
    _cloud = PointCloudPtr(new PointCloud);

    float * vcs = (float*)((PyArrayObject*)vertices)->data;
    float * norms = (float*)((PyArrayObject*)normals)->data;
    int vcs_q = ((PyArrayObject*)vertices)->dimensions[0];
    int vcsStr = ((PyArrayObject*)vertices)->strides[0]/4;
    int normStr = ((PyArrayObject*)normals)->strides[0]/4;

    (*_cloud).resize(vcs_q);
    for (int i=0;i<vcs_q;i++) {
            (*_cloud)[i].x = *(vcs + i*vcsStr);
            (*_cloud)[i].y = *(vcs + i*vcsStr + 1);
            (*_cloud)[i].z = *(vcs + i*vcsStr + 2);

            (*_cloud)[i].normal_x = *(norms + i*normStr);
            (*_cloud)[i].normal_y = *(norms + i*normStr + 1);
            (*_cloud)[i].normal_z = *(norms + i*normStr + 2);
    }
#ifdef PCL_OPENNI_DEVICE_SUPPORT
    image.reset();
    depth.reset();
#endif /* PCL_OPENNI_DEVICE_SUPPORT */
}

void RGBDContainer::updateCloud(RGBDContainer::PointCloudPtr pointcloud)
{
    _cloud = pointcloud;
}

void RGBDContainer::reassignImg(PyObject * in_img)
{
    if(img != NULL)
        Py_DECREF(img);
    img = in_img;
}

PyObject * RGBDContainer::getCloudArrays()
{
    // We initialize a tuple where to store all data
    PyObject* toReturn = PyTuple_New(5);
    // --------- RGB Image -------------
    if(img!=NULL){  //Already has the numpy array
        Py_INCREF(img);
        PyTuple_SetItem(toReturn, 0, img);
#ifdef PCL_OPENNI_DEVICE_SUPPORT
    }else if(image){  // If otherwise contains an OpenNI/PCL image, we create a new numpy a array using the rgb_buffer
        npy_intp img_dim[] = {image->getHeight(),image->getWidth(),3};
        img = PyArray_SimpleNewFromData(3, img_dim, PyArray_UBYTE, (void *)rgb_buffer);
        // Save the reference into the tuple
        Py_INCREF(img);  // we keep a reference within the object
        PyTuple_SetItem(toReturn, 0, img);
#endif /* PCL_OPENNI_DEVICE_SUPPORT */
    }else{  // There is nothing
        Py_INCREF(Py_None);
        PyTuple_SetItem(toReturn, 0, Py_None);
    }
    // ---------- Depth Image --------------
    if(dpt!=NULL){  //Already has the numpy array
        Py_INCREF(dpt);
        PyTuple_SetItem(toReturn, 1, dpt);
#ifdef PCL_OPENNI_DEVICE_SUPPORT
    }else if(depth){
        npy_intp depth_dim[] = {depth->getHeight(),depth->getWidth()};
        #ifdef USE_OPENNI2
            dpt = PyArray_SimpleNewFromData(2, depth_dim, NPY_SHORT, (void *)depth->getData ());
        #else
            dpt = PyArray_SimpleNewFromData(2, depth_dim, NPY_SHORT, (void *)depth->getDepthMetaData ().Data ());
        #endif /* USE_OPENNI2 */
        Py_INCREF(dpt);  // we keep a reference within the object
        PyTuple_SetItem(toReturn, 1, dpt);
#endif /* PCL_OPENNI_DEVICE_SUPPORT */
    }else{
        Py_INCREF(Py_None);
        PyTuple_SetItem(toReturn, 1, Py_None);
    }
    // -------- Point cloud vertices ----------
    if(vcs!=NULL){  //Already has the numpy array
        Py_INCREF(vcs);
        PyTuple_SetItem(toReturn, 2, vcs);
    }else if(_cloud){
        PyArray_Descr* desc = PyArray_DescrFromType(NPY_FLOAT32);
        npy_intp vcs_dim[] = {_cloud->size(),3};
        npy_intp vcs_strides[] = {sizeof(PType),sizeof(float)};
        vcs = PyArray_NewFromDescr( &PyArray_Type, desc, 2, vcs_dim, vcs_strides, (void*)&((*_cloud)[0].x), 0, NULL);
        Py_INCREF(vcs);  // we keep a reference within the object
        PyTuple_SetItem(toReturn, 2, vcs);
    }else{
        Py_INCREF(Py_None);
        PyTuple_SetItem(toReturn, 2, Py_None);
    }
    // -------- Point cloud normals ----------
    if(vnorms!=NULL){  //Already has the numpy array
        Py_INCREF(vnorms);
        PyTuple_SetItem(toReturn, 3, vnorms);
    }if(_cloud){
        PyArray_Descr* desc = PyArray_DescrFromType(NPY_FLOAT32);
        npy_intp vcs_dim[] = {_cloud->size(),3};
        npy_intp vcs_strides[] = {sizeof(PType), sizeof(float)};
        vnorms = PyArray_NewFromDescr( &PyArray_Type, desc, 2, vcs_dim, vcs_strides, (void*)&((*_cloud)[0].normal_x), 0, NULL);
        Py_INCREF(vnorms);  // we keep a reference within the object
        PyTuple_SetItem(toReturn, 3, vnorms);
    }else{
        Py_INCREF(Py_None);
        PyTuple_SetItem(toReturn, 3, Py_None);
    }
    // -------- The arrays of valid points ----------
    if(valid!=NULL){  //Already has the numpy array
        Py_INCREF(valid);
        PyTuple_SetItem(toReturn, 4, valid);
    }if(valid_buffer!=NULL){
        npy_intp valid_dim[] = {_cloud->size()};
        valid = PyArray_SimpleNewFromData(1, valid_dim, NPY_BOOL, (void *)valid_buffer);
        Py_INCREF(valid);  // we keep a reference within the object
        PyTuple_SetItem(toReturn, 4, valid);
    }else{
        Py_INCREF(Py_None);
        PyTuple_SetItem(toReturn, 4, Py_None);
    }
    return toReturn;
}
