/**
 * @file Helper class for speed up computation in handling 3D meshes

Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch
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

#ifndef __FAST_MESH_H__
#define __FAST_MESH_H__

#include <Python.h>
#include "arrayobject.h"
#include <string>
#include <iostream>

// Implements the closest point search directly in the GPU
class fastMesh{
public:
    /**
    */
    fastMesh();

    ~fastMesh();

    
    /**
     * Given a set of vertices, their connections and a threshold, if an edge has a distance
     * larger than the threshold, this function will remove it and return a similar list
     * of facets/fnorms, etc according to the quantity of removed vertices
     * @param mFvcs The list of vertices
     * @param tupleFaces A tuple with the list of facets, and possibly related objects, such as fnorms
     * @param threshold The threshold of distance to apply
     */
    PyObject * thresholdFacets(PyObject * mFvcs, PyObject * tupleFaces, double threshold);
 
    /**
     * This function computes the normals associated with each of the facets
     * 
     * @param mFvcs Contains the vertices values
     * @param mFfaces Contains the description of faces.
     * @param mFnormals The destination array, where the face normals are stored (already allocated in memory)
     */
    void computeFaceNormals(PyObject * mFvcs, PyObject * mFfaces, PyObject * mFnormals, float maxDepthChange);
    
    /**
     * This function computes the normal at each vertex by averaging and normalizing
     * the normals of the neighbouring faces
     * @param mFvcs The list of vertices
     * @param mFfaces The list of faces (with indices to the vertices)
     * @param mFFacenormals The list of vertex normals, should be already allocated
     * @param mFVertexnormals The list of vertex normals, should be already allocated
     */
    void computeVertexNormals(PyObject * mFvcs, PyObject * mFfaces, PyObject * mFFacenormals, PyObject * mFVertexnormals);
    
    /**
     * 
     * @param mFfaces The list of facets
     * @param toRemove The vertices to remove as a boolean array
     * @param tuplef A tuple containing the list of vertices, and any other array vertex related
     * such as txcoord, curvature, etc. These are all numpy arrays of floats
     * @return A large tuple beginning with the new set of facets, and then the redefined list of items that were in the input parameter tuple
     */
    PyObject * removeVertices(PyObject * toRemove, PyObject * tupleVcs, PyObject * tupleFaces); 
    
    /**
     * This function takes a boolean list of faces to be removed and generates the
     * substitution for the set of faces and a possible face normals list
     * @param toRemove the List of faces to remove, True: remove, False: keep
     * @param tupleFaces a Tuple with the faces, fnormals, etc.
     * @return A substitute for tupleFaces, but with the corresponding values removed
     */
    PyObject * removeFaces(PyObject * toRemove, PyObject * tupleFaces);
    
    /**
     * This function smooth as surface by averaging the vertices using their neighbours
     */
    void smoothSurface(PyObject * mFvcs, PyObject * mFfaces);
};

#endif
