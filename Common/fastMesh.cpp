/**

Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch/
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

#include "fastMesh.h"

fastMesh::fastMesh(){
    import_array();//IMPORTANT, to create numpy arrays
}

fastMesh::~fastMesh() {
}

PyObject * fastMesh::thresholdFacets(PyObject * mFvcs, PyObject * tupleFaces, double threshold){
    threshold = threshold*threshold;
    PyObject* mFfaces = PyTuple_GetItem(tupleFaces,0);
    int * faces = (int*)((PyArrayObject*)mFfaces)->data;
    int faces_q = ((PyArrayObject*)mFfaces)->dimensions[0];
    int faces_stride = ((PyArrayObject*)mFfaces)->strides[0]/4;
    float * vcs = (float*)((PyArrayObject*)mFvcs)->data;
    int vcsStr = ((PyArrayObject*)mFvcs)->strides[0]/4;
    npy_intp npfaces_q[]= {(npy_intp)faces_q};
    PyObject* PyRemoveF = PyArray_SimpleNew(1, npfaces_q, PyArray_BOOL);
    npy_ubyte * keepF = (npy_ubyte*)((PyArrayObject*)PyRemoveF)->data;

    double a[3];
    double magnitude;
    for(int f=0;f<faces_q;f++){
            int * cFace = faces+f*faces_stride;
            float * vcs0 = vcs + cFace[0]*vcsStr;
            float * vcs1 = vcs + cFace[1]*vcsStr;
            float * vcs2 = vcs + cFace[2]*vcsStr;
            npy_ubyte * cBool = keepF + f;
            *cBool = false;
            // Test the first edge
            a[0] = vcs0[0] - vcs1[0];
            a[1] = vcs0[1] - vcs1[1];
            a[2] = vcs0[2] - vcs1[2];
            magnitude = (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
            if(magnitude>threshold){
                    *cBool = true;
                    continue;
            }
            // Test the second edge
            a[0] = vcs1[0] - vcs2[0];
            a[1] = vcs1[1] - vcs2[1];
            a[2] = vcs1[2] - vcs2[2];
            magnitude = (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
            if(magnitude>threshold){
                    *cBool = true;
                    continue;
            }
            // Test the third edge
            a[0] = vcs0[0] - vcs2[0];
            a[1] = vcs0[1] - vcs2[1];
            a[2] = vcs0[2] - vcs2[2];
            magnitude = (a[0]*a[0]+a[1]*a[1]+a[2]*a[2]);
            if(magnitude>threshold){
                    *cBool = true;
                    continue;
            }
    }
    PyObject * new_tupleFaces = removeFaces(PyRemoveF, tupleFaces);
    Py_DECREF(PyRemoveF);
    return new_tupleFaces;
}


PyObject * fastMesh::removeVertices(PyObject * toRemove, PyObject * tupleVcs, PyObject * tupleFaces){
    PyObject* mFvcs = PyTuple_GetItem(tupleVcs,0);//The first one are the vertices
    PyObject* mFfaces = PyTuple_GetItem(tupleFaces,0);
    int vcs_q = ((PyArrayObject*)mFvcs)->dimensions[0];
    int faces_q = ((PyArrayObject*)mFfaces)->dimensions[0];
    int * faces = (int*)((PyArrayObject*)mFfaces)->data;
    int faces_stride = ((PyArrayObject*)mFfaces)->strides[0]/4;
    npy_ubyte* toRem = (npy_ubyte*)((PyArrayObject*)toRemove)->data;//A boolean list
    int * map = (int*)malloc(sizeof(int)*vcs_q);// The remaping of indices
    int counter = 0;
    // ***********  Creates the mapping of vertex indices
    for(int v=0;v<vcs_q;v++){
         if(toRem[v]==0){
                 map[v]=counter;//Vertices maintained will have an accumulated index
                 counter++;
         }else{
                 map[v]=-1;//Vertices to be removed will be given negative remapping 
         }
    }
    int new_vcs_q = counter;
    // *********** Check which set of facets to maintain
    npy_intp npfaces_q[]= {(npy_intp)faces_q};
    PyObject* PyRemoveF = PyArray_SimpleNew(1, npfaces_q, PyArray_BOOL);
    npy_ubyte * removeF = (npy_ubyte*)((PyArrayObject*)PyRemoveF)->data;
    for(int f=0;f<faces_q;f++){
            int * cFace = faces+f*faces_stride;
            removeF[f] = (map[cFace[0]]==-1)||(map[cFace[1]]==-1)||(map[cFace[2]]==-1);
    }
    // ********** Remove the useless facets and fnormals, etcs *****
    PyObject * new_tupleFaces = removeFaces(PyRemoveF, tupleFaces);
    // Applies the mapping to the new set of faces
    PyObject* new_mFfaces = PyTuple_GetItem(new_tupleFaces,0);
    int * new_faces = (int*)((PyArrayObject*)new_mFfaces)->data;
    int new_faces_stride = ((PyArrayObject*)new_mFfaces)->strides[0]/4;
    int new_faces_q = ((PyArrayObject*)new_mFfaces)->dimensions[0];
    for(int f=0;f<new_faces_q;f++){
            int * new_cFace = new_faces+f*new_faces_stride;
            new_cFace[0] = map[new_cFace[0]];
            new_cFace[1] = map[new_cFace[1]];
            new_cFace[2] = map[new_cFace[2]];
    }
    // *** Creates the tuple to be returned ******
    PyObject* toReturn = PyTuple_New(2);
    PyTuple_SetItem(toReturn, 1, new_tupleFaces);
    // **** Starts creating the new tuple for vertices related data ****
    Py_ssize_t numEl = PyTuple_GET_SIZE(tupleVcs);
    PyObject* new_tupleVcs = PyTuple_New(numEl);
    // For each of the elements in the tuple recreate a numpy array
    for(int i=0;i<numEl;i++){
        PyObject * PyEl = PyTuple_GetItem(tupleVcs,i);
        npy_intp newel_dim[2];
        newel_dim[0]= new_vcs_q;
        newel_dim[1]= ((PyArrayObject*)PyEl)->dimensions[1];
        // Allocates the new numpy array and stores it into the result tuple
        PyTuple_SetItem(new_tupleVcs, i, PyArray_SimpleNew(2, newel_dim, PyArray_FLOAT));
    }
    // Copy the data for the newly created arrays.
    // It's better to iterate first through objects, instead of through vertices firsts
    // This to avoid cache errors.
    for(int i=0;i<numEl;i++){
        PyObject * PyEl = PyTuple_GetItem(tupleVcs,i);//This borrows the reference
        PyObject * new_PyEl = PyTuple_GetItem(new_tupleVcs,i);//This borrows the reference
        float * el = (float*)((PyArrayObject*)PyEl)->data;
        int el_stride = ((PyArrayObject*)PyEl)->strides[0]/4;
        float * newel = (float*)((PyArrayObject*)new_PyEl)->data;
        int newel_stride = ((PyArrayObject*)new_PyEl)->strides[0]/4;
        int columns = ((PyArrayObject*)PyEl)->dimensions[1];
        counter = 0;
        for(int v=0;v<vcs_q;v++){//Go through each of the initial vertices
                if(toRem[v]==0){//If the vertex is not to be removed... then store it
                    float * eldata = el + el_stride*v;
                    float * neweldata = newel + newel_stride*counter;
                    for(int c=0;c<columns;c++){
                            neweldata[c] = eldata[c];
                    }
                    counter++;
                }
        }
    }
    PyTuple_SetItem(toReturn, 0, new_tupleVcs);
    free(map);
    Py_DECREF(PyRemoveF);//To destroy this array that won't be used any more
    return toReturn;
}

PyObject * fastMesh::removeFaces(PyObject * toRemove, PyObject * tupleFaces){
    PyObject* mFfaces = PyTuple_GetItem(tupleFaces,0);
    PyObject* mFfnorms=0;
    bool hasfnorms = PyTuple_GET_SIZE(tupleFaces)>1;
    if(hasfnorms){
         mFfnorms = PyTuple_GetItem(tupleFaces,1);
    }
    int faces_q = ((PyArrayObject*)mFfaces)->dimensions[0];
    npy_ubyte* toRem = (npy_ubyte*)((PyArrayObject*)toRemove)->data;//A boolean list
    int counter = 0;
    for(int f=0;f<faces_q;f++){//Counts how many faces will be kept
         if(toRem[f]==0)counter++;
    }
    // ******  Creates the new facets list and copy the data that needs to be copied, also with the fnorms if available
    npy_intp new_mFfaces_dim[] = {0, 3};
    new_mFfaces_dim[0]=counter;
    PyObject* new_mFfaces = PyArray_SimpleNew(2, new_mFfaces_dim, PyArray_INT);
    PyObject* new_mFfnorms = 0;
    if(hasfnorms){
        new_mFfnorms = PyArray_SimpleNew(2, new_mFfaces_dim, PyArray_FLOAT);
    }
    // Old faces
    int * faces = (int*)((PyArrayObject*)mFfaces)->data;
    int faces_stride = ((PyArrayObject*)mFfaces)->strides[0]/4;
    // New faces
    int * new_faces = (int*)((PyArrayObject*)new_mFfaces)->data;
    int new_faces_stride = ((PyArrayObject*)new_mFfaces)->strides[0]/4;
    // Old face normals
    float * fnorms = 0;
    int fnorms_stride = 0;
    // New face normals
    float * new_fnorms = 0;
    int new_fnorms_stride = 0;
    if(hasfnorms){//If there is indeed face normals, then assign the true data
        fnorms = (float*)((PyArrayObject*)mFfnorms)->data;
        fnorms_stride = ((PyArrayObject*)mFfnorms)->strides[0]/4;
        new_fnorms = (float*)((PyArrayObject*)new_mFfnorms)->data;
        new_fnorms_stride = ((PyArrayObject*)new_mFfnorms)->strides[0]/4;
    }
    //***** Sets the data in the newly created arrays according to whether it should be kept or not
    counter=0;
    for(int f=0;f<faces_q;f++){
            if(toRem[f]==0){
                    int * cFace = faces+f*faces_stride;
                    int * new_cFace = new_faces+counter*new_faces_stride;
                    new_cFace[0] = cFace[0];
                    new_cFace[1] = cFace[1];
                    new_cFace[2] = cFace[2];
                    if(hasfnorms){
                        float * cfnorm = fnorms+f*fnorms_stride;
                        float * new_cfnorm = new_fnorms+counter*new_fnorms_stride;
                        new_cfnorm[0] = cfnorm[0];
                        new_cfnorm[1] = cfnorm[1];
                        new_cfnorm[2] = cfnorm[2];
                    }
                    counter++;
            }
    }
    //*** Creates the tuple where to store the new arrays
    PyObject* toReturn =  0;
    if(hasfnorms){
            toReturn = PyTuple_New(2);//A tuple containing the redefined object and the new set of facets
            PyTuple_SetItem(toReturn, 1, new_mFfnorms);
    }else{
            toReturn = PyTuple_New(1);//A tuple containing the redefined object and the new set of facets
    }
    PyTuple_SetItem(toReturn, 0, new_mFfaces);
    //Note: setting objects to tuples, makes the tuple to take ownership of the reference
    return toReturn;
}



// This function assumes the mesh is triangular, and the memory is arranged
// linearly. For example, in the case of vertices, one vertex is after the other
// and for each of them there are three values. Not necessarily successive
// between vertex and vertex
void fastMesh::computeFaceNormals(PyObject * mFvcs, PyObject * mFfaces, PyObject * mFnormals, float maxDepthChange){
    // Definition of useful variables: pointers to data, quantity of faces, etc
    int faces_q = ((PyArrayObject*)mFfaces)->dimensions[0];
    float * vcs = (float*)((PyArrayObject*)mFvcs)->data;
    float * normals = (float*)((PyArrayObject*)mFnormals)->data;
    int * faces = (int*)((PyArrayObject*)mFfaces)->data;
    int vcsStr = ((PyArrayObject*)mFvcs)->strides[0]/4;
    int faces_stride = ((PyArrayObject*)mFfaces)->strides[0]/4;
    int normals_stride = ((PyArrayObject*)mFnormals)->strides[0]/4;
    // The two vectors defining the facet, their cross product correspond to the normal
    float a[3];
    float b[3];
    for(int f=0;f<faces_q;f++){
            int * cFace = faces+f*faces_stride;
            float * vcs0 = vcs + cFace[0]*vcsStr;
            float * vcs1 = vcs + cFace[1]*vcsStr;
            float * vcs2 = vcs + cFace[2]*vcsStr;
            float * norm = normals + f*normals_stride;
            // Definition of the first vector for the current facet
            a[0]= vcs1[0] - vcs0[0];
            a[1]= vcs1[1] - vcs0[1];
            a[2]= vcs1[2] - vcs0[2];
            // Definition of the second vector for the current facet
            b[0]= vcs2[0] - vcs0[0];
            b[1]= vcs2[1] - vcs0[1];
            b[2]= vcs2[2] - vcs0[2];
            // We identify facets containing NaNs or that have a change of depth above the given threshold
            if((abs(a[2]) > maxDepthChange)||(abs(b[2]) > maxDepthChange)||(a[2]!=a[2])||(b[2]!=b[2])){
                norm[0] = 0.0;
                norm[1] = 0.0;
                norm[2] = -1.0;
            }else{
                // The normal is defined as the cross product between these two vectors
                norm[0] = a[1]*b[2]-a[2]*b[1];
                norm[1] = a[2]*b[0]-a[0]*b[2];
                norm[2] = a[0]*b[1]-a[1]*b[0];
                // Normalization of the norm
                float mag = sqrt(norm[0]*norm[0]+norm[1]*norm[1]+norm[2]*norm[2]);
                if(mag!=0.0){
                        norm[0]=norm[0]/mag;
                        norm[1]=norm[1]/mag;
                        norm[2]=norm[2]/mag;
                }
            }
    }
}

// This function has the same assumptions as the computeFaceNormals function
void fastMesh::computeVertexNormals(PyObject * mFvcs, PyObject * mFfaces, PyObject * mFFacenormals, PyObject * mFVertexnormals){
    int vcs_q = ((PyArrayObject*)mFvcs)->dimensions[0];
    int faces_q = ((PyArrayObject*)mFfaces)->dimensions[0];
    // Pointers to the data
    float * vcs = (float*)((PyArrayObject*)mFvcs)->data;
    int   * faces = (int*)((PyArrayObject*)mFfaces)->data;
    float * fnorms = (float*)((PyArrayObject*)mFFacenormals)->data;
    float * vnorms = (float*)((PyArrayObject*)mFVertexnormals)->data;
    // Strides
    int vcsStr = ((PyArrayObject*)mFvcs)->strides[0]/4;
    int facesStr = ((PyArrayObject*)mFfaces)->strides[0]/4;
    int fnormsStr = ((PyArrayObject*)mFFacenormals)->strides[0]/4;
    int vnormsStr = ((PyArrayObject*)mFVertexnormals)->strides[0]/4;
    // Initializes the memory of the vertex normals to zero
    for(int v =0;v<vcs_q;v++){
         float * vcurr = vnorms + v*vnormsStr;
         vcurr[0]=0.0;
         vcurr[1]=0.0;
         vcurr[2]=0.0;
    }
    // For each of the faces it accumulates its normal into the corresponding vertex position
    for(int f=0;f<faces_q;f++){
            int * cFace = faces+f*facesStr;
            float * cFaceNorm = fnorms+f*fnormsStr;
            if(cFaceNorm[2]!=-1.0){  // Non valid facets have a normal of 0,0,-1 . In this way we ignore them
                for(int v=0;v<3;v++){
                    float * cvnorm = vnorms + cFace[v]*vnormsStr;
                    cvnorm[0] = cvnorm[0] + cFaceNorm[0];
                    cvnorm[1] = cvnorm[1] + cFaceNorm[1];
                    cvnorm[2] = cvnorm[2] + cFaceNorm[2];
                }
            }
    }
    // Normalizes to 1 the magnitude of the vertex normals
    for(int v =0;v<vcs_q;v++){
         float * vcurr = vnorms + v*vnormsStr;
         float mag = sqrt(vcurr[0]*vcurr[0]+vcurr[1]*vcurr[1]+vcurr[2]*vcurr[2]);
         if(mag!=0.0){
             vcurr[0]=vcurr[0]/mag;
             vcurr[1]=vcurr[1]/mag;
             vcurr[2]=vcurr[2]/mag;
         }else{
             // these happens for invalid points (with NaNs) or for isolated ones (not connected to any valid point)
             vcurr[0]=vcurr[1]=0.0; vcurr[2]=1.0;
//       float * vcs_curr = vcs + v*vcsStr;  // Setting the corresponding point as invalid
//       vcs_curr[0]=vcs_curr[1]=vcs_curr[2] = NAN;
         }
    }
}


// This function has the same assumptions as the computeFaceNormals function
void fastMesh::smoothSurface(PyObject * mFvcs, PyObject * mFfaces){
    int vcs_q = ((PyArrayObject*)mFvcs)->dimensions[0];
    int faces_q = ((PyArrayObject*)mFfaces)->dimensions[0];
    // Pointers to the data
    float * vcs = (float*)((PyArrayObject*)mFvcs)->data;
    int   * faces = (int*)((PyArrayObject*)mFfaces)->data;
    // Strides
    int vcsStr = ((PyArrayObject*)mFvcs)->strides[0]/4;
    int facesStr = ((PyArrayObject*)mFfaces)->strides[0]/4;
    
    float * new_vcs = (float*)malloc(sizeof(float)*3*vcs_q);// The new vertices (the accumulated neighbours)
    int * v_count = (int*)malloc(sizeof(int)*vcs_q);// The count of neighbours
    
    // Initiallizes the memory to zero
    for(int v =0;v<vcs_q;v++){
         float * vnew_curr = new_vcs + v*3;
         v_count[v]=0;
         vnew_curr[0]=0.0;
         vnew_curr[1]=0.0;
         vnew_curr[2]=0.0;
    }
    int i=0;
    int j=0;
    float * ve1;
    float * ve2;
    // For each of the facets it accumulates the vertices values according to the defined neighbourhood
    for(int f=0;f<faces_q;f++){
            int * cFace = faces+f*facesStr;
            //float * cFaceNorm = fnorms+f*fnormsStr;
            for(i=0;i<3;i++){
                j = i+1;
                if(j==3)j=0;
                int e1 = cFace[i];
                int e2 = cFace[j];
                // For position e1, it accumulates the vertex in position e2
                ve1 = new_vcs + 3*e1;
                ve2 = vcs + e2*vcsStr;
                v_count[e1] = v_count[e1] + 1;
                for(int d=0;d<3;d++) ve1[d] = ve1[d] + ve2[d];//Copy the dimensions...
            }
    }
    // Now make the copy into the original array and make the weighting according to the quantity of neighbors
    for(int v =0;v<vcs_q;v++){
         if(v_count[v]>0){
             float * vcurr = vcs + v*vcsStr;
             float * vnew_curr = new_vcs + v*3;
             for(int d=0;d<3;d++){
                     vcurr[d] = 0.7*vcurr[d]+(0.3/((float)v_count[v]))*vnew_curr[d];
             }
         }// If it had not neighbors... then its not changed... as simple as that :)
    }
    free(new_vcs);
    free(v_count);
}


