"""
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
"""
import numpy as np
from vertices import Vertices
from fastMesh import fastMesh
from scipy import sparse
import os

class Mesh(Vertices):
    fastMesh = fastMesh()#Auxiliary class implemented in C++ to speed things up
    # If theInput is:
    #    - None, then makes nothing
    #    - a string, opens a mesh from a file
    #    - an object and from_file is false, it assumes is another mesh and makes a copy
    # If it is being taken from file, if scales is different than 0, then makes a multiscale pyramid (it's mostly important for bosphorus)
    # it uses one out of "scales+1" points in the scan (horizontally and vertically).
    # - If uniqueScale is True, then a single scale will be obtained, the scale index: "scales-1"
    def __init__(self, theInput=None, from_file=True, scales = 1, uniqueScale = True, loadTexture = True, maxDepthChange=10000000.0):
        self.maxDepthChange = maxDepthChange
        self.type = 'triangulated'
        if theInput is not None:
            if type(theInput) is str:
                self.loadFile(theInput, scales, uniqueScale,loadTexture)
            elif hasattr(theInput,'vcs') and hasattr(theInput,'faces'):  # If the input is a mesh
                # Makes a complete copy of the input mesh, this is intended for truly
                # editing the new version
                self.vcs = theInput.vcs.copy()
                self.vcs_q = theInput.vcs_q
                self.faces = theInput.faces.copy()
                self.faces_q = theInput.faces_q
                if hasattr(theInput,'fnorms'):
                    self.fnorms = theInput.fnorms.copy()
                if hasattr(theInput,'vnorms'):
                    self.vnorms = theInput.vnorms.copy()
                if hasattr(theInput,'texture'):
                    self.texture = theInput.texture.copy()
                    self.txwidth, self.txheight = theInput.txwidth, theInput.txheight
                if hasattr(theInput,'txcoord'):
                    self.txcoord = theInput.txcoord.copy()
                if hasattr(theInput,'color'):
                    self.color = theInput.color.copy()
    
    def loadFile(self, theFile):
        filename, extension = os.path.splitext(theFile)
        if extension == '.npz':
            # A mesh previously saved in a numpy compressed format
            mesh = np.load(theFile)
            self.vcs = mesh['vcs']
            self.vcs_q = self.vcs.shape[0]
            self.faces = mesh['faces']
            self.faces_q = self.faces.shape[0]
            try:
                self.txcoord = mesh['txcoord']
                self.texture = mesh['texture']
                self.txheight, self.txwidth = self.texture.shape[0], self.texture.shape[1]
            except:
                pass
        elif extension == '.obj':
            self.loadObj(theFile)
        else:
            print 'WARNING: Mesh file type not recognized'
            print theFile
        # Vertex normals computation
        self.computeFaceNormals()
        self.computeVertexNormals()


    def removeVertices(self, list):
        """
            Remove a set of vertices

            Given a list of vertices, this function takes care of removing them. This will
            handle the updating of the parameters involved. Additionally it will remove the
            facets which have one vertex from this set. Note: after calling this function
            there might be vertices without any associated facet. Depending on your purpose
            you might want to call "cleanVertices". It should be better to reimplement it in C++.
        """
        if list.dtype == np.bool:# Check that it is given by a boolean definition, otherwise make one
            blist = list
        else:
            blist = np.zeros(self.vcs.shape[0],dtype=np.bool)
            blist[list] = True
        if hasattr(self, 'removedHistory'):
            # This is useful for keeping control of which vertices were removed, it is
            # needed for example when data is retrieved from a depth camera and the original
            # quantity of vertices is always the same
            self.removedHistory[~self.removedHistory]= blist
        fnorms_orig_substitute, vnorms_orig_substitute, vcs_orig_substitute = False, False, False
        faces_related = [self.faces]
        if hasattr(self,'fnorms'):
            faces_related.append(self.fnorms)
            if hasattr(self,'fnorms_orig') and False:
                if not (self.fnorms_orig is self.fnorms):
                    faces_related.append(self.fnorms_orig)
                    fnorms_orig_substitute = True
        vcs_related = [self.vcs]
        if hasattr(self, 'vcs_orig'):
            if not (self.vcs_orig is self.vcs):
                vcs_related.append(self.vcs_orig)
                vcs_orig_substitute = True
        if hasattr(self, 'vnorms'):
            vcs_related.append(self.vnorms)
            if hasattr(self,'vnorms_orig'):
                if not (self.vnorms_orig is self.vnorms):
                    vcs_related.append(self.vnorms_orig)
                    vnorms_orig_substitute = True
        if hasattr(self,'txcoord'):
            vcs_related.append(self.txcoord)
        if hasattr(self,'curv'):
            vcs_related.append(self.curv)
        # Makes the call to the C defined function
        vcs_tuple = tuple(vcs_related)
        faces_tuple = tuple(faces_related)
#    print 'Sending quantity of vcs: ', len(vcs_tuple), len(faces_tuple)
        result = self.fastMesh.removeVertices(blist, vcs_tuple, faces_tuple)
        new_vcs_tuple, new_faces_tuple = result
#    print 'The new results, for vcs: ',len(new_vcs_tuple),' for faces ',len(new_faces_tuple)
        # interprets the result
        self.vcs = new_vcs_tuple[0]
        index = 1
        if hasattr(self,'vcs_orig'):
            if vcs_orig_substitute:
                self.vcs_orig= new_vcs_tuple[index]
                index = index+1
            else:
                self.vcs_orig=self.vcs
        if hasattr(self,'vnorms'):
            self.vnorms = new_vcs_tuple[index]
            index = index+1
        if hasattr(self,'vnorms_orig'):
            if vnorms_orig_substitute:
                self.vnorms_orig= new_vcs_tuple[index]
                index = index+1
            else:
                self.vnorms_orig=self.vnorms
        if hasattr(self,'txcoord'):
            self.txcoord = new_vcs_tuple[index]
            index = index+1
        if hasattr(self,'curv'):
            self.curv= new_vcs_tuple[index]
            index = index+1
        self.faces = new_faces_tuple[0]
        index = 1
        if hasattr(self,'fnorms'):
            self.fnorms= new_faces_tuple[index]
            index = index+1
        if hasattr(self,'fnorms_orig'):
            if fnorms_orig_substitute:
                self.fnorms_orig= new_faces_tuple[index]
                index = index+1
            else:
                self.fnorms_orig=self.fnorms
        self.vcs_q = self.vcs.shape[0]
        self.faces_q = self.faces.shape[0]

    def thresholdFacetSide(self, threshold):
        """
        This function removes any facet with a side larger than this threshold
        """
        faces_related = [self.faces]
        if hasattr(self,'fnorms'):
            faces_related.append(self.fnorms)
        faces_tuple = tuple(faces_related)
        result = self.fastMesh.thresholdFacets(self.vcs, faces_tuple, threshold)
        self.faces = result[0]
        index = 1
        if hasattr(self,'fnorms'):
            self.fnorms= result[index]
            index = index+1
        self.faces_q = self.faces.shape[0]
    

    def cleanVertices(self):
        """
        This function scans and removes vertices not associated to any facet. It would
        have to redefine the values of the facets list as the vertices indexes would be
        different
        """
        used = np.unique(self.faces)
        not_used_bool = np.ones(self.vcs_q, np.bool)
        not_used_bool[used] = False
        if np.sum(not_used_bool)>0:
            self.removeVertices(not_used_bool)

    def saveNPZ(self,filename):
        # This function saves the object into a npz format
        if hasattr(self,'texture'):
            np.savez(filename, vcs=self.vcs, faces = self.faces, texture = self.texture, txcoord = self.txcoord)
        else:
            np.savez(filename, vcs=self.vcs, faces = self.faces)

    def saveObj(self,filename):
        """
        Saves the mesh into a waveform obj file for compatibility with external software (it mostly save vertices and facets)
        """
        file = open(filename,'w')
        # Saves the list of vertices. Optimize this TODO, the for loops are horribly slow
        for i in np.arange(self.vcs.shape[0]):
            file.write('v ')
            np.savetxt(file,np.reshape(self.vcs[i,:],(1,self.vcs.shape[1])),fmt='%f',delimiter=' ')
        for i in np.arange(self.faces.shape[0]):
            file.write('f ')
            np.savetxt(file,np.reshape(self.faces[i,:]+1,(1,self.faces.shape[1])),fmt='%d',delimiter=' ')# In the obj standard, the vertex indices start from 1
        file.close()

    def loadObj(self,filename):
        """
        Loads a waveform obj file
        """
        vcs = []
        faces = []
        file = open(filename)
        for line in file:
            if len(line)>2:
                if line[:2] == 'v ':# Check if it corresponds to a vertex
                    nums = line[2:].split(' ')
                    vcs.append([float(num) for num in nums])
                elif line[:2] == 'f ':# Check if it correpsonds to a facet
                    nums = line[2:].split(' ')
                    nums = [n.split('/')[0] for n in nums]# It might be the extended format of obj
                    nums = [int(num)-1 for num in nums]
                    if len(nums)==3:
                        faces.append(nums)
        file.close()
        self.vcs = np.array(vcs,dtype=np.float32)
        self.faces = np.array(faces,dtype=np.int32)
        self.vcs_q = self.vcs.shape[0]
        self.faces_q = self.faces.shape[0]

    def getTexture(self):
            if hasattr(self, 'texture'):
                return self.texture.tostring()
                #return str(np.getbuffer(self.texture))
            return None


    def computeFaceNormals(self):
        """
        Calls the C++ implementation that computes facets normals
        """
        if not hasattr(self, 'fnorms'):
            self.fnorms = np.empty(self.faces.shape,dtype=np.float32)
        elif self.fnorms.shape != self.faces.shape:
            self.fnorms = np.zeros(self.faces.shape,dtype=np.float32)
        self.fastMesh.computeFaceNormals(self.vcs, self.faces, self.fnorms, self.maxDepthChange)

    def computeVertexNormals(self):
        """
        Computes the vertices normals from the surrounding facets
        """
        if not hasattr(self, 'vnorms'):
            self.vnorms = np.empty(self.vcs.shape,dtype=np.float32)
        elif self.vnorms.shape != self.vcs.shape:
            self.vnorms = np.zeros(self.vcs.shape,dtype=np.float32)
        self.fastMesh.computeVertexNormals(self.vcs, self.faces, self.fnorms, self.vnorms)

    # Using the list of faces it returns a list of edges, which might be repeated
    def getEdges(self):
        elist = np.zeros((2,self.faces.size),dtype=np.int32)
        for i in np.arange(self.faces.shape[1]):
            elist[0,i*self.faces_q:(i+1)*self.faces_q] = self.faces[:,i]
            elist[1,i*self.faces_q:(i+1)*self.faces_q] = self.faces[:,(i+1)%self.faces.shape[1]]
        return elist

    # Return a list of unrepeated edges, undirected
    def uniqueEdges(self):
        elist = self.getEdges()
        # Arrange to the edges ordained for each pair from higher index to lower, that way repeated vertices will fall in the same bin of the edge matrix
        aux = elist[0] < elist[1]
        aux2 = elist[0,aux]
        elist[0,aux] = elist[1,aux]
        elist[1,aux] = aux2
        aux = np.empty((elist.shape[1]),dtype=np.bool)
        aux.fill(True)
        # By doing this, it will build a sparse lower diagonal matrix, where repeated edges fall into the single bin. It's a work around not to use loops
        aux = sparse.coo_matrix((aux, (elist[0],elist[1])), shape=(self.vcs_q, self.vcs_q), dtype=np.bool).tocsr()
        aux = sparse.find(aux)
        elist = np.zeros((2,aux[0].size),dtype=np.int32)
        elist[0] = aux[0]
        elist[1] = aux[1]
        # Goes back to list of edges using the coordinates of the matrix, this time there are no repeated vertices
        return elist

    def getVerticesNeighbors(self, i, edges=None):
        if edges is not None:# It is better to use this when available
            neigh = edges[1, edges[0,:]==i]
        else:
            edges = self.getEdges()
            neigh = edges[1,edges[0,:]==i]

    def smoothSurface(self):
        self.fastMesh.smoothSurface(self.vcs, self.faces)


    """
    This function smooth a bit the surface averaging the vertex position using
    its neighbors...
    """
    def smoothSurfaceOld(self):
        v_new = np.empty(self.vcs.shape)
        edges = self.getEdges()
        # This is extremely slow... almost desperate... for BMVC deadline :(
        for i in np.arange(self.vcs.shape[0]):
            n = edges[1,edges[0,:]==i]
            if len(n)>0:
                v_new[i,:] = 0.6+self.vcs[i,:] + (0.4/len(n))*np.sum(self.vcs[n,:],0)
            else:
                v_new[i,:] = self.vcs[i,:]
        pass
        self.vcs[:]=v_new[:]

    def computeBorderVertices(self):
        elist = self.getEdges()
        aux = elist[0] < elist[1]
        aux2 = elist[0,aux]
        elist[0,aux] = elist[1,aux]
        elist[1,aux] = aux2
        aux = np.ones((elist.shape[1]),dtype=np.int8)# This will allow an edge to be shared up to 255 times
        # Make a sparse matrix, lower diagonal, with the count of vertices
        aux = sparse.coo_matrix((aux, (elist[0],elist[1])), shape=(self.vcs_q, self.vcs_q), dtype=np.int8).tocsr()
        aux = sparse.find(aux)
        aux2 = aux[2]==1 #Edges with only one occurrence are in the border
        aux = np.hstack((aux[0][aux2],aux[1][aux2]))
        self.borderVcs = np.unique(aux)
        
    # For a given mesh, returns the list of vertices which are in the border
    def borderVertices(self):
        if not hasattr(self,'borderVcs'):
            self.computeBorderVertices()
        return self.borderVcs


    def rigidTransform(self, R=None, t=None, s=1.0, redef = False):
        """
            Rigidly transform this mesh, i.e. it applies a Rotation, Translation and a scaling factor

            The applied transformation is:

                 vcs_new = sR vcs_old + t

            This function is intended to make rigid transformations of the current mesh
            with the possibility of succesive transformations. Therefore, there are two
            main functionalities:
            1) The given rigid transformation is defined according to the reference position.
            Therefore succesive calls of this function will transform from this reference position.
            This is the case when the parameter "redef" is False (default).
            2) If redef is True, then it will make the transformation from the reference
            position and redefine the reference state as the outcome of the transformation.
            This would be important when the rigid transformation is given as a succession
            of "deltas"
        """
        super(Mesh,self).rigidTransform(R,t,s,redef)# Optimize TODO
        if R is not None:
            if hasattr(self,'fnorms'):
                if not hasattr(self,'fnorms_orig'):
                    self.fnorms_orig = self.fnorms
                self.fnorms = np.dot(self.fnorms_orig, R.transpose())
                if redef:
                    self.fnorms_orig = self.fnorms
            if hasattr(self,'vnorms'):
                if not hasattr(self,'vnorms_orig'):
                    self.vnorms_orig = self.vnorms
                self.vnorms = np.dot(self.vnorms_orig, R.transpose())
                if redef:
                    self.vnorms_orig = self.vnorms

    # Gives a block of data, consistent with the classes for rendering using OpenGL. Redefine Nicer TODO
    def packArrayData(self):
        #previous = time.clock()
        if hasattr(self, 'color'):
            if self.color.shape[1] == 3:
                aux = np.zeros((9*self.vcs_q),dtype=np.float32)
            else:#With alpha channel
                aux = np.zeros((10*self.vcs_q),dtype=np.float32)
        elif  hasattr(self,'txcoord'):
            aux = np.zeros((8*self.vcs_q),dtype=np.float32)
        else:
            aux = np.zeros((6*self.vcs_q),dtype=np.float32)
        aux[:self.vcs_q*3] = self.vcs.flatten()
        aux[self.vcs_q*3:self.vcs_q*6] = self.vnorms.flatten()
        if hasattr(self,'color'):
            aux[self.vcs_q*6:] = self.color.flatten()
        elif hasattr(self, 'txcoord'):
            aux[self.vcs_q*6:self.vcs_q*8] = self.txcoord.flatten()
        #print 'Packing takes ',1000*(time.clock()-previous),' (ms)'
        return aux
        
