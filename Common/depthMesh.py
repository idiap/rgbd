"""
This is a specialization of the mesh class which knows the hold data was generated from RGB-D cameras. It takes
advantage of the image-like order to speed up part of the processes

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
"""
import numpy as np
from rgbd.Streaming.RGBDProcessing import RGBDContainer
from mesh import Mesh

class DepthMesh(Mesh):
    """
    Specialized mesh class for manipulating data coming from an RGB-D camera

    """
    depth_size = None
    @staticmethod
    def genFacets(w=640, h=480):
        """
        Generates a mesh topology adequate for a given depth resolution
        """
        faces = np.empty((2*(w-1)*(h-1), 3), dtype = np.int32)
        # First set of facets
        faces[:faces.shape[0]/2, 0] = np.arange(faces.shape[0]/2)
        faces[:faces.shape[0]/2, 0] += (faces[:faces.shape[0]/2, 0]/(w-1))
        faces[:faces.shape[0]/2, 2] = faces[:faces.shape[0]/2, 0] + 1
        faces[:faces.shape[0]/2, 1] = faces[:faces.shape[0]/2, 0] + w
        # Second set of facets
        faces[faces.shape[0]/2:, 0] = faces[:faces.shape[0]/2, 0] + 1
        faces[faces.shape[0]/2:, 2] = faces[faces.shape[0]/2:, 0] + w
        faces[faces.shape[0]/2:, 1] = faces[faces.shape[0]/2:, 2] - 1
        return faces

    @classmethod
    def redefine_static_variables(dM, w=640, h=480):
        """
        Define a set of variables which remain unchanged for a stream of data, e.g. the depth topology
        """
        dM.depth_size = (w, h)

        # The triangulation of the depth map
        dM.depth_facets = dM.genFacets(w=w, h=h)

        # If the stream of data is "registered", i.e. with a 1 to 1 correspondence between the depth and
        # rgb, then the texture mapping is constant
        colcoord = np.linspace(0, 1, w)
        rowcoord = np.linspace(0, 1, h)
        cols, rows = np.meshgrid(colcoord, rowcoord)
        dM.registeredTxcoord = np.float32(np.hstack( (cols.flatten().reshape(-1, 1), rows.flatten().reshape(-1, 1))))

    def __init__(self, theInput=None, camera= None, maxDepthChange=0.1):
        self.KDH = camera  # It will be convenient to keep a reference to the RGB-D camera object
        self.maxDepthChange = maxDepthChange
        self.type = 'triangulated'
        if theInput is not None:
            if type(theInput) is RGBDContainer:
                self.RGBDContainer = theInput
                self.fromRGBDContainer(camera)
            else:
                if hasattr(theInput,'vcs'):
                    # Then it is a mesh from which a copy has to be made
                    super(DepthMesh, self).__init__(theInput)# This will make the copy of the standard mesh data

    def __del__(self):
        self.texture = None
        self.depth = None
        self.vcs = None
        self.txcoord = None
        self.faces = None
        self.RGBDContainer = None

    def fromRGBDContainer(self, camera):
        """
        This function takes the RGBDContainer object and makes it look like a 3D mesh

        """
        data = self.RGBDContainer.getCloudArrays()
        # First we assign the variables which don't need an adjustment
        self.texture  = data[0]
        self.depth    = data[1]
        self.vcs      = data[2]
        self.vnorms   = data[3]
        self.valid_points = data[4]
        self.txwidth, self.txheight = self.texture.shape[1], self.texture.shape[0]
        self.KDH = camera
        # We verify the static variables are fine, otherwise we redefine them for the incoming data stream.
        # Normally this would be called only for the 1st frame of a stream of data
        if DepthMesh.depth_size is None or DepthMesh.depth_size[0] != self.depth.shape[1] or DepthMesh.depth_size[1] != self.depth.shape[0]:
            DepthMesh.redefine_static_variables(w=self.depth.shape[1], h = self.depth.shape[0])
        if self.RGBDContainer.isRegistered():
            self.txcoord = DepthMesh.registeredTxcoord
        else:
            self.txcoord = self.KDH.rgb_camera.project3Dto2D(self.vcs)
            self.txcoord[:, 0] = self.txcoord[:, 0]/self.KDH.rgb_camera.width
            self.txcoord[:, 1] = self.txcoord[:, 1]/self.KDH.rgb_camera.height
        self.faces = DepthMesh.depth_facets  # A previously constructed mesh for the Kinect data (640x480)
        self.faces_q = self.faces.shape[0]
        self.vcs_q = self.vcs.shape[0]
        if not self.RGBDContainer.normalsComputed():
            self.computeFaceNormals()
            self.computeVertexNormals()

    def crop(self, x_0, x_1, y_0, y_1, with_normals=False):
        """
        This function generates another mesh (not depthMesh) by cropping the organized set of veftices (a depth ROI)

        """
        w, h = self.depth.shape[1], self.depth.shape[0]
        if x_0 < 0 or y_0 < 0 or x_1 >= w or y_1 >= h or x_1 <=x_0 or y_1 <= y_0:
            return None
        mesh = Mesh()
        vcs_aux = self.vcs.reshape(self.depth.shape[0], self.depth.shape[1], 3)
        # Vertices
        mesh.vcs = vcs_aux[y_0:y_1, x_0:x_1, :].reshape(-1, 3)
        mesh.vcs_q = mesh.vcs.shape[0]
        # Normals
        if with_normals:
            vnorms_aux = self.vnorms.reshape(self.depth.shape[0], self.depth.shape[1], 3)
            mesh.vnorms = vnorms_aux[y_0:y_1, x_0:x_1, :].reshape(-1, 3)
        # Facets
        mesh.faces = DepthMesh.genFacets(x_1 - x_0, y_1 - y_0)
        mesh.faces_q = mesh.faces.shape[0]
        # texture mapping
        txcoord_aux = self.txcoord.reshape(self.depth.shape[0], self.depth.shape[1], 2)
        mesh.txcoord = txcoord_aux[y_0:y_1, x_0:x_1, :].reshape(-1, 2)
        mesh.texture = self.texture
        mesh.txwidth, mesh.txheight = self.txwidth, self.txheight
        return mesh
