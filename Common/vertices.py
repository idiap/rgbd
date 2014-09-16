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

class Vertices(object):
    """
    A container class for a set of 3D points (a point cloud)

    """
    def __init__(self,vcs=None, color= None):
        self.preferColor = True
        if vcs is not None:
                self.vcs = vcs.copy()
                self.vcs_q = self.vcs.shape[0]
                if color is not None:
                    self.color = color

    def getDims(self):
        return np.concatenate((self.center,self.dims))

    def rigidTransform(self, R=None, t=None, s=1.0, redef = False):
        """
            Rigidly transform this set of vertices, i.e. it applies a Rotation, Translation and a scaling factor

            The applied transformation is:

                 vcs_new = sR vcs_old + t

            Computes a rigid transformation of all the vertices, a rotation and a translation (column)
            The new vertices will be:   v_new = sR*v_old + t. R is assumed to be 3x3 and t 3x1, scale
            is a scalar.
            If 'redef' is true, then the object position will be completely redefined. However
            if it is False the transformation will be applied to the original position
            of the object, defined in its creation or the last time this function was
            called with the 'redef' variable as True
        """
        # It is faster to make some tests rather than apply the whole transformation
        # even if redundant. Optimize TODO
        if not hasattr(self,'vcs_orig'):
            self.vcs_orig = self.vcs# Even if vcs is modified, vcs_orig is maintained
        if R is None:
            if s != 1.0:
                self.vcs = s*self.vcs_orig
            else:
                self.vcs = self.vcs_orig
        else:
            self.vcs = np.dot(self.vcs_orig, s*R.transpose())
        if t is not None:
            self.vcs = self.vcs + t.transpose()
        if redef:
            self.vcs_orig = self.vcs

    def updateDims(self):
        mins = self.vcs.min(axis = 0)
        maxs = self.vcs.max(axis = 0)
        self.center =self.vcs.mean(0)
        self.dims = maxs - mins
        self.len  = sum(self.dims)
        return np.concatenate((self.center,self.dims))
    
    def normalize(self):
        self.updateDims()
        self.vcs = self.vcs - self.center
        self.vcs = self.vcs/np.max(self.dims)
        self.norm_scale = np.max(self.dims)# Keeps track of the normalization constant
        self.updateDims()
        return self.norm_scale

    def packArrayData(self):
        if hasattr(self,'color'):
             aux = np.vstack((self.vcs,self.color))
        else:
             aux = self.vcs
        return aux.flatten()

