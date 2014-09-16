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
from RGBDProcessing import *
import scipy.io as scio
import numpy as np

class RGBDCamera(object):
    """
    A class which defines an RGBD camera structure together with calibration data capabilities.

    Its nothing but a container of two camera objects together with depth mapping routines.

    The camera(s) pose w.r.t. the World Coordinate system will be handled as follows. Two main poses are described:
        RGB camera pose w.r.t. the WCS: self.R_pos, self.T_pos
        Depth camera pose w.r.t. the RGB camera: self.R, self.T

    This fully defines the poses of both cameras w.r.t. to the WCS, and its easier to provide and interpret data
    referred to the WCS

    """
    def __init__(self):
        super(RGBDCamera, self).__init__()
        self.rgb_camera = camera()
        self.depth_camera = depth_camera()
        self.setDefaultParameters()

    def updateParameters(self):
        # It computes the depth camera's pose w.r.t. the WCS from the RGB-D bundle position and the pose of the depth
        # camera w.r.t. to the RGB camera
        R_depth = self.R_pos
        T_depth = self.T_pos
        if self.R is not None:
            if self.R_pos is not None:
                R_depth = np.dot(self.R_pos, self.R)
            else:
                R_depth = self.R
        if self.T is not None:
            if self.R_pos is not None:
                T_depth = np.dot(self.R_pos, self.T)
            else:
                T_depth = self.T
            if self.T_pos is not None:
                T_depth = T_depth + self.T_pos
        self.R_depth = R_depth
        self.T_depth = T_depth
        # This calls will make the camera's objects to keep references to the parameters arrays
        self.rgb_camera.setParameters(self.rgb_size[0], self.rgb_size[1], self.rgb_intrinsics, self.rgb_distorsion, self.R_pos, self.T_pos)
        self.depth_camera.setParameters(self.depth_size[0], self.depth_size[1], self.depth_intrinsics, self.depth_distorsion, self.R_depth, self.T_depth)
        self.depth_camera.setDepthParameters(self.k_coeff, self.alpha, self.beta, self.depthMM)

    def move(self, R=None, T=None):
        """
        Moves the entire RGB-D camera to the specified pose.

        This is convenient to handle a world coordinate system. Within the RGB-D ensemble, the RGB camera's focal point
        define the location of the RGB-D ensemble coordinate system. The updateParameters function (called here by
        default) makes sure to reposition the depth camera accordingly.
        """
        self.R_pos = R
        self.T_pos = T
        self.updateParameters()

    def setOpenNIParameters(self, pclStreamer):
        """
        Sets parameters according to OpenNI, which correspond to those of a connected device and streaming

        Here it's assumed that OpenNI handle most of the calibration things such that RGB and Depth are registered and
        the depth map is given in millimeters. This means that they also share the intrinsic parameters.
        """
        intrinsics = np.zeros((3, 3), dtype=np.float32)
        pclStreamer.getCameraIntrinsics (intrinsics)
        dimensions = np.int32(np.array([640, 480, 640, 480]))
        res = pclStreamer.getDimensions(dimensions)
        rgb_width, rgb_height, depth_width, depth_height = int(dimensions[0]), int(dimensions[1]), int(dimensions[2]), int(dimensions[3])
        self.rgb_size = (rgb_width, rgb_height)
        self.depth_size = (depth_width, depth_height)
        self.rgb_intrinsics = intrinsics
        self.rgb_distorsion = None
        self.depth_intrinsics = intrinsics
        self.depth_distorsion = None
        self.R = None  # OpenNI registers the data
        self.T = None  # OpenNI registers the data
        self.depthMM = True  # Already in millimeters
        self.beta = None
        self.alpha = None
        self.k_coeff = None
        self.setDefaultPose()
        # Calls to update the objects in the C++ side
        self.updateParameters()

    def setParametersFromFile(self, calibration_file=''):
        self.loadParameters(calibration_file)
        self.updateParameters()

    def setCalibration(self, calibration=None):
        """
        This function assigns calibration parameters
        """
        if calibration.__class__ is str:
            self.setParametersFromFile(calibration)
        elif calibration.__class__ is tuple:  # In case the parameters are simply given as a tuple
            if calibration[0].__class__ is str:
                self.setParametersFromFile(calibration)
            else:
                # The calibration is given directly from OpenNI
                self.rgb_intrinsics = calibration[0]
                self.rgb_distorsion = calibration[1]
                self.depth_intrinsics = calibration[2]
                self.depth_distorsion = calibration[3]
                self.R = calibration[4]
                self.T = calibration[5]
                self.beta = calibration[6]
                self.alpha = calibration[7]
                self.k_coeff = calibration[8]
                self.depthMM = calibration[9]
                self.rgb_size = calibration[10]
                self.depth_size = calibration[11]
                self.updateParameters()
        else:
            self.setDefaultParameters()


    def setDefaultParameters(self):
        """
        The set of parameters obtained by Nicolas Burrus (http://nicolas.burrus.name/)for one of his devices.

        Here we use them just as default values for the Kinect, however we expect much better parameters, either found
        by the user (e.g. calibrating using Daniel Herrera's toolbox) or provided by OpenNI
        """
        self.rgb_intrinsics = np.reshape(np.array([ 5.3009194943536181e+02, 0., 3.2821930715948992e+02, 0.,5.2635860167133876e+02, 2.6872781351282777e+02, 0., 0., 1. ],dtype=np.float32),(3,3))
        self.rgb_distorsion = np.reshape(np.array([  2.6172416643533958e-01, -8.2104703257074252e-01,-1.0637850248230928e-03, 8.4946289275097779e-04, 8.9012728224037985e-01],dtype=np.float32),(1,5))

        self.depth_intrinsics = np.reshape(np.array([ 5.9425464969100040e+02, 0., 3.3978729959351779e+02, 0., 5.9248479436384002e+02, 2.4250301427866111e+02, 0., 0., 1. ],dtype=np.float32),(3,3))
        self.depth_distorsion = np.reshape(np.array([-2.6167161458989197e-01, 9.9319844495479259e-01,-1.0221823575713733e-03, 5.3621541487535148e-03,-1.2966457585622253e+00 ],dtype=np.float32),(1,5))

        self.R = np.reshape(np.array([9.9977321644139494e-01, 1.7292658422779497e-03,
             -2.1225581878346968e-02, -2.0032487074002391e-03,
             9.9991486643051353e-01, -1.2893676196675344e-02,
             2.1201478274968936e-02, 1.2933272242365573e-02,
             9.9969156632836553e-01],dtype=np.float32),(3,3))

        self.T = np.reshape(np.array([2.1354778990792557e-02, 2.5073334719943473e-03,-1.2922411623995907e-02 ],dtype=np.float32),(3,1))
        self.k_coeff = np.reshape(np.array([  3.11993031e+00, -2.85193648e-03],dtype=np.float32),(1,2))#Coefficients from depth to meters 1/(self.k_coeff[1]* d + self.k_coeff[0])
        self.beta = None  # Matrix with correction of the disparity
        self.alpha = None
        self.rgb_size = (640, 480)
        self.depth_size = (640, 480)
        self.depthMM = False
        self.setDefaultPose()
        self.updateParameters()

    def loadTxtParameters(self, calFile):
        params = {}
        handler = file(calFile ,'r')
        lines = handler.readlines()
        handler.close()
        pos = 0
        while pos < len(lines):
            header = lines[pos].strip()
            if header[0] == '[' and header[-1]==']':
                header = header[1:-1]
            else:
                raise
            pos += 1
            done_pars = False
            vals = []
            while not done_pars and pos < len(lines):
                line = lines[pos]
                if line[0] == '[':
                    break
                vals.append( [float(val) for val in lines[pos].strip().split(';')])
                pos += 1
            params[header] = np.float32(np.array(vals))
        params['resolution'] = params['resolution'].flatten()
        params['resolution'] = (int(params['resolution'][0]), int(params['resolution'][1]) )
        if 'distorsion' in params.keys():
            params['distorsion'] = params['distorsion'].reshape(1,-1)
        else:
            params['distorsion'] = None
        if 'R' not in params.keys():
            params['R'] = None
        if 'T' not in params.keys():
            params['T'] = None

        return params

    def setDefaultPose(self):
        """
        Sets the default pose of the RGB-D ensemble. A minor transform to have a similar configuration to OpenGL
        """
        R = np.identity(3, dtype=np.float32)
        T = np.zeros((3, 1), dtype=np.float32)
        R[1, 1] = -1
        R[2, 2] = -1
        T[2, 0] = 1
        self.R_pos = R
        self.T_pos = T

    def checkObjectFromLoad(self, val):
        """
        Using numpy savez/load, objects are saved as an ndarray of "objects", this takes the object out of the array
        """
        if type(val) is np.ndarray:
            if val.dtype == object and val.size == 1:
                return val.reshape(1)[0]
        return val

    def loadParameters(self, calFile = None):
        """
        Loads the set of calibration parameters from files
        """
        if calFile.__class__ is tuple:
            # This is the case where separate txt files are given for each sensor
            rgb_params = self.loadTxtParameters(calFile[0])
            depth_params = self.loadTxtParameters(calFile[1])

            self.rgb_size = rgb_params['resolution']
            self.rgb_intrinsics = rgb_params['intrinsics']
            self.rgb_distorsion = rgb_params['distorsion']
            # In the case of txt files, the camera poses are with respect to the world coordinate system
            R_rgb = rgb_params['R']
            T_rgb = rgb_params['T']

            self.depth_size = depth_params['resolution']
            self.depth_intrinsics = depth_params['intrinsics']
            self.depth_distorsion = depth_params['distorsion']
            depth_R = depth_params['R']
            depth_T = depth_params['T']

            R_pos_inv = R_rgb.transpose()
            T_pos_inv = -np.dot(R_pos_inv, T_rgb)
            # We compute the relative position between both cameras, depth cam to RGB cam
            self.R = np.dot(R_pos_inv, depth_R)
            self.T = np.dot(R_pos_inv, depth_T) + T_pos_inv

            if 'k_coefficients' in depth_params.keys():
                self.k_coeff = depth_params['k_coefficients']
                self.depthMM = False
            else:
                self.k_coeff = None
                self.depthMM = True

            if 'beta' in depth_params.keys():
                self.beta = depth_params['beta']
            else:
                self.beta = None

            if 'alpha' in depth_params.keys():
                self.alpha = depth_params['alpha']
            else:
                self.alpha = None
            # Position the stereo ensemble anchored at the RGB camera pose
            self.R_pos = R_rgb
            self.T_pos = T_rgb
        elif calFile[-3:]=='npz':
            pars = np.load(calFile) # A numpy z file
            self.rgb_intrinsics = self.checkObjectFromLoad(pars['rgb_intrinsics'])
            self.rgb_distorsion = self.checkObjectFromLoad(pars['rgb_distorsion'])
            self.depth_intrinsics = self.checkObjectFromLoad(pars['depth_intrinsics'])
            self.depth_distorsion = self.checkObjectFromLoad(pars['depth_distorsion'])
            self.R = self.checkObjectFromLoad(pars['R'])
            self.T = self.checkObjectFromLoad(pars['T'])
            self.k_coeff = self.checkObjectFromLoad(pars['k_coeff'])
            self.beta = self.checkObjectFromLoad(pars['beta'])
            self.alpha = self.checkObjectFromLoad(pars['alpha'])
            self.depthMM = self.checkObjectFromLoad(pars['depthMM'])
            self.depthMM = True
            try:
                self.rgb_size = self.checkObjectFromLoad(pars['rgb_size'])
                self.depth_size = self.checkObjectFromLoad(pars['depth_size'])
                self.rgb_size = (int(self.rgb_size[0]), int(self.rgb_size[1]))
                self.depth_size = (int(self.depth_size[0]), int(self.depth_size[1]))
            except:
                self.rgb_size = (640, 480)
                self.depth_size = (640, 480)
            try:
                self.R_pos = self.checkObjectFromLoad(pars['R_pos'])
                self.T_pos = self.checkObjectFromLoad(pars['T_pos'])
            except:
                self.setDefaultPose()
        elif  calFile[-3:]=='mat':
            # Then it is a mat file (from the calibration toolbox made by Daniel Herrera)
            pars = scio.loadmat(calFile)
            self.rgb_intrinsics = np.float32(pars['rK'][0][0].copy())
            self.rgb_distorsion = np.float32(pars['rkc'][0][0].copy())
            self.depth_intrinsics = np.float32(pars['dK'].copy())
            self.depth_distorsion = np.float32(pars['dkc'].copy())
            self.R = np.float32(pars['dR'].copy())
            self.T = np.float32(pars['dt'].copy())

            # Coefficients to transform from disparity to meters
            try:
               # If provided by the toolbox
               self.k_coeff = np.float32(pars['dc'].copy())
               self.depthMM = False
            except:
               self.k_coeff = None
               self.depthMM = True
            try:
               # Undistortion of the disparity
               self.alpha = np.float32(pars['dc_alpha'].copy())
               self.beta = np.float32(pars['dc_beta'].copy())
            except:
               self.alpha = None
               self.beta = None
            try:
               self.rgb_size = np.int32(pars['rgb_size'].copy()).flatten()
               self.depth_size = np.int32(pars['depth_size'].copy()).flatten()
               self.rgb_size = int(self.rgb_size[0]), int(self.rgb_size[1])
               self.depth_size = int(self.depth_size[0]), int(self.depth_size[1])
            except:
               self.rgb_size = (640, 480)
               self.depth_size = (640, 480)
            self.setDefaultPose()
        elif calFile[-3:]=='cal':  # For the BIWI dataset
            file = open(calFile)
            self.rgb_intrinsics = []
            for i in range(3):
                line = file.readline().strip().split(' ')
                self.rgb_intrinsics.append([float(l) for l in line])
            self.rgb_intrinsics = np.float32(np.array(self.rgb_intrinsics).reshape(3,3))
            line = file.readline()
            line = file.readline().strip().split(' ')
            self.rgb_distorsion = np.zeros((1,5),dtype=np.float32)
            self.rgb_distorsion[0,:len(line)] = np.float32(np.array([float(l) for l in line]))
            line = file.readline()
            self.R = []
            for i in range(3):
                line = file.readline().strip().split(' ')
                self.R.append([float(l) for l in line])
            self.R = np.float32(np.array(self.R).reshape(3,3))
            line = file.readline()
            line = file.readline().strip().split(' ')
            self.T= np.float32(np.array([float(l) for l in line])).reshape(3,1)/1000.0
            file.close()
            # ---- Opens the depth file ----
            file = open(calFile[:-7]+'depth.cal')
            self.depth_intrinsics = []
            for i in range(3):
                line = file.readline().strip().split(' ')
                self.depth_intrinsics.append([float(l) for l in line])
            self.depth_intrinsics = np.float32(np.array(self.depth_intrinsics).reshape(3,3))
            line = file.readline()
            line = file.readline().strip().split(' ')
            self.depth_distorsion = np.zeros((1,5),dtype=np.float32)
            self.depth_distorsion[0,:len(line)] = np.float32(np.array([float(l) for l in line]))
            line = file.readline()
            self.R_d = []
            for i in range(3):
                line = file.readline().strip().split(' ')
                self.R_d.append([float(l) for l in line])
            self.R_d = np.float32(np.array(self.R_d).reshape(3,3))
            line = file.readline()
            line = file.readline().strip().split(' ')
            self.T_d= np.float32(np.array([float(l) for l in line])).reshape(3,1)/1000.0
            file.close()
            self.k_coeff = np.reshape(np.array([  3.11993031e+00, -2.85193648e-03],dtype=np.float32),(1,2))#Coefficients from depth to meters 1/(self.k_coeff[1]* d + self.k_coeff[0])
            self.beta = None # Matrix with correction of the disparity
            self.alpha = None
            self.rgb_size = (640, 480)
            self.depth_size = (640, 480)
            self.setDefaultPose()
        else:
            print 'WARNING: File format not recognized! Setting default calibration'
            self.setDefaultParameters()

    def saveCalibrationParameters(self, filename):
        np.savez(filename, rgb_intrinsics=self.rgb_intrinsics, rgb_distorsion=self.rgb_distorsion,
               depth_intrinsics=self.depth_intrinsics, depth_distorsion=self.depth_distorsion,
               R=self.R, T=self.T, k_coeff=self.k_coeff, beta=self.beta, alpha=self.alpha, depthMM=self.depthMM,
               R_pos=self.R_pos, T_pos=self.T_pos, rgb_size=self.rgb_size, depth_size=self.depth_size)


    def printParameters(self):
        print 'rgb_size', self.rgb_size, self.rgb_size.__class__, self.rgb_size[0].__class__
        print 'rgb_intrinsics' , self.rgb_intrinsics
        print 'rgb_distorsion' , self.rgb_distorsion
        print 'depth_size ', self.depth_size, self.depth_size.__class__, self.depth_size[0].__class__
        print 'depth_intrinsics' , self.depth_intrinsics
        print 'depth_distorsion' , self.depth_distorsion
        print 'R' , self.R
        print 'T' , self.T
        print 'beta' , self.beta
        print 'alpha' , self.alpha
        print 'k_coeff' , self.k_coeff
        print 'depthMM' , self.depthMM
        print 'R_pos', self.R_pos
        print 'T_pos', self.T_pos