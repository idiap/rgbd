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
from PySide import QtCore, QtGui, QtOpenGL
from OpenGL import GL
from OpenGL.arrays import vbo
import numpy as np

class Viewer(QtOpenGL.QGLWidget):
        """
        Qt Widget which is able to render 3D meshes.

        This is class which extend the Qt QGLWidget by allowing a background
        rendering together with a list of meshes. These meshes are consistent
        with the definition of the vertices, mesh and model classes. Additionally
        their position in the world coordinate system can be specified.

        """
        updateSignal = QtCore.Signal()
        def __init__(self, parent=None):
                """ Constructor, just an empty list of objects to be rendered, no background """
                super(Viewer, self).__init__(parent)
                self.objects = []
                self.renderMode = 'F'
                self.str = None
                # This is a global transformation, which affects all the objects at once (except for the background, which is always fixed)
                self.pose = np.zeros(7, dtype=np.float32)
                self.pose[2] = -10.0
                self.pose[6] = 2.0 #scale
                self.fps = 1
                # Refreshing
                self.timer = QtCore.QTimer(self)
                self.connect(self.timer, QtCore.SIGNAL("timeout()"),self.updateGL)
                self.updateSignal.connect(self.updateGL)
                self.ambient = (1.0,1.0,1.0,1.0)
                self.diffuse = (1.0,1.0,1.0,1.0)
                self.specular = (0.0,0.0,0.0,1.0)
                self.position = (0,0,0.0,0.0)
                #self.timer.start(1000/self.fps)

        def __del__(self):
            """ Destroy the defined buffers for the background """
            if hasattr(self,'pbo'):
                GL.glDeleteBuffers(1,self.pbo)
            if hasattr(self,'texid'):
                GL.glDeleteTextures(self.texid)

        def minimumSizeHint(self):
                """ Returns the minimum size of the widget """ 
                return QtCore.QSize(480, 480)
        
        def sizeHint(self):
                """ Returns the size of the widget """  
                return QtCore.QSize(480, 480)
        
## Rendering objects manipulation

        def addObject(self, obj, pose=None):
            """ Add a new object to the list of meshes to render """
            # Should use a Mutex TODO to control adding and removing and editing objects
            theId = 0
            if self.objects.count(None) > 0:
                theId = self.objects.index(None)
                self.objects[theId] = OpenGLObject(obj,pose)
            else:
                self.objects.append(OpenGLObject(obj,pose))
                theId = len(self.objects)-1 # Return an "id" for the added object
            return theId
        
        def clean(self):
            """ Removes all the objects """
            self.objects = []
        
        def removeObject(self, theId):
            """ Of the added meshes, remove the one with the id theId """
            # Should use a Mutex TODO to control adding and removing and editing objects
            self.objects[theId] = None#The garbage collector should take care of it

        def setBackground(self, aStreamer=None):
            """ Sets an streamer object as the one who provides the background image """
            self.str = aStreamer
            if self.str:
                GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, self.pbo)
                GL.glBufferData(GL.GL_PIXEL_UNPACK_BUFFER, self.str.current(), GL.GL_DYNAMIC_DRAW)
                GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, 0)

        def updateBackground(self):
            """ Updates the Background image """
            if self.str:
                GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, self.pbo)
                GL.glBufferSubData( GL.GL_PIXEL_UNPACK_BUFFER , 0 , self.str.current())
                GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, 0)
                self.updateGL()

        def setRenderingType(self, typ):
            """ Changes between different types of rendering: Filled, Lines, Points """
            self.renderMode = typ
            self.updateGL()
            
## OpenGL main functions definition

        def initializeGL(self):
            """ Set OpenGL variables """
            self.qglClearColor(QtGui.QColor(250,250,250,255))
            #GL.glShadeModel(GL.GL_FLAT)
            GL.glShadeModel(GL.GL_SMOOTH)
            GL.glEnable(GL.GL_DEPTH_TEST)
            GL.glEnable(GL.GL_POINT_SMOOTH )
            #GL.glEnable(GL.GL_CULL_FACE)
            # Creates the pixel buffer object for the background and loads image data into it
            self.pbo = long(GL.glGenBuffers(1))
            GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, self.pbo)
            if self.str:
                GL.glBufferData(GL.GL_PIXEL_UNPACK_BUFFER, self.str.current(), GL.GL_DYNAMIC_DRAW)
            # Creates the texture object for the background
            self.texid = long(GL.glGenTextures(1))
            GL.glBindTexture(GL.GL_TEXTURE_2D, self.texid)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_NEAREST)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_NEAREST)
            GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT,1)
            # Loads image data from the PBO to the texture object
            if self.str:
                GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, 3, self.str.width, self.str.height, 0, GL.GL_RGB, GL.GL_UNSIGNED_BYTE, GL.GLvoid)
            # Unbinds everything
            GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
            GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, 0)

        def paintGL(self):
            """ Thread safe slot, the event loop from Qt for this thread is the same for OpenGL """
            self.makeCurrent()
            GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
            # Render video/background
            if self.str:
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
                GL.glLoadIdentity()
                GL.glEnable(GL.GL_TEXTURE_2D)
                GL.glTexEnvf(GL.GL_TEXTURE_ENV, GL.GL_TEXTURE_ENV_MODE, GL.GL_DECAL)
                GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, self.pbo)
                GL.glBindTexture(GL.GL_TEXTURE_2D, self.texid)
                GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, 3, self.str.width, self.str.height, 0, GL.GL_RGB, GL.GL_UNSIGNED_BYTE, GL.GLvoid)
                GL.glBegin(GL.GL_QUADS)
                width, height, depth = 1, 1, 24.999
                GL.glTexCoord2f(1.0, 1.0)
                GL.glVertex3f(width,height, -depth)
                GL.glTexCoord2f(0.0, 1.0)
                GL.glVertex3f(-width,height, -depth)
                GL.glTexCoord2f(0.0, 0.0)
                GL.glVertex3f(-width,-height, -depth)
                GL.glTexCoord2f(1.0, 0.0)
                GL.glVertex3f(width,-height, -depth)
                GL.glEnd()
                GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
                GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, 0)
            if self.renderMode == 'L':
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_LINE)
            elif self.renderMode == 'P':
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_POINT)
            else:
                GL.glPolygonMode(GL.GL_FRONT_AND_BACK, GL.GL_FILL)
            # Sets identity for positioning the lighting
            GL.glLoadIdentity()
            # Set a configuration for the lighting conditions
            GL.glEnable(GL.GL_LIGHTING)
            GL.glEnable(GL.GL_LIGHT0)
            GL.glLightfv(GL.GL_LIGHT0, GL.GL_AMBIENT, self.ambient)
            GL.glLightfv(GL.GL_LIGHT0, GL.GL_DIFFUSE, self.diffuse)
            GL.glLightfv(GL.GL_LIGHT0, GL.GL_SPECULAR, self.specular)
            GL.glLightfv(GL.GL_LIGHT0, GL.GL_POSITION, self.position)#(0,0,0.0,0.0))
            GL.glEnable(GL.GL_COLOR_MATERIAL)
            GL.glColorMaterial(GL.GL_FRONT_AND_BACK, GL.GL_AMBIENT_AND_DIFFUSE)
            #print 'Repainting!'
            # Global transformation
            if self.pose.size == 16:
                GL.glMultMatrixf(self.pose.transpose())
                GL.glScalef(4.0,4.0,4.0)
            else:
                GL.glTranslated(self.pose[0], self.pose[1], self.pose[2])
                GL.glRotated(self.pose[3] , 1.0, 0.0, 0.0)
                GL.glRotated(self.pose[4] , 0.0, 1.0, 0.0)
                GL.glRotated(self.pose[5] , 0.0, 0.0, 1.0)
                GL.glScalef(self.pose[6],self.pose[6],self.pose[6])
            
            ## Render overlaying objects
            # Should use a Mutex TODO to control adding and removing objects
            for obj in self.objects:
                if obj:
                    GL.glEnable(GL.GL_LIGHTING)
                    
                    if hasattr(obj.mesh,'texture') or hasattr(obj.mesh,'color'):
                        #print 'white ',obj.mesh.__class__, obj.mesh.vcs_q
                        GL.glColor3f(1.0, 1.0, 1.0)
                        if hasattr(obj.mesh,'transparency'):
                            GL.glEnable(GL.GL_BLEND)
                            GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
                            GL.glColor4f(1.0, 1.0, 1.0,obj.mesh.transparency)
                    else:
                        GL.glColor3f(0.3, 0.6, 1.0)
                        if hasattr(obj.mesh,'transparency'):
                            GL.glEnable(GL.GL_BLEND)
                            GL.glBlendFunc(GL.GL_SRC_ALPHA, GL.GL_ONE_MINUS_SRC_ALPHA)
                            GL.glColor4f(0.3, 0.6, 1.0,obj.mesh.transparency)
                            
                    if hasattr(obj.mesh,'GLmaterial'):
                        GL.glMaterialfv(GL.GL_FRONT_AND_BACK,GL.GL_AMBIENT,obj.mesh.GLmaterial.ambient)
                        GL.glMaterialfv(GL.GL_FRONT_AND_BACK,GL.GL_DIFFUSE,obj.mesh.GLmaterial.diffuse)
                        GL.glMaterialfv(GL.GL_FRONT_AND_BACK,GL.GL_SPECULAR,obj.mesh.GLmaterial.specular)
                        GL.glMaterialf(GL.GL_FRONT_AND_BACK,GL.GL_SHININESS,obj.mesh.GLmaterial.shininess)
                    else:#Use the default material
                        GL.glMaterialfv(GL.GL_FRONT_AND_BACK,GL.GL_AMBIENT,(0.1,0.1,0.1,1.0))
                        GL.glMaterialfv(GL.GL_FRONT_AND_BACK,GL.GL_DIFFUSE,(1.0,1.0,1.0,1.0))
                        GL.glMaterialfv(GL.GL_FRONT_AND_BACK,GL.GL_SPECULAR,(0.0,0.0,0.0,1.0))
                        #GL.glMaterialf(GL.GL_FRONT_AND_BACK,GL.GL_SHININESS,10)
                    # Modify to the object position
                    if hasattr(obj.mesh, 'renderPose'):
                        # If for renderig the mesh has a special pose
                        GL.glPushMatrix()
                        GL.glMultMatrixf(obj.mesh.renderPose.transpose())
                    GL.glPushMatrix()
                    if obj.pose.size == 16:
                        GL.glMultMatrixf(obj.pose.transpose())
                    else:
                        GL.glTranslated(obj.pose[0], obj.pose[1], obj.pose[2])
                        GL.glRotated(obj.pose[3] , 1.0, 0.0, 0.0)
                        GL.glRotated(obj.pose[4] , 0.0, 1.0, 0.0)
                        GL.glRotated(obj.pose[5] , 0.0, 0.0, 1.0)
                    GL.glEnableClientState(GL.GL_VERTEX_ARRAY)
                    #dataUploaded = obj.vbo.copied
                    obj.vbo.bind() # Uploads data if it hasn't been done before
                    #if not dataUploaded and obj.vbo.copied:
                    #  print 'A bulk of data has been copied for ', obj.mesh.__class__
                    GL.glVertexPointer(3, GL.GL_FLOAT, 0, GL.GLvoid)
                    if hasattr(obj.mesh, 'color'):
                        GL.glEnableClientState(GL.GL_COLOR_ARRAY)
                        colorChannels = 3
                        if hasattr(obj.mesh,'transparency'):
                            colorChannels = 4
                        if hasattr(obj.mesh, 'faces'):
                            if obj.mesh.type is 'lines':
                                GL.glColorPointer(colorChannels, GL.GL_FLOAT,0,obj.vbo+int(obj.mesh.vcs_q*12))# The color is right after the vertices
                            else:
                                GL.glColorPointer(colorChannels, GL.GL_FLOAT,0,obj.vbo+obj.mesh.vcs_q*12+obj.mesh.vcs_q*12)#Packed after vertices and normals
                        else:
                            GL.glColorPointer(colorChannels, GL.GL_FLOAT,0,obj.vbo+obj.mesh.vcs_q*12)#Packed right after vertices
                    elif hasattr(obj.mesh,'texture'):
                        obj.uploadTexture()# Uploads only when necessary (externally modified)
                        GL.glEnable(GL.GL_TEXTURE_2D)
                        GL.glTexEnvf(GL.GL_TEXTURE_ENV, GL.GL_TEXTURE_ENV_MODE, GL.GL_MODULATE)
                        GL.glBindTexture(GL.GL_TEXTURE_2D, obj.meshtexid)
                        GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, obj.txpbo)
                        GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, 3, obj.mesh.txwidth, obj.mesh.txheight, 0, GL.GL_RGB, GL.GL_UNSIGNED_BYTE, GL.GLvoid)
                        GL.glEnableClientState(GL.GL_TEXTURE_COORD_ARRAY)
                        GL.glTexCoordPointer(2, GL.GL_FLOAT,0,obj.vbo+obj.mesh.vcs_q*12+obj.mesh.vcs_q*12)
                    if hasattr(obj.mesh, 'faces'):
                        obj.uploadIndices()# Uploads only when necessary (externally modified)
                        GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, obj.vboidx)
                        if obj.mesh.type is 'lines':
                            GL.glDisable(GL.GL_LIGHTING)# To not change the color of the lines when rotating
                            GL.glColor3f(0.0, 0.0, 0.0)# Always black lines
                            GL.glLineWidth(3.0);
                            GL.glDrawElements(GL.GL_LINES, obj.mesh.faces_q*2, GL.GL_UNSIGNED_INT, GL.GLvoid )
                        else:
                            GL.glEnableClientState(GL.GL_NORMAL_ARRAY)
                            GL.glNormalPointer(GL.GL_FLOAT, 0, obj.vbo+obj.mesh.vcs_q*12)
                            GL.glDrawElements(GL.GL_TRIANGLES, obj.mesh.faces_q*3, GL.GL_UNSIGNED_INT, GL.GLvoid )
                    else:
                        GL.glPointSize(2.0)
                        GL.glDrawArrays( GL.GL_POINTS, 0, obj.mesh.vcs_q)
                    GL.glDisableClientState(GL.GL_VERTEX_ARRAY)
                    GL.glDisableClientState(GL.GL_NORMAL_ARRAY)
                    GL.glDisableClientState(GL.GL_COLOR_ARRAY)
                    GL.glDisableClientState(GL.GL_TEXTURE_COORD_ARRAY)
                    obj.vbo.unbind()
                    GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0)
                    GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
                    GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, 0)
                    GL.glPopMatrix()
                    if hasattr(obj.mesh, 'renderPose'):
                        GL.glPopMatrix()
                    GL.glDisable(GL.GL_BLEND)
                    #GL.glDisable(GL.GL_COLOR_MATERIAL)

        def resizeGL(self, width, height):
            side = min(width, height)
            if side < 0:
                return
            GL.glViewport(0, 0, width, height)
            GL.glMatrixMode(GL.GL_PROJECTION)
            GL.glLoadIdentity()
            #it's better an orthographic projection in visualization, to avoid perspective
            #distorsions between the front objects and the background video
            GL.glOrtho(-1,1,-1,1,0.001,25)
            GL.glMatrixMode(GL.GL_MODELVIEW) 

class OpenGLObject():
    """ 
    This is a class which holds an object such as a mesh, cloud of points or a model and
    takes care of handling proper OpenGL buffers, used when rendering the object in the
    specified position. It is thread safe as it is intended to use OpenGL operations by
    the viewer class, the modifications of the mesh can be done outside 
    """
        
    def __init__(self, obj, pose=None):
        """ Constructor """  
        self.mesh = obj # Here "mesh" is used for consistency, but it can be a "vertices" or "model" class
        if pose is None:
            # Pose will be x,y,z,xRot,yRot,zRot,scale
            self.pose = np.zeros(7,dtype=np.float32)
            self.pose[6] = 1.0 #scale
        else:
            self.pose = pose
        # Load the vertices data into the vertex buffer object
        self.data = self.mesh.packArrayData()
        #print '>> The Type of data in the constructor is ', self.data.dtype
        # Rescale the vertices according to the scale (only the vertices) #TODO Check if this scaling alters the original buffer
        if self.pose.size == 7:# pose can be a 4x4 matrix
            self.data[:self.mesh.vcs_q*3] = self.pose[6]*self.data[:self.mesh.vcs_q*3]
        self.vbo = vbo.VBO(self.data)
        if hasattr(self.mesh, 'faces'):
            self.indicesUploaded = False
        if hasattr(self.mesh, 'texture'):
            self.textureUploaded = False
    
    def __del__(self):
        """ Makes sure to destroy all the buffers that were created """
        self.vbo.delete()
        if hasattr(self,'vboidx'):
            GL.glDeleteBuffers(1,self.vboidx)
        if hasattr(self,'txpbo'):
            GL.glDeleteBuffers(1,self.txpbo)
        if hasattr(self,'meshtexid'):
            GL.glDeleteTextures(self.meshtexid)

    def uploadIndices(self):
        """ Load the indices into its corresponding buffer object """
        if not self.indicesUploaded:
            if not hasattr(self,'vboidx'):
                self.vboidx = long(GL.glGenBuffers(1))
            GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, self.vboidx)
            GL.glBufferData(GL.GL_ELEMENT_ARRAY_BUFFER, self.mesh.faces, GL.GL_STATIC_DRAW)
            GL.glBindBuffer(GL.GL_ELEMENT_ARRAY_BUFFER, 0)
            self.indicesUploaded = True

    def uploadTexture(self):
        """ Creates the texture object """
        if not self.textureUploaded:
            if not hasattr(self,'meshtexid'):
                self.meshtexid =long(GL.glGenTextures(1))
            # Creates the pixel buffer object
            if not hasattr(self,'txpbo'):
                self.txpbo = long(GL.glGenBuffers(1))
            GL.glBindTexture(GL.GL_TEXTURE_2D, self.meshtexid)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MAG_FILTER, GL.GL_LINEAR)
            GL.glTexParameterf(GL.GL_TEXTURE_2D, GL.GL_TEXTURE_MIN_FILTER, GL.GL_LINEAR)
            GL.glPixelStorei(GL.GL_UNPACK_ALIGNMENT,1)
            GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, self.txpbo)
            # Uploads texture data to the PBO
            GL.glBufferData(GL.GL_PIXEL_UNPACK_BUFFER, self.mesh.getTexture(), GL.GL_DYNAMIC_DRAW)
            # Copy data from PBO to Texture object
            GL.glTexImage2D(GL.GL_TEXTURE_2D, 0, 3, self.mesh.txwidth, self.mesh.txheight, 0, GL.GL_RGB, GL.GL_UNSIGNED_BYTE, GL.GLvoid)
            # Unbinds
            GL.glBindTexture(GL.GL_TEXTURE_2D, 0)
            GL.glBindBuffer(GL.GL_PIXEL_UNPACK_BUFFER, 0)
            self.textureUploaded = True

    def updateGeometryTexture(self, newMesh):
        self.mesh = newMesh
        self.updateVertices()
        self.updateNormals()
        self.updateTxCoord()
        self.textureUploaded = False
        #self.indicesUploaded = False

    def updateTxCoord(self):
        self.vbo[self.mesh.vcs_q*6:self.mesh.vcs_q*8] = self.mesh.txcoord.flatten()

    def updateVertices(self):
        """ Loads new vertices data into the VBO """
        self.vbo[:self.mesh.vcs_q*3] = self.pose[6]*self.mesh.vcs.flatten()

    def updateNormals(self):
        self.vbo[self.mesh.vcs_q*3:self.mesh.vcs_q*6] = self.mesh.vnorms.flatten()

    def updateColors(self):
     """ Updates new colors data for display """
     if hasattr(self.mesh,'faces'):
         if self.mesh.faces.shape[1] < 3:
             self.vbo[self.mesh.vcs_q*3:] = self.mesh.color.flatten()# For lines and points, the color is right after the vertices coordinates, no normals
         else:
             self.vbo[self.mesh.vcs_q*6:] = self.mesh.color.flatten()
     else:
         self.vbo[self.mesh.vcs_q*3:] = self.mesh.color.flatten()

    def setNewMesh(self, obj, pose=None):
        """ Sets a new mesh """
        self.mesh = obj # Here "mesh" is used for consistency, but it can be a "vertices" or "model" class, but
        # it should be of the same type as the one being replaced
        if pose is None:
            # Pose will be x,y,z,xRot,yRot,zRot,scale
            self.pose = np.zeros(7,dtype=np.float32)
            self.pose[6] = 1.0 #scale
        else:
            self.pose = pose
        data = self.mesh.packArrayData()
        # Rescale the vertices according to the scale (only the vertices)
        data[:self.mesh.vcs_q*3] = self.pose[6]*data[:self.mesh.vcs_q*3]
        self.vbo.set_array(data)
        if hasattr(self.mesh, 'faces'):
            self.indicesUploaded = False
        if hasattr(self.mesh, 'texture'):
            self.textureUploaded = False

    

