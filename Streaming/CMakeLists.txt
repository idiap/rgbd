# CMAKE configuration file. Designed for the face3Dtracking framework
#
# Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch
# Written by Kenneth Funes <kenneth.funes@idiap.ch>
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>

###### RGBD Streaming ######
SET(MYRGBD_SOURCES RGBDCalibration.cpp RGBDContainer.cpp camera.cpp)
SET(MYRGBD_HEADERS RGBDCalibration.h RGBDContainer.h camera.h)

# Creating a SWIG module called RGBDProcessing to be imported from Python

SET(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_SOURCE_DIR}")
SET_SOURCE_FILES_PROPERTIES(RGBDProcessing.i PROPERTIES CPLUSPLUS ON)
SWIG_ADD_MODULE(RGBDProcessing python RGBDProcessing.i ${MYRGBD_SOURCES} ${MYRGBD_HEADERS})
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
GET_TARGET_PROPERTY(INCLUDE_STUFF ${SWIG_MODULE_RGBDProcessing_REAL_NAME} INCLUDE_DIRECTORIES)
SET_TARGET_PROPERTIES(${SWIG_MODULE_RGBDProcessing_REAL_NAME} PROPERTIES LINK_FLAGS "${GLOBAL_SWIG_LINK_FLAGS}" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
SWIG_LINK_LIBRARIES(RGBDProcessing ${PYTHON_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LDFLAGS} ${BOOST_LIBRARIES})
