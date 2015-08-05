# CMAKE configuration file. Designed for the RGB-D framework
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

cmake_minimum_required(VERSION 2.0)
if(COMMAND cmake_policy)
  cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

####################################################
##      SEARCH FOR DEPENDENCIES
####################################################
OPTION(CMAKE_VERBOSE_MAKEFILE ON)

# If this option stays OFF, then no connected device is expected. Only processing recorded data
OPTION(INCLUDE_PCL_DEVICE_SUPPORT "Include PCL/OpenNI support" OFF)
# The selection of the OpenNI version is here exclusive
OPTION(USE_OPENNI2 "Compile for OpenNI2 instead of OpenNI" OFF)
MESSAGE (" Path :" ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH  ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmakeModules")
MESSAGE ("Path :" ${CMAKE_MODULE_PATH})
FIND_PACKAGE(PythonInterp 2.7 REQUIRED)
FIND_PACKAGE(PythonLibs 2.7 REQUIRED)
FIND_PACKAGE(SWIG REQUIRED)
FIND_PACKAGE(Numpy REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(OpenCV 2.4 REQUIRED)
FIND_PACKAGE(OpenGL REQUIRED)
FIND_PACKAGE(PyOpenGL REQUIRED)
FIND_PACKAGE(PySide REQUIRED)
FIND_PACKAGE(Boost REQUIRED thread system)
IF(INCLUDE_PCL_DEVICE_SUPPORT)
    FIND_PACKAGE(PCL 1.7)
    IF(PCL_FOUND)
      include_directories(${PCL_INCLUDE_DIRS})
      link_directories(${PCL_LIBRARY_DIRS})
      SET(PCL_OPENNI_DEVICE_SUPPORT ON)
    ENDIF(PCL_FOUND)
ENDIF(INCLUDE_PCL_DEVICE_SUPPORT)

link_directories(${Boost_LIBRARY_DIRS})

################### Finds OS ####################
IF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
   SET(MY_OS "OS X")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

IF(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    SET(MY_OS "Linux")
ENDIF(${CMAKE_SYSTEM_NAME} MATCHES "Linux") 

###############  Python ##################
INCLUDE_DIRECTORIES(${PYTHON_INCLUDE_PATH})

############## Numpy ########################
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${PYTHON_NUMPY_INCLUDE_DIR})

######################## SWIG ###############################
INCLUDE(${SWIG_USE_FILE})
SET(CMAKE_SWIG_FLAGS "")
SET(GLOBAL_SWIG_LINK_FLAGS "")

if(MY_OS STREQUAL "OS X")
  SET(GLOBAL_SWIG_LINK_FLAGS "-undefined suppress -flat_namespace")
endif(MY_OS STREQUAL "OS X")

MACRO(MYSWIG_ADD_MODULE name)
  # As it is a macro, even invoked from a subfolder, variables such as cmake_current_source_dir would be well defined
  SET(CMAKE_SWIG_OUTDIR "${CMAKE_CURRENT_SOURCE_DIR}")
  SET_SOURCE_FILES_PROPERTIES(${name}.i PROPERTIES CPLUSPLUS ON)
  SWIG_ADD_MODULE(${name} python ${name}.i ${name}.cpp ${name}.h)
  INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR} ${OpenCV_INCLUDE_DIRS})
  SET_TARGET_PROPERTIES(${SWIG_MODULE_${name}_REAL_NAME} PROPERTIES LINK_FLAGS "${GLOBAL_SWIG_LINK_FLAGS}" LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}")
  SWIG_LINK_LIBRARIES(${name} ${PYTHON_LIBRARIES} ${OpenCV_LDFLAGS} ${Boost_LIBRARIES})
ENDMACRO(MYSWIG_ADD_MODULE)

######### Configuration files for the project ########
# Create a configuration header for the project
configure_file(
  "${PROJECT_SOURCE_DIR}/config.h.in"
  "${PROJECT_SOURCE_DIR}/config.h"
)
configure_file(
  "${PROJECT_SOURCE_DIR}/config.py.in"
  "${PROJECT_SOURCE_DIR}/config.py"
)
##################################################
##    Adding subdirectories for the project
##################################################
add_subdirectory(Common)
add_subdirectory(Streaming)
add_subdirectory(Rendering)

