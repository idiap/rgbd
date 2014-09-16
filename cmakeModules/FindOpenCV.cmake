# - Try to find Opencv based on pck-config, main method for using opencv in a
#   C/C++ project.
#
#  This will define the following variables
#  OpenCV_FOUND - system has OpenCV
#  OpenCV_LIBRARIES - name of the libraries to be linked against
#  OpenCV_LIBRARIES_DIRS - their location, if not standard
#  OpenCV_LDFLAGS - mainly adding the libraries
#  OpenCV_LDFLAGS_OTHERS
#  OpenCV_INCLUDE_DIRS 
#  OpenCV_CFLAGS -mainly including the file
#  OpenCV_CFLAGS_OTHER
#  OpenCV_PREFIX
# 
#  Copyright (c) 2011 Idiap Research Institute, http://www.idiap.ch/
#  Written by Kenneth Funes <kenneth.funes@idiap.ch>
#  This program is free software: you can redistribute it and/or modify
#  it under the terms of the GNU Lesser General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU Lesser General Public License for more details.
#
#  You should have received a copy of the GNU Lesser General Public License
#  along with this program.  If not, see <http://www.gnu.org/licenses/>

FIND_PACKAGE(PkgConfig)
if(OpenCV_FIND_REQUIRED)
  pkg_check_modules(OpenCV REQUIRED opencv)
else(OpenCV_FIND_REQUIRED)
  pkg_check_modules(OpenCV opencv)
endif(OpenCV_FIND_REQUIRED)

