# Find PyOpenGL
# 
# Copyright (c) 2014 Idiap Research Institute, http://www.idiap.ch
# Written by Kenneth Funes <kenneth.funes@idiap.ch>
#
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
#
# Find the installed version of PyOpenGL. The Python interpreter should have been found
# before calling FindOpenGL.
#
# After using it you will get the following variables
#
# PYOPENGL_FOUND - The flag indicating whether it was found in this machine
#
# PYOPENGL_FILE - The location of the .py file
#
# PYOPENGL_VERSION_STR - The version of PyOpenGL as a string
#
IF(EXISTS PYOPENGL_FILE)
  # Already in cache, be silent
  SET(PYOPENGL_FOUND TRUE)
ELSE(EXISTS PYOPENGL_FILE)
  FIND_FILE(_find_pyopengl_py FindPyOpenGL.py PATHS ${CMAKE_MODULE_PATH})
  EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} ${_find_pyopengl_py} OUTPUT_VARIABLE pyopengl_config)
  IF(pyopengl_config)
    STRING(REGEX REPLACE "^pyopengl_file:([^\n]+).*$" "\\1" PYOPENGL_FILE ${pyopengl_config})
    STRING(REGEX REPLACE ".*\npyopengl_version:([^\n]+).*$" "\\1" PYOPENGL_VERSION_STR ${pyopengl_config})
    SET(PYOPENGL_FOUND TRUE)
  ENDIF(pyopengl_config)

  IF(PYOPENGL_FOUND)
    IF(NOT PYOPENGL_FIND_QUIETLY)
      MESSAGE(STATUS "Found PyOpenGL at:" ${PYOPENGL_FILE})
    ENDIF(NOT PYOPENGL_FIND_QUIETLY)
  ELSE(PYOPENGL_FOUND)
    IF(PYOPENGL_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find PyOpenGL")
    ENDIF(PYOPENGL_FIND_REQUIRED)
  ENDIF(PYOPENGL_FOUND)
ENDIF(EXISTS PYOPENGL_FILE)

