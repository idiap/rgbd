# Find PySide
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
# Find the installed version of PySide. The Python interpreter should have been found
# before calling FindPySide.
#
# After using it you will get the following variables
#
# PYSIDE_FOUND - The flag indicating whether it was found in this machine
#
# PYSIDE_FILE - The location of the .py file
#
# PYSIDEL_VERSION_STR - The version of PySide as a string
#
IF(EXISTS PYSIDE_FILE)
  # Already in cache, be silent
  SET(PYSIDE_FOUND TRUE)
ELSE(EXISTS PYSIDE_FILE)

  FIND_FILE(_find_pyside_py FindPySide.py PATHS ${CMAKE_MODULE_PATH})

  EXECUTE_PROCESS(COMMAND ${PYTHON_EXECUTABLE} ${_find_pyside_py} OUTPUT_VARIABLE pyside_config)
  IF(pyside_config)
    STRING(REGEX REPLACE "^pyside_file:([^\n]+).*$" "\\1" PYSIDE_FILE ${pyside_config})
    STRING(REGEX REPLACE ".*\npyside_version:([^\n]+).*$" "\\1" PYSIDE_VERSION_STR ${pyside_config})
    SET(PYSIDE_FOUND TRUE)
  ENDIF(pyside_config)

  IF(PYSIDE_FOUND)
    IF(NOT PYSIDE_FIND_QUIETLY)
      MESSAGE(STATUS "Found PySide at:" ${PYSIDE_FILE})
    ENDIF(NOT PYSIDE_FIND_QUIETLY)
  ELSE(PYSIDE_FOUND)
    IF(PYSIDE_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find PySide")
    ENDIF(PYSIDE_FIND_REQUIRED)
  ENDIF(PYSIDE_FOUND)
ENDIF(EXISTS PYSIDE_FILE)

