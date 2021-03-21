#
#   Copyright 2020 CNRS INRIA
#
#   Author: Guilhem Saurel, Rohan Budhiraja
#

# Try to find cppad
# in standard prefixes and in ${cppad_PREFIX}
# Once done this will define
#  cppad_FOUND - System has cppad
#  cppad_INCLUDE_DIR - The cppad include directories
#  cppad_LIBRARY - The libraries needed to use cppad
#  cppad_DEFINITIONS - Compiler switches required for using cppad
#  cppad_VERSION - Version of cppad found

FIND_PATH(cppad_INCLUDE_DIR
  NAMES cppad/configure.hpp
  PATHS ${cppad_PREFIX}
  PATH_SUFFIXES include
  )
FIND_LIBRARY(cppad_LIBRARY
  NAMES cppad_lib
  PATHS ${cppad_PREFIX}
  PATH_SUFFIXES lib
  )

IF(cppad_INCLUDE_DIR AND EXISTS "${cppad_INCLUDE_DIR}/cppad/configure.hpp")
  file(STRINGS "${cppad_INCLUDE_DIR}/cppad/configure.hpp" cppad_version_str
    REGEX "^# *define[\t ]+CPPAD_PACKAGE_STRING[\t ]+\"cppad-.*\"")
  string(REGEX REPLACE "^# *define[\t ]+CPPAD_PACKAGE_STRING[\t ]+\"cppad-([^\"]*)\".*" "\\1"
    cppad_VERSION "${cppad_version_str}")
ENDIF()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(cppad REQUIRED_VARS cppad_LIBRARY cppad_INCLUDE_DIR VERSION_VAR cppad_VERSION)
mark_as_advanced(cppad_INCLUDE_DIR cppad_LIBRARY)
