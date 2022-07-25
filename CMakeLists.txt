# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the Apache-2.0 license

cmake_minimum_required(VERSION 3.5)
project(matlab-whole-body-simulator)

# There is nothing to build, only the a few MATLAB files to copy to the install folder:
# - the package 'lib/+mwbs' into the '<install prefix>/mex' folder.
# - The library folder 'lib/RobotDynamicsWithContacts' into '<install prefix>/mex/+mwbs'.
# - The URDF models in 'urdf' folder into '<install prefix>/share'.

set(MATLAB_WHOLE_BODY_SIMULATOR_INSTALL_MATLAB_FILESDIR "mex" CACHE
    STRING "Location (relative to the install prefix) in which the Matlab .m files and simulink .slx models are installed.")

set(MATLAB_WHOLE_BODY_SIMULATOR_INSTALL_URDF_FILESDIR "share" CACHE
    STRING "Location (relative to the install prefix) in which the URDF model files are installed.")

# Load custom macro and functions
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/ycm-0.12)

# Common source files
set(M_FILES_DIR ${matlab-whole-body-simulator_SOURCE_DIR}/lib/)
set(URDF_FILES_DIR ${matlab-whole-body-simulator_SOURCE_DIR}/urdf/RRbot)

# Copy the classes and Simulink slx library files
install(
  DIRECTORY ${M_FILES_DIR}
  DESTINATION ${MATLAB_WHOLE_BODY_SIMULATOR_INSTALL_MATLAB_FILESDIR}
  USE_SOURCE_PERMISSIONS)

# Copy robot models
install(
  DIRECTORY ${URDF_FILES_DIR}
  DESTINATION ${MATLAB_WHOLE_BODY_SIMULATOR_INSTALL_URDF_FILESDIR}
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*.dae"
  PATTERN "*.stl"
  PATTERN "*.urdf")

include(AddUninstallTarget)
