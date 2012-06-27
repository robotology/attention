# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta,Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
project (logpolarAttention)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

set(CMAKE_MODULE_PATH ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf) 

#find_package(IPP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GtkPlus REQUIRED)
find_package(IPOPT)

include(iCubOptions)
include(iCubHelpers)

SET(CUDAVISION_INCLUDE_DIRS   ${CMAKE_CURRENT_SOURCE_DIR}/cudaVision/include)
SET(ATTENTIONLIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/attentionLib/include)

# default prefix for applications
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

### Go through single folder
option(ENABLE_CUDA  "Do you want to compile with Cuda?" OFF)

if (ENABLE_CUDA)  
  add_definitions (-DWITH_CUDA)
  add_subdirectory(cudaVision)
endif (ENABLE_CUDA)

ADD_SUBDIRECTORY(attentionLib)
add_subdirectory(app)
add_subdirectory(src)

