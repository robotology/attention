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
find_package(ICUBcontrib REQUIRED)

set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH} 
  ${YARP_MODULE_PATH} 
  ${ICUB_MODULE_PATH} 
  ${ICUBCONTRIB_MODULE_PATH} # note 10/10/13 aligned to the standard Rea
  ) 


include(ICUBcontribHelpers)
icubcontrib_set_default_prefix() #set CMAKE_INSTALL_PREFIX to the ICUBcontrib one; print warning if user modifies it
#set_contrib_default_prefix()

include(YarpInstallationHelpers)

#find_package(IPP REQUIRED)
find_package(OpenCV REQUIRED)
find_package(GtkPlus REQUIRED)
find_package(GSL)
find_package(IPOPT)

#INCLUDE(iCubOptions)
#include(iCubHelpers) # removed 13/11/2013 
include(ICUBcontribOptions)


SET(CUDAVISION_INCLUDE_DIRS   ${CMAKE_CURRENT_SOURCE_DIR}/cudaVision/include)
SET(ATTENTIONLIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/attentionLib/include)

# default prefix for applications
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

### Go through single folder
option(ENABLE_CUDA  "Do you want to compile with Cuda?" OFF)

if (ENABLE_CUDA)  
    find_package(CUDA QUIET REQUIRED)
    add_definitions (-DWITH_CUDA)
    add_subdirectory(cudaVision)
endif (ENABLE_CUDA)

add_subdirectory(attentionLib)
add_subdirectory(app)
add_subdirectory(src)

icubcontrib_finalize_export(attentionLib) #macro added 13/11/2013
icubcontrib_add_uninstall_target()        #macro added 13/11/2013