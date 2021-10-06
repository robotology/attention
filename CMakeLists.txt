# Copyright: (C) 2014 RBCS Robotics Brain and Cognitive Science
# Authors: Francesco Rea, Giorgio Metta, Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.12)

project (attention)
find_package(YARP REQUIRED COMPONENTS sig cv)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribHelpers)
include(ICUBcontribOptions)

icubcontrib_set_default_prefix() #set CMAKE_INSTALL_PREFIX to the ICUBcontrib one; print warning if user modifies it


# default prefix for applications
# set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")
SET(ATTENTIONLIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/attentionLib/include)

add_subdirectory(attentionLib)
add_subdirectory(src)


#icubcontrib_finalize_export(attention)
icubcontrib_set_default_prefix()
icubcontrib_add_uninstall_target()        
