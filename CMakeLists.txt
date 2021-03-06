# Copyright: (C) 2014 RBCS Robotics Brain and Cognitive Science
# Authors: Francesco Rea, Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
if(COMMAND cmake_policy)
    cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

project (attention)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})
include(YarpInstallationHelpers)
message("PROJECT_SOURCE_DIR: ${PROJECT_SOURCE_DIR} ")
include(${PROJECT_SOURCE_DIR}/conf/iCubUseQt5.cmake)
#include(iCubUseQt5)

find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

set(CMAKE_MODULE_PATH 
  ${CMAKE_MODULE_PATH} 
  ${YARP_MODULE_PATH} 
  ${ICUB_MODULE_PATH} 
  ${ICUBCONTRIB_MODULE_PATH} # note 10/10/13 aligned to the standard Rea
  ) 


include(ICUBcontribHelpers)
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
include(ICUBcontribHelpers)
include(ICUBcontribOptions)
icubcontrib_set_default_prefix() #set CMAKE_INSTALL_PREFIX to the ICUBcontrib one; print warning if user modifies it

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
add_definitions(${YARP_DEFINES})
include(YarpInstallationHelpers)

find_package(Qt3)
include(FindGTK2)

find_package(GTK)
find_package(Qt5 COMPONENTS Core Widgets OpenGL Quick Qml Concurrent PrintSupport QUIET)

find_package(OpenCV REQUIRED)
message(STATUS "OpenCV version is ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}")
find_package(IPOPT)

find_package(logpolar QUIET)
IF (logpolar_FOUND)
   message("Attention System: logpolar library detected and used.")
   message ("logpolar_INCLUDE_DIRS : ${logpolar_INCLUDE_DIRS};")
ELSE ()
   message("Attention System: logpolar library undetected")
ENDIF ()


SET(CUDAVISION_INCLUDE_DIRS   ${CMAKE_CURRENT_SOURCE_DIR}/cudaVision/include)
SET(ATTENTIONLIB_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/attentionLib/include)

# default prefix for applications
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

### Go through single folder
option(ENABLE_CUDA  "Do you want to compile with Cuda?" OFF)
option(ENABLE_GTK2  "Do you want to compile with GTK2?" OFF)

if (ENABLE_CUDA)  
    find_package(CUDA QUIET REQUIRED)
    add_definitions (-DWITH_CUDA)
    add_subdirectory(cudaVision)
endif (ENABLE_CUDA)

add_subdirectory(attentionLib)
add_subdirectory(app)
add_subdirectory(src)

icubcontrib_finalize_export(attentionLib) #macro added 13/11/2013
icubcontrib_finalize_export(attention)
icubcontrib_add_uninstall_target()        #macro added 13/11/2013
