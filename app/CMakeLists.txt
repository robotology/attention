# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

#cmake_minimum_required(VERSION 2.6)

#project(boltzmannMachineLibrary)

# Check for the minimal set of packages required for the global project.
# If we include them here, sub-projects can be configured a bit faster.
# Only a few demo projects are included so far.

#FIND_PACKAGE(ICUB) 
#SET(CMAKE_MODULE_PATH ${ICUB_MODULE_PATH} ${CMAKE_MODULE_PATH})

#include(iCubHelpers)

# default prefix for applications
#SET(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}"  CACHE PATH "Application path prefix")
				
add_subdirectory ( biologicalProcessing )
add_subdirectory ( logpolarAttention    )
#add_subdirectory(protoObjectVisualAttention)
#add_subdirectory(dumpApplication)

## make all applications show up as single target
#icub_app_all()



