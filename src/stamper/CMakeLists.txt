# Copyright: (C) 2013 RBCS Robotics Brain and Cognitive Science
# Authors: Rea Francesco
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
# YARP needs CMake 2.6 or greater
cmake_minimum_required(VERSION 2.6)
SET(KEYWORD "stamper")
PROJECT(${KEYWORD})

# find YARP
find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${ICUB_INCLUDE_DIRS})

# set up our program
add_executable(${KEYWORD} src/stamper.cpp)

# link with YARP libraries
target_link_libraries(${KEYWORD} ${YARP_LIBRARIES})
target_link_libraries(${KEYWORD} ${ICUB_LIBRARIES})

INSTALL_TARGETS(/bin ${KEYWORD})
