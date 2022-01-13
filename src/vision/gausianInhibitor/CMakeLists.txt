# Copyright (C) 2021 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(KEYWORD "gausianInhibitor")
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 3.12)

find_package(ICUBcontrib REQUIRED)
find_package(YARP REQUIRED os dev sig cv math)
find_package(YCM REQUIRED )
set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)





INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/include
        ${ICUB_INCLUDE_DIRS}
        ${ICUB_CONTRIB_DIRS}
        ${attentionLib_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})




# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD}
            ${folder_source}
            ${folder_header}
            )

    TARGET_LINK_LIBRARIES(${KEYWORD}
            ${YARP_LIBRARIES}
            )

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)