# Copyright: 2012 Robotics, Brain and Cognitive Sciences Dept. Istituto Italiano di Teconologia
# Authors: Dalia De Santis, Jacopo Zenzeri, Tomas Werner
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


SET(KEYWORD ColorSegmentation)
project(${KEYWORD})
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 3.12)


find_package(YARP REQUIRED)
find_package(YCM REQUIRED)
find_package(ICUBcontrib REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})
set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

icubcontrib_set_default_prefix()
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

#search for includes
INCLUDE_DIRECTORIES(
        ${PROJECT_SOURCE_DIR}/include
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c )
FILE(GLOB folder_header include/iCub/darwin/*.h )
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up our main executable
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD}
            ${folder_source}
            ${folder_header}
            )

    TARGET_LINK_LIBRARIES( ${KEYWORD}
            ${YARP_LIBRARIES}
            )

    install(TARGETS  ${KEYWORD} DESTINATION bin)


ELSE (folder_source)
    MESSAGE( "No source code files found. Please add something")
ENDIF (folder_source)