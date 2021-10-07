# Copyright: (C) 2015 Robotics, Brain and Cognitive Science
# Authors: Francesco Rea, Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 3.12)
SET(KEYWORD "eeProfiler")
PROJECT(${KEYWORD})

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)
find_package(ICUB REQUIRED)
find_package(logpolar REQUIRED)
#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIR}
    ${logpolar_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
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
		    ${IPP_LIBRARIES}
            ${OpenCV_LIBRARIES}
		logpolar
		iKin
    )
    
  INSTALL_TARGETS(/bin ${KEYWORD})

ELSE (folder_source)
    MESSAGE( "No source code files found. Please add something")
	
ENDIF (folder_source)


