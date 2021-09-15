# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ali Paikan
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


set(KEYWORD faceDetector)
project(${KEYWORD})
cmake_minimum_required(VERSION 3.12)

find_package(YARP REQUIRED)
find_package(OpenCV REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})


set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

INCLUDE_DIRECTORIES(
		${PROJECT_SOURCE_DIR}/include
		${OPENCV_INCLUDE_DIR}
)

# Search for source code.
FILE(GLOB folder_source *.cpp *.cc *.c)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up the main executable.
IF (folder_source)
	ADD_EXECUTABLE(${KEYWORD}
			${folder_source}
			${folder_header}
			)

	TARGET_LINK_LIBRARIES(${KEYWORD}
			${OpenCV_LIBS}
			${YARP_LIBRARIES}
			)

	INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
	MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)


