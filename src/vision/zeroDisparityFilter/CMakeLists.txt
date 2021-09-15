# Copyright: (C) 2021 RBCS Robotics Brain and Cognitive Science
# Authors: Francesco Rea, Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(KEYWORD zeroDisparityFilter)
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 3.12)
#
find_package(YCM REQUIRED)
find_package(YARP REQUIRED COMPONENTS sig cv)
find_package(OpenCV REQUIRED)

set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

INCLUDE_DIRECTORIES(
		${PROJECT_SOURCE_DIR}/include
		${ICUB_CONTRIB_DIRS}
		${OPENCV_INCLUDE_DIR}
)


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
			PRIVATE
			${OpenCV_LIBS}
			${YARP_LIBRARIES}
			)

	INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
	MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
