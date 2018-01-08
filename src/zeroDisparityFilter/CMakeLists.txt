# Copyright: (C) 2014 RBCS Robotics Brain and Cognitive Science
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
set (CMAKE_CXX_STANDARD 11)
SET(PROJECTNAME zeroDisparityFilter)
PROJECT(${PROJECTNAME})

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)

#set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} "-fpermissive")

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

FIND_PACKAGE(YARP REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)

LINK_LIBRARIES(${YARP_LIBRARIES})
INCLUDE_DIRECTORIES(${YARP_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
	${ICUB_INCLUDE_DIRS}
	${OPENCV_INCLUDE_DIR} 
)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header} )

TARGET_LINK_LIBRARIES(
    ${PROJECTNAME}
	${YARP_LIBRARIES}
    ${OpenCV_LIBS}
)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
