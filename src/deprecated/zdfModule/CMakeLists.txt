# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME zdfMod)
PROJECT(${PROJECTNAME})

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

FIND_PACKAGE(YARP REQUIRED)
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
	${ICUB_INCLUDE_DIRS}
	/opt/intel/ipp/include
)

ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header} )

TARGET_LINK_LIBRARIES(
    ${PROJECTNAME}

	${YARP_LIBRARIES}
)

INSTALL(TARGETS ${PROJECTNAME} DESTINATION bin)
