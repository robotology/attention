# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KEYWORD "saliencyBlobFinder")
PROJECT(${KEYWORD})

#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${IPP_INCLUDE_DIRS}
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
		logpolar
		icubmod
		iKin
    )
    
	INSTALL_TARGETS(/bin ${KEYWORD})

ELSE (folder_source)
    MESSAGE( "No source code files found. Please add something")
	
ENDIF (folder_source)


