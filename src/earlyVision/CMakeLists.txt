# Copyright: (C) 2010 RobotCub Consortium
# Authors: Rea Francesco, Shashank
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KEYWORD "earlyVision")
PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)
FIND_PACKAGE(OpenCV REQUIRED)


INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${logpolar_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS} 	
    ${OPENCV_INCLUDE_DIR}    
    	
)

# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/*.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

#SET(OPENCV_LIBRARIES "/home/rea/work/OpenCV-2.0.0/lib")
MESSAGE(STATUS "openCV inc" ${OpenCV_LIBS})
# Set up the main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )

    MESSAGE("OpenCV_LIBS ${OpenCV_LIBS}" )

    if (ENABLE_CUDA)  
        TARGET_LINK_LIBRARIES(${KEYWORD} logpolar cudaVision ${OpenCV_LIBS} ${YARP_LIBRARIES} )	
    else (ENABLE_CUDA)
        TARGET_LINK_LIBRARIES(${KEYWORD} logpolar ${OpenCV_LIBS} ${YARP_LIBRARIES} )	
    endif (ENABLE_CUDA)


    INSTALL_TARGETS(/bin ${KEYWORD})
	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
	
ENDIF (folder_source)
