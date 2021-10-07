# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta, Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
SET(KEYWORD "gazeArbiter")
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 3.12)

if(MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
endif()


find_package(YARP REQUIRED)
find_package(YCM REQUIRED)
find_package(ICUB REQUIRED)
find_package(OpenCV REQUIRED)


list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
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
	        ${OpenCV_LIBRARIES}
	        iKin
	)	

    INSTALL_TARGETS(/bin ${KEYWORD})
	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
	
ENDIF (folder_source)
