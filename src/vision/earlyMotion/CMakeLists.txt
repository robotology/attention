# Copyright: (C) 2011 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KEYWORD "earlyMotion")
PROJECT(${KEYWORD})

#FIND_PACKAGE(ICUB REQUIRED)
#FIND_PACKAGE(YARP REQUIRED)
#SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)
#FIND_PACKAGE(OpenCV REQUIRED)


find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)
find_package(ICUBcontrib REQUIRED)
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH}
                              ${ICUBCONTRIB_MODULE_PATH})
find_package(logpolar QUIET)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${logpolar_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS} 	
    ${IPP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIR}	
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

  #TARGET_LINK_LIBRARIES(${KEYWORD} logpolar ${IPP_LIBRARIES} ${OpenCV_LIBS} ${YARP_LIBRARIES})
  target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} logpolar ${YARP_LIBRARIES})
  install(TARGETS ${KEYWORD} DESTINATION bin)
    #INSTALL_TARGETS(/bin ${KEYWORD})
	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
	
ENDIF (folder_source)
