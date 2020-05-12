# Copyright: (C) 2012 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KEYWORD "attentionPrioritiser")
PROJECT(${KEYWORD})

message ("logpolar_INCLUDE_DIRS : ${logpolar_INCLUDE_DIRS};")
message ("YARP_INCLUDE_DIRS ${YARP_INCLUDE_DIRS} ")
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS}
    ${ICUB_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${iKin_INCLUDE_DIRS}
    ${ATTENTIONLIB_INCLUDE_DIRS}
    ${logpolar_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source 
	src/attPrioritiserModule.cpp
	src/attPrioritiserThread.cpp
	src/main.cpp
	src/prioCollectorThread.cpp
	src/sacPlannerThread.cpp
    src/oculomotorController.cpp
    src/trajectoryPredictor.cpp
)

FILE(GLOB folder_header 
	include/iCub/attPrioritiserModule.h
	include/iCub/attPrioritiserThread.h
	include/iCub/observable.h
	include/iCub/observer.h
	include/iCub/prioCollectorThread.h
	include/iCub/sacPlannerThread.h
	include/iCub/trackerThread.h
    include/iCub/oculomotorController.h
    include/iCub/trajectoryPredictor.h
)

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
	        #icubmod 
            attentionLib
            logpolar
	)	

    INSTALL_TARGETS(/bin ${KEYWORD})
	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
	
ENDIF (folder_source)
