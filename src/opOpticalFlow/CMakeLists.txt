# Copyright (C) 2013 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

SET(KEYWORD "openvisOpticalFlow")
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 2.6)

set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)


INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS} 	
    ${ICUB_INCLUDE_DIRS}	
    ${ICUB_CONTRIB_DIRS}
    ${OPENCV_INCLUDE_DIR} 
)

# Search for source code.
FILE(GLOB folder_source  
  main.cpp
  opticalFlowModule.cpp
  opticalFlowThread.cpp
  OpenCVImageAdapter.cpp
  OvImageAdapter.cpp
  OvImageT.cpp
  OvStereoDiffuseMatcherT.cpp
  )
FILE(GLOB folder_header  
  opticalFlowModule.h
  opticalFlowThread.h
  OpenCVImageAdapter.h 
  OvImageAdapter.h
  Openvis3d.h
  OvStereoGlobalMatcherT.h
  OvDisparityPostprocessor.h
  OvImageT.h
  OvImagePairPreprocessorT.h
  OvLocalMatcherT.h
  OvFlowGlobalMatcherT.h
  OvFlowPostprocessor.h
  OvStereoT.h
  OvFlowT.h
  BTLocalMatcherT.h
  OvStereoDiffuseMatcherT.h
  OvFlowDiffuseMatcherT.h
  )
MESSAGE("folder_header in openvis3d ${folder_header}")

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
