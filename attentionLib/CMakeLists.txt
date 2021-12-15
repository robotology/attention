# Copyright: (C) 2012 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 3.12)

SET(PROJECTNAME attentionLib)
PROJECT(${PROJECTNAME})

find_package(YARP REQUIRED)
find_package(YCM REQUIRED)

SET(PROJECT_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include)

# Find .cpp and .h files automatically.  This is a bit lazy,
# and in principle it would be better to list these files manually.
SET(folder_source 
  ./src/predModels.cpp
)

SET(folder_header 
  ./include/iCub/attention/predModels.h
  ./include/iCub/attention/evalThread.h
  ./include/iCub/attention/commandDictionary.h
)

MESSAGE("folder_header ${folder_header}")

# Declare groups of source and header files -- makes things pretty in MSVC.
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

message ("PROJECT_SOURCE_DIR ${PROJECT_SOURCE_DIR}")
# Add our include files into our compiler's search path.
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include
        ${ICUB_INCLUDE_DIRS}
)

# Create everything needed to build our library
ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})

# Makes VS a bit quiter wrt warnings.
IF(MSVC)
    SET_TARGET_PROPERTIES(${PROJECTNAME} PROPERTIES COMPILE_FLAGS /wd4996)
ENDIF(MSVC)

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES})


icubcontrib_export_library(${PROJECTNAME} INTERNAL_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include
                                          EXTERNAL_INCLUDE_DIRS ${ICUB_INCLUDE_DIRS}
                                          DESTINATION include/iCub/attention
                                          FILES  ${folder_header})
