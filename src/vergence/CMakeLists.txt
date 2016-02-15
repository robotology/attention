# Copyright: (C) 2010 RobotCub Consortium
# Authors: Lorenzo Natale
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME vergence)
project(${PROJECTNAME})

if(MSVC)
   add_definitions(-D_USE_MATH_DEFINES)
endif()

find_package(YARP)
find_package(ICUB)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubOptions)
include(iCubHelpers)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")
set(ICUB_APPLICATIONS_PREFIX "$ENV{ICUB_ROOT}" CACHE PATH "Application path prefix")

file(GLOB folder_source src/*.cpp src/*.cc src/*.c)
file(GLOB folder_header include/iCub/*.h)

source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${YARP_INCLUDE_DIRS}
                    ${iKin_INCLUDE_DIRS}
                    ${logpolar_INCLUDE_DIRS})

add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
target_link_libraries(${PROJECTNAME} 
  iKin 
  logpolar 
  #icubmod 
  ${YARP_LIBRARIES}
)
install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_DIR}/bin)

