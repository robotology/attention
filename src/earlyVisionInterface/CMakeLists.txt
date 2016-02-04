# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta, Shashank Pathak
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(KEYWORD "earlyVisionInterface")
PROJECT(${KEYWORD})

FIND_PACKAGE(ICUB)
FIND_PACKAGE(YARP)
SET (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_DIR}/conf ${YARP_DIR}/conf)
FIND_PACKAGE(OpenCV REQUIRED)

# set up to use GTK+
#LINK_LIBRARIES(${GtkPlus_LIBRARIES})
#LINK_DIRECTORIES(${GTKPLUS_LIBRARY_DIR})
#INCLUDE_DIRECTORIES(${GtkPlus_INCLUDE_DIRS})

#search for includes
INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${ICUB_INCLUDE_DIRS}
    ${YARP_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}    	
    ${GTK2_INCLUDE_DIRS}
)

# libsigc++ 2.5.1 or later requires c++11. This is automatically
# enabled with CMake 3.1 or later (using FindGTK2.cmake from CMake
# 3.5). With older versions, we enable it here.
if(NOT ${CMAKE_MINIMUM_REQUIRED_VERSION} VERSION_LESS 3.1)
  message(AUTHOR_WARNING "CMAKE_MINIMUM_REQUIRED_VERSION is now ${CMAKE_MINIMUM_REQUIRED_VERSION}. This check can be removed.")
endif()
if(CMAKE_VERSION VERSION_LESS 3.1)
  if(NOT ${GTK2_SIGC++_VERSION_MAJOR}.${GTK2_SIGC++_VERSION_MINOR}.${GTK2_SIGC++_VERSION_MICRO} VERSION_LESS 2.5.1)
    add_definitions(${CXX11_FLAGS})
  endif()
endif()


# Search for source code.
FILE(GLOB folder_source src/*.cpp src/*.cc src/.c)
FILE(GLOB folder_header include/iCub/*.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

# Set up our main executable.
IF (folder_source)
    ADD_EXECUTABLE(${KEYWORD} 
        ${folder_source} 
        ${folder_header}
    )
    TARGET_LINK_LIBRARIES( ${KEYWORD}
      ${YARP_LIBRARIES}
      #${GTKPLUS_LINK_FLAGS}
      ${GTK2_LIBRARIES}
    )	
    INSTALL_TARGETS(/bin ${KEYWORD})
	
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
	
ENDIF (folder_source)


