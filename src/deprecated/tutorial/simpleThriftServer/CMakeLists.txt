#CMakeList.txt
cmake_minimum_required(VERSION 2.8.9)
#find YARP
find_package(YARP REQUIRED)
#compile definition file to generate source code into the desired directory
set(generated_libs_dir ".")
yarp_idl_to_dir(demo.thrift ${generated_libs_dir} sources headers include_dirs)
# generated source files now listed in ${sources}
# generated header files now listed in ${headers}
# paths to include now listed in ${include_dirs}

SET(KEYWORD "simpleThrift")
PROJECT(${KEYWORD})
cmake_minimum_required(VERSION 2.6)

set(ICUB_CONTRIB_DIRS $ENV{ICUB_DIR}/include)

INCLUDE_DIRECTORIES(
    ${PROJECT_SOURCE_DIR}/include
    ${YARP_INCLUDE_DIRS} 	
    ${ICUB_INCLUDE_DIRS}	
    ${ICUB_CONTRIB_DIRS}
    ${ATTENTIONLIB_INCLUDE_DIRS}
)

# Search for source code.
FILE(GLOB folder_source
        src/DemoServer.cpp
        src/Demo.cpp
  )
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
      )	

    INSTALL_TARGETS(/bin ${KEYWORD})
ELSE (folder_source)
    MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)
