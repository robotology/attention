#  Copyright: 2011 (C) Robotics, Brain and Cognitive Sciences - Italian Institute of Technology (IIT)
#  Authors: Ali Paikan <ali.paikan@iit.it>
# 
#  Copy Policy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
#

# Note set CUDA_BUILD_EMULATION to OFF to enable Device mode compilation.

cmake_minimum_required(VERSION 2.6.2)

set(PROJECTNAME cudaVision)
project(${PROJECTNAME})

#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/CMake/cuda" ${CMAKE_MODULE_PATH})
find_package(CUDA QUIET REQUIRED)


##############################################################################
# There are two ways to add include directories to the NVCC command
# line:

#set(CUDA_PROPAGATE_HOST_FLAGS OFF)

# The cuda_include_directories adds paths to only cuda compilation.
CUDA_INCLUDE_DIRECTORIES(
  ${CMAKE_CURRENT_SOURCE_DIR}
  )

SET(PROJECT_INCLUDE_DIRS
  ${PROJECT_SOURCE_DIR}/include)

SET(folder_source 
  src/convolution.cu
)

SET(folder_header 
  include/iCub/cudaVision/cudaVision.h 
)


INCLUDE_DIRECTORIES(
  ${CUDA_INCLUDE_DIRS}
  ${PROJECT_INCLUDE_DIRS}
  folder_header
  )

#link_directories(${CMAKEDEMO_BINARY_DIR}/w01-intro)
#set(CMAKE_CXX_FLAGS "-g -Wall")
#set(CUDA_NVCC_FLAGS "-arch=compute_13")

# Create everything needed to build our library
CUDA_ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header} STATIC)

# Generating PTX files.
# CUDA_COMPILE_PTX(PTX_FILES CUDA_FILES test_bin.cu)

# Add a special target to clean nvcc generated files.
CUDA_BUILD_CLEAN_TARGET()

