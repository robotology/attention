# Copyright: (C) 2014 Robotics Brain and Cognitive Science
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

if(OpenCV_FOUND)
    add_subdirectory( opfExtractor  )
else(OpenCV_FOUND)
    message(STATUS "OpenCV not found/selected in attention/biologicalProcessing, skipping modules")
endif(OpenCV_FOUND)