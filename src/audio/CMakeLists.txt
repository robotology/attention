# Copyright: (C) 2014 RobotCub Consortium
# Authors: Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

add_subdirectory  ( audioAttention           )
add_subdirectory  ( audioInterface           )
add_subdirectory  ( audioTemplate            )

add_subdirectory  ( headingAudio             )
add_subdirectory  ( remoteInterface          )
add_subdirectory  ( mexInterface              )
add_subdirectory  ( audioFrameExtractor       )

if(OpenCV_FOUND)
    add_subdirectory  ( cartesianVisualGenerator )
else(OpenCV_FOUND)
    message(STATUS "OpenCV not found/selected in attention/audio, skipping modules")
endif(OpenCV_FOUND)