# Copyright (C) 2020 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 2.6)

add_subdirectory( allocentricAudioPriorAdder)

if(OpenCV_FOUND)
  add_subdirectory( egocentricAudioCropper  )
  
else(OpenCV_FOUND)
  message(STATUS "OpenCV not found/selected, skipping modules")
endif(OpenCV_FOUND)
