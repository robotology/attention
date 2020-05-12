# Copyright (C) 2020 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 2.6)



if(OpenCV_FOUND)
  add_subdirectory( ColorSegmentation         )
  add_subdirectory( earlyMotion               )
  add_subdirectory( earlyVision               )
  add_subdirectory( faceDetector              )
  add_subdirectory( faceTracker               )
  add_subdirectory( repeater                  )
  add_subdirectory( selectiveAttentionEngine  )
  add_subdirectory( zeroDisparityFilter       )
else(OpenCV_FOUND)
  message(STATUS "OpenCV not found/selected, skipping modules")
endif(OpenCV_FOUND)


if(IPOPT_FOUND)
    add_subdirectory( saliencyBlobFinder )
else(IPOPT_FOUND)
    message(STATUS "IPOPT not found/selected, skipping saliencyBlobFinder ")
endif(IPOPT_FOUND)
