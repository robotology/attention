# Copyright (C) 2020 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 3.12)
find_package(YARP REQUIRED COMPONENTS sig cv)

find_package(OpenCV REQUIRED PATHS /usr/local/src/robot/supportedLibs/opencv-4.5.3/lib/cmake/opencv4/ NO_DEFAULT_PATH)

if(OpenCV_FOUND)
  add_subdirectory( ColorSegmentation         )
  add_subdirectory( earlyMotion               )
  add_subdirectory( earlyVision               )
  add_subdirectory( faceDetector              )
  add_subdirectory( faceTracker               )
  add_subdirectory( repeater                  )
  add_subdirectory( selectiveAttentionEngine  )
  add_subdirectory( zeroDisparityFilter       )
  add_subdirectory( saliencyBlobFinder )
else(OpenCV_FOUND)
  message(STATUS "OpenCV not found/selected, skipping modules")
endif(OpenCV_FOUND)


# if(IPOPT_FOUND)
#    add_subdirectory( saliencyBlobFinder )
#else(IPOPT_FOUND)
#    message(STATUS "IPOPT not found/selected, skipping saliencyBlobFinder ")
#endif(IPOPT_FOUND)
