# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea, Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

if(logpolar_FOUND)
  
endif(logpolar_FOUND)


if(OpenCV_FOUND)
  #add_subdirectory( eyePathFinder             )
  add_subdirectory( earlyMotion               )
  add_subdirectory( earlyVision               )
  add_subdirectory( ColorSegmentation         )  
  add_subdirectory( ikart                     )
  add_subdirectory( faceDetector              )
  add_subdirectory( faceTracker               )
  add_subdirectory( iCubGuiPopulator          )
  #add_subdirectory( iKartFollower             )
  add_subdirectory( eyeInterpreter            )
  #add_subdirectory( mosaic                    )
  add_subdirectory( logOpticFlow              )
  #add_subdirectory( opticFlowInterface        )
  #add_subdirectory( opOpticalFlow             )
  add_subdirectory( repeater                  )
  add_subdirectory( selectiveAttentionEngine  )
  add_subdirectory( vergence                  )
  add_subdirectory( skinProcessor             )
  add_subdirectory( stereoAttentiveSelector   )
  add_subdirectory( stamper                   )
  #add_subdirectory( test                      )
  #add_subdirectory( tutorial                  )
  #add_subdirectory( visualFilter              )
  #add_subdirectory(workingMemory              )   
  add_subdirectory( zeroDisparityFilter       )
  #add_subdirectory( zdfMod                    )
  #add_subdirectory( selAttentionInterface     )
  add_subdirectory( topDownAttention          )
else(OpenCV_FOUND)
  message(STATUS "OpenCV not found/selected, skipping modules")
endif(OpenCV_FOUND)

#graphic user interfaces
# Tools available both for Qt5 and GTK2
if(ICUB_USE_Qt5)
  #add_subdirectory(guis/selAttentionGui)
endif()

if(ENABLE_GTK2)
  add_subdirectory( earlyVisionInterface      )
  add_subdirectory( salBlobFinderInterface    )
  add_subdirectory( selAttentionInterface     )
  add_subdirectory( zdfControl                )
endif()

if(NOT ICUB_USE_Qt5 AND NOT ICUB_USE_GTK2)
   message(STATUS "Qt5 and GTK2 not found/selected, skipping iCubSkinGui")
   message(STATUS "Qt5 and GTK2 not found/selected, skipping frameGrabberGui2")
   message(STATUS "Qt5 and GTK2 not found/selected, skipping skinManagerGui")
endif()

if(IPOPT_FOUND)
    add_subdirectory( gazeArbiter               )
    add_subdirectory( attentionPrioritiser      )
    add_subdirectory( saliencyBlobFinder        )
    add_subdirectory( handProfiler              )
    #add_subdirectory( kalmanTest               )
    #add_subdirectory( predictorTest            )
else(IPOPT_FOUND)
    message(STATUS "IPOPT not found/selected, skipping gazeArbiter")
endif(IPOPT_FOUND)


# subdirectories
add_subdirectory( biologicalProcessing      )
#add_subdirectory( audio                     )


