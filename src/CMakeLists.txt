# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta, Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

add_subdirectory( attentionPrioritiser      )
add_subdirectory( eyePathFinder             )
add_subdirectory( earlyMotion               )
add_subdirectory( earlyVision               )
add_subdirectory( earlyVisionInterface      )
add_subdirectory( iCubGuiPopulator          )
add_subdirectory( mosaic                    )
add_subdirectory( logOpticFlow              )
add_subdirectory( opticFlowInterface        )
add_subdirectory( repeater                  )
add_subdirectory( selectiveAttentionEngine  )
add_subdirectory( selAttentionInterface     )
add_subdirectory( salBlobFinderInterface    )
add_subdirectory( vergence                  )
add_subdirectory( visualFeatureExtractor    )
add_subdirectory( zdfControl                )
add_subdirectory( saliencyBlobFinder        )
add_subdirectory( stereoAttentiveSelector   )
#add_subdirectory(visualFilter)    
#add_subdirectory(zdfModule)


if(IPP_FOUND)
  add_subdirectory(yuvProcessor)
else(IPP_FOUND)
    message(STATUS "IPP not found/selected, skipping modules")
endif(IPP_FOUND)

if(IPOPT_FOUND)
    add_subdirectory(gazeArbiter)
else(IPOPT_FOUND)
    message(STATUS "IPOPT not found/selected, skipping gazeArbiter")
endif(IPOPT_FOUND)

