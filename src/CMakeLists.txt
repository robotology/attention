# Copyright: (C) 2010 RobotCub Consortium
# Authors: Francesco Rea, Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)

<<<<<<< HEAD
#add_subdirectory( audio                     )
#add_subdirectory( audioAttention           )
#add_subdirectory( audioInterface           )
=======
add_subdirectory( attentionPrioritiser      )
add_subdirectory( audio                     )
>>>>>>> d6da2a9e3480c83dcfd4d927b7ba9dc06eea5f6c
add_subdirectory( eyePathFinder             )
add_subdirectory( earlyMotion               )
add_subdirectory( earlyVision               )
add_subdirectory( earlyVisionInterface      )
add_subdirectory( ikart                     )
add_subdirectory( faceDetector              )
add_subdirectory( iCubGuiPopulator          )
#add_subdirectory( iKartFollower             )
add_subdirectory( eyeInterpreter            )
add_subdirectory( mosaic                    )
add_subdirectory( logOpticFlow              )
add_subdirectory( opticFlowInterface        )
#add_subdirectory( opOpticalFlow            )
#add_subdirectory( repeater                  )
add_subdirectory( selectiveAttentionEngine  )
add_subdirectory( selAttentionInterface     )
add_subdirectory( salBlobFinderInterface    )
add_subdirectory( vergence                  )
#add_subdirectory( visualFeatureExtractor   )
add_subdirectory( zdfControl                )
add_subdirectory( stereoAttentiveSelector   )
add_subdirectory( stamper                   )
add_subdirectory( test                      )
add_subdirectory( tutorial                  )
#add_subdirectory( visualFilter             )
add_subdirectory(workingMemory              )   
#add_subdirectory( zeroDisparityFilter      )
#add_subdirectory( moveBall     )
add_subdirectory( farnebackTest)
add_subdirectory( batchAnalysis)
add_subdirectory( batchAnalysis2 )

if(IPP_FOUND)
  add_subdirectory(yuvProcessor)
else(IPP_FOUND)
    message(STATUS "IPP not found/selected, skipping modules")
endif(IPP_FOUND)

if(IPOPT_FOUND)
    add_subdirectory( gazeArbiter               )
    add_subdirectory( attentionPrioritiser      )
    add_subdirectory( saliencyBlobFinder        )
<<<<<<< HEAD
    add_subdirectory( kalmanTest                )
    add_subdirectory( predictorTest             )
=======
    #add_subdirectory( kalmanTest                )
    #add_subdirectory( predictorTest             )
>>>>>>> d6da2a9e3480c83dcfd4d927b7ba9dc06eea5f6c
else(IPOPT_FOUND)
    message(STATUS "IPOPT not found/selected, skipping gazeArbiter")
endif(IPOPT_FOUND)


