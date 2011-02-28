# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(colourSaliency)
add_subdirectory(eyePathFinder)
add_subdirectory(iCubGuiPopulator)
add_subdirectory(selectiveAttentionEngine)
add_subdirectory(selAttentionInterface)
add_subdirectory(saliencyBlobFinder)
add_subdirectory(salBlobFinderInterface)
add_subdirectory(stereoAttentiveSelector)
add_subdirectory(visualFilter)
add_subdirectory(yuvProcessor)
add_subdirectory(zdfModule)

if(IPOPT_FOUND)
    add_subdirectory(gazeArbiter)
else(IPOPT_FOUND)
    message(STATUS "IPOPT not found/selected, skipping gazeArbiter")
endif(IPOPT_FOUND)

# some garbage?
#add_subdirectory(indepLogMotion)
#add_subdirectory(zdfControl)
