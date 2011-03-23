# Copyright: (C) 2010 RobotCub Consortium
# Authors: Giorgio Metta, Francesco Rea
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

add_subdirectory(colourSaliency)
add_subdirectory(eyePathFinder)
add_subdirectory(iCubGuiPopulator)
add_subdirectory(mosaic)
add_subdirectory(selectiveAttentionEngine)
add_subdirectory(selAttentionInterface)
add_subdirectory(salBlobFinderInterface)


if(IPP_FOUND)
    if(IPOPT_FOUND)
      add_subdirectory(saliencyBlobFinder)
    endif(IPOPT_FOUND)  
    add_subdirectory(stereoAttentiveSelector)
    add_subdirectory(visualFilter)
    add_subdirectory(yuvProcessor)
    add_subdirectory(zdfModule)
else(IPP_FOUND)
    message(STATUS "IPP not found/selected, skipping saliencyBlobFinder")
    message(STATUS "IPP not found/selected, skipping stereoAttentiveSelector")
    message(STATUS "IPP not found/selected, skipping visualFilter")
    message(STATUS "IPP not found/selected, skipping yuvProcessor")
    message(STATUS "IPP not found/selected, skipping zdfModule")
endif(IPP_FOUND)

if(IPOPT_FOUND)
    add_subdirectory(gazeArbiter)
else(IPOPT_FOUND)
    message(STATUS "IPOPT not found/selected, skipping gazeArbiter")
endif(IPOPT_FOUND)

