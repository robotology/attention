# Copyright (C) 2020 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 3.12)

add_subdirectory( gazeArbiter  )
add_subdirectory( eeProfiler   )
add_subdirectory( attentionPrioritiser )

if(IPOPT_FOUND)
    message(STATUS "IPOPT not found/selected, skipping Modules")
endif(IPOPT_FOUND)
