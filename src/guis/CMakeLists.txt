# Copyright (C) 2020 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 2.6)

if(ENABLE_GTK2)
  add_subdirectory( selAttentionInterface     )
else(ENABLE_GTK2)
    message(STATUS "ENABLE_GTK2 not found/selected, skipping modules")
endif(ENABLE_GTK2)
