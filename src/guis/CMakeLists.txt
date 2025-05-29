# Copyright (C) 2020 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 3.12)

find_package(GTK)


if(GTK_FOUND)
  add_subdirectory( selAttentionInterface     )
  message(STATUS "ENABLE_GTK  found/selected, building sel Attention Interface")
else(GTK_FOUND)
    message(STATUS "ENABLE_GTK not found/selected, skipping modules")
endif(GTK_FOUND)
