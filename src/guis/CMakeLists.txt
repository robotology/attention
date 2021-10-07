# Copyright (C) 2020 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Omar Eldardeer
# CopyPolicy: Released under the terms of the GNU GPL v2.0.


cmake_minimum_required(VERSION 3.12)

find_package(GTK2)


if(GTK2_FOUND)
  add_subdirectory( selAttentionInterface     )
else(GTK2_FOUND)
    message(STATUS "ENABLE_GTK2 not found/selected, skipping modules")
endif(GTK2_FOUND)
