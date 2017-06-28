#!/bin/bash

#Connect the IcubSim eye to the zeroDisparityMod port
yarp connect /icubSim/cam/left /zeroDisparityFilterMod/imageLeft:i
yarp connect /icubSim/cam/right /zeroDisparityFilterMod/imageRight:i

#Create the yarpView for zeroDisparityView
yarpview --name /imageProb  > /dev/null 2>&1 & 
yarpview --name /imageSeg  > /dev/null 2>&1 &
yarpview --name /imageDog  > /dev/null 2>&1 &
yarpview --name /imageTemp  > /dev/null 2>&1 &

#create the yarpview fro IcubSim Camera
yarpview --name /leftCamera  > /dev/null 2>&1 & 
yarpview --name /RightCamera  > /dev/null 2>&1 &

sleep 10

#Connect to the zeroDisparityFilterMod port
yarp connect /zeroDisparityFilterMod/imageProb:o /imageProb
yarp connect /zeroDisparityFilterMod/imageSeg:o /imageSeg
yarp connect /zeroDisparityFilterMod/imageDog:o /imageDog
yarp connect /zeroDisparityFilterMod/imageTemp:o /imageTemp

#Connect zdfControl to zeroDisparityFilter
yarp connect /zdfControl/command:o /zeroDisparityFilterMod

#Connect iCubSim camera to yarpView
yarp connect /icubSim/cam/left /leftCamera
yarp connect /icubSim/cam/right /RightCamera
