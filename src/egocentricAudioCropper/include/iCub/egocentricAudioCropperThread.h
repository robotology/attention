// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C)2020  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Omar Eldardeer
  * email: omar.eldardeer@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  * http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/
#ifndef ATTENTION_EGOCENTRIC_AUDIO_CROPPER_THREAD_H
#define ATTENTION_EGOCENTRIC_AUDIO_CROPPER_THREAD_H

#include <stdio.h>
#include <cmath>
#include <string>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Network.h>
#include <yarp/os/all.h>
using namespace yarp::os;
using namespace std;

typedef yarp::sig::Matrix yMatrix;
typedef yarp::sig::ImageOf<yarp::sig::PixelMono>  yImgPixelMono;
class egocentricAudioCropperThread : public PeriodicThread {
public:
    egocentricAudioCropperThread(string moduleName = "egocentricAudioCropper");
    ~egocentricAudioCropperThread();
    bool configure(yarp::os::ResourceFinder &rf);
    bool threadInit();
    void run();
    void threadRelease();
    string getName(const char* p) const;

private:
    Network yarp;
    RpcClient port;
    string moduleName;
    string inputPortName;
    string inputGazeAnglesPortName;
    string outputPortName;
    string outputImgPortName;


    //Parameters
    string cameraFileName;
    string cameraContextName;
    string cameraSide;
    double azimuthAngle;
    double cameraWidth;
    double cameraFocalLength;
    double cameraAOV;
    int cameraSideAOV;
    int azimuthIndex;

    yImgPixelMono* outputImg;
    Bottle* gazeAnglesBottle;

    BufferedPort<Bottle>  inputGazeAnglesPort;;
    BufferedPort<yMatrix>  inputPort;
    BufferedPort<yMatrix>  outputPort;
    BufferedPort<yImgPixelMono> outputImgPort;

};
#endif //ATTENTION_EGOCENTRIC_AUDIO_CROPPER_THREAD_H
