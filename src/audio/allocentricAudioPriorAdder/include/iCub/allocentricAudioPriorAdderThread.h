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
#ifndef ATTENTIONN_AUDIO_PRIOR_THREAD_H
#define ATTENTIONN_AUDIO_PRIOR_THREAD_H

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
#include <vector>
using namespace yarp::os;
using namespace std;

typedef yarp::sig::Matrix yMatrix;
typedef yarp::os::BufferedPort< yMatrix > yMatrixBuffer;

class allocentricAudioPriorAdderThread : public PeriodicThread {
public:
    allocentricAudioPriorAdderThread(string moduleName = "allocentricAudioPriorAdder");
    ~allocentricAudioPriorAdderThread();
    bool configure(yarp::os::ResourceFinder &rf);
    bool threadInit();
    void run();
    void threadRelease();
    string getName(const char* p) const;

private:
    Network yarp;
    RpcClient port;
    string moduleName;

    //Parameters
    double rawPowerThreshold;

    Bottle priorAngles;
    int sideWindowWidth;

    int priorAnglesCount;
    vector<int> priorAnglesIdxList;

    //Port Name
    string inputProbabilityAngleMapPortName ;
    string inputRawPowerPortName            ;
    string outputAngleMapPortName           ;


    yMatrix probabilityAngleMapMatrix;
    yMatrix rawPowerMatrix;
    yMatrix angleMapMatrix;

    double rawPowerTotal;

    yMatrixBuffer  inputProbabilityAngleMapPort;
    yMatrixBuffer  inputRawPowerPort;
    yMatrixBuffer  outputAngleMapPort;

    void publishOutPorts();
};
#endif //ATTENTIONN_AUDIO_PRIOR_THREAD_H
