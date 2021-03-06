// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C)2019  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#ifndef _ATTENTION_PERIODIC_H_
#define _ATTENTION_PERIODIC_H_

#include <stdio.h>
#include <string>
#include <array>
#include <array>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Network.h>
#include <yarp/os/all.h>
#include "iCub/state.h"
using namespace yarp::os;
using namespace std;




class topDownAttentionPeriodic : public PeriodicThread {
public:
    topDownAttentionPeriodic(double p,string moduleName);
    bool threadInit();
    void afterStart(bool s);
    void run();
    void threadRelease();

    void setRandomMode();
    void setBlobMode();
    void setIntensityMode();
    void setChrominanceMode();
    void setEdgesMode();
    void setMotionMode();
    void setOrientationMode();
    void setFaceMode();
    void setBioMotioMode();
    void setAudioMode();

private:
    Network yarp;
    RpcClient port;
    int msgCount;
    string moduleName;
    string clientName;
    array<state,9> attentionStates;
    bool randomMode;


    void sendAttentionToPort(ATTENTION_MODES mode);

};

#endif
