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


#ifndef _ATTENTION_Module_H_
#define _ATTENTION_Module_H_

#include <yarp/os/all.h>
#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Log.h>
#include <iCub/attention/commandDictionary.h>

//within project includes
#include <iCub/topDownAttentionPeriodic.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class topDownAttentionModule : public RFModule {

    string moduleName;
    string handlerPortName;
    Port handlerPort;

    topDownAttentionPeriodic *pThread;

public:

    bool configure(ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod();
    bool updateModule();

};

#endif
