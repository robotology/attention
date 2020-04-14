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

#ifndef ATTENTION_ATTENTIONMANAGERMODULE_H
#define ATTENTION_ATTENTIONMANAGERMODULE_H


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
#include <iCub/attentionManagerThread.h>
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

class attentionManagerModule : public RFModule {

    string moduleName;
    string handlerPortName;
    Port handlerPort;

    attentionManagerThread *pThread;

public:

    bool configure(ResourceFinder &rf);
    bool interruptModule();
    bool close();
    bool respond(const Bottle& command, Bottle& reply);
    double getPeriod();
    bool updateModule();

};


#endif //ATTENTION_ATTENTIONMANAGERMODULE_H
