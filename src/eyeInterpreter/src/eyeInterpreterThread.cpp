// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
  * Copyright (C)2014  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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

/**
 * @file eyeInterpreterThread.cpp
 * @brief Implementation of the eventDriven thread (see eyeInterpreterThread.h).
 */

#include <iCub/eyeInterpreterThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

eyeInterpreterThread::eyeInterpreterThread():inputCbPort() {
    robot = "icub";        
}

eyeInterpreterThread::eyeInterpreterThread(string _robot, string _configFile):inputCbPort(){
    robot = _robot;
    configFile = _configFile;
}

eyeInterpreterThread::~eyeInterpreterThread() {
    // do nothing
}

bool eyeInterpreterThread::threadInit() {

    
    /* open ports */ 
    //inputCbPort.hasNewImage = false;
    //inputCbPort.useCallback();          // to enable the port listening to events via callback

    if (!inputCbPort.open(getName("/img:i").c_str())) {
        cout <<": unable to open port for reading events  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputPort.open(getName("/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
    

}

void eyeInterpreterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string eyeInterpreterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void eyeInterpreterThread::setInputPortName(string InpPort) {
    this->inputPortName = InpPort;
}

void eyeInterpreterThread::run() {    
    while (isStopping() != true) {
        if (inputCbPort.getInputCount()) {
            inputImage = inputCbPort.read(true);
		 if (outputPort.getOutputCount()) {
            		outputPort.prepare() = *inputImage;
            		outputPort.write();
		 }  
        }
    }               
}

void eyeInterpreterThread::threadRelease() {
    // nothing
     
}

void eyeInterpreterThread::onStop() {
    //inputCallbackPort.interrupt();
    outputPort.interrupt();

    //inputCallbackPort.close();
    outputPort.close();
}

