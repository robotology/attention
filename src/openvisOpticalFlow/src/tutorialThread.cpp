// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file tutorialThread.cpp
 * @brief Implementation of the eventDriven thread (see tutorialThread.h).
 */

#include <iCub/tutorialThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

tutorialThread::tutorialThread() {
    robot = "icub";        
}

tutorialThread::tutorialThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

tutorialThread::~tutorialThread() {
    // do nothing
}

bool tutorialThread::threadInit() {

    
   
    if (!outputPort.open(getName("/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
    

}

void tutorialThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string tutorialThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void tutorialThread::setInputPortName(string InpPort) {
    
}

void tutorialThread::run() {    
    while (isStopping() != true) {

        //code here .....
        

        if (outputPort.getOutputCount()) {
            outputPort.prepare() = *inputImage;
            outputPort.write();  
        }
    }               
}

void tutorialThread::threadRelease() {
    // nothing
     
}

void tutorialThread::onStop() {
    
    outputPort.interrupt();

    
    outputPort.close();
}

