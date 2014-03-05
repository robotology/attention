// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2014  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file audioInterfaceRatethread.cpp
 * @brief Implementation of the eventDriven thread (see audioInterfaceRatethread.h).
 */

#include <iCub/audioInterfaceRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100 //ms

audioInterfaceRatethread::audioInterfaceRatethread() {
    robot = "icub";        
}

audioInterfaceRatethread::audioInterfaceRatethread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

audioInterfaceRatethread::~audioInterfaceRatethread() {
    // do nothing
}

bool audioInterfaceRatethread::threadInit() {
    if (!inputPort.open(getName("/audio:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
   
    if (!outputPort.open(getName("/audio:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
    

}

void audioInterfaceRatethread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string audioInterfaceRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void audioInterfaceRatethread::setInputPortName(string InpPort) {
    
}

void audioInterfaceRatethread::run() {    
    while(!isStopping()){

        //code here ....
        if(inputPort.getInputCount()){
            Bottle* read = inputPort.read();
            printf(" bottle received %s \n", read->toString().c_str());
        }
        /*
        if (outputPort.getOutputCount()) {
            outputPort.prepare() = *inputImage;
            outputPort.write();  
        }
        */
    }
                  
}

void tutorialThread::onStop() {
    printf("closing the ports \n");
    outputPort.interrupt();   
    inputPort.interrupt();
    outputPort.close();
    inputPort.close();
}

void audioInterfaceRatethread::threadRelease() {
    inputPort.close();
    outputPort.close();
}


