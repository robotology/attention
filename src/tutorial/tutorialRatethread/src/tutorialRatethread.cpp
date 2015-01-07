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
 * @file tutorialRatethread.cpp
 * @brief Implementation of the eventDriven thread (see tutorialRatethread.h).
 */

#include <iCub/tutorialRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100 //ms

tutorialRatethread::tutorialRatethread():RateThread(THRATE) {
    robot = "icub";        
}

tutorialRatethread::tutorialRatethread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

tutorialRatethread::~tutorialRatethread() {
    // do nothing
}

bool tutorialRatethread::threadInit() {
    // opening the port for direct input
    if (!inputPort.open(getName("/image:i").c_str())) {
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!outputPort.open(getName("/img:o").c_str())) {
        yError(": unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void tutorialRatethread::setName(string str) {
    this->name=str;
}


std::string tutorialRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void tutorialRatethread::setInputPortName(string InpPort) {
    
}

void tutorialRatethread::run() {    
    //code here .....
    if (inputPort.getInputCount()) {
        inputImage = inputPort.read(true);   //blocking reading for synchr with the input
        result = processing();
    }

    if (outputPort.getOutputCount()) {
        *outputImage = outputPort.prepare();
        outputImage->resize(inputImage->width(), inputImage->height());
        // changing the pointer of the prepared area for the outputPort.write()
        outputPort.prepare() = *inputImage;

        outputPort.write();
    }

}

bool tutorialRatethread::processing(){
    // here goes the processing...
    return true;
}


void tutorialRatethread::threadRelease() {
    // nothing
    inputPort.interrupt();
    outputPort.interrupt();
    inputPort.close();
    outputPort.close();
}


