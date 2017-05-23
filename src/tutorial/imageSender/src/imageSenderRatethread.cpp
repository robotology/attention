// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2017  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Fabio Vannucci
  * email: fabio.vannucci@iit.it
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
 * @file imageSender.cpp
 * @brief Implementation of a module that sends continuously an image in an output port
 */

#include <iCub/imageSenderRatethread.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;
using namespace cv;


#define THRATE 100 //ms

imageSenderRatethread::imageSenderRatethread():RateThread(THRATE) {
    robot = "icub";        
}

imageSenderRatethread::imageSenderRatethread(string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

imageSenderRatethread::~imageSenderRatethread() {
    // do nothing
}

bool imageSenderRatethread::threadInit() {
    // opening the port for direct input

    if (!outputPort.open(getName("/img:o").c_str())) {
        yError(": unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    outputImage = new ImageOf<PixelRgb>;

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void imageSenderRatethread::setName(string str) {
    this->name=str;
}


std::string imageSenderRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}


void imageSenderRatethread::run() {    
    
    if (outputPort.getOutputCount()) {
        
        IplImage* Ipl = cvLoadImage("/usr/local/src/robot/attention/src/tutorial/imageSender/grossa.jpg", 1); 
        //yDebug("eccola");
        //cvShowImage("prova", Ipl);
        //cvWaitKey();
        outputImage = &outputPort.prepare();
        outputImage->resize(600,600);
        outputImage->wrapIplImage(Ipl);
        outputPort.write();
    }

}

bool imageSenderRatethread::processing(){

    return true;
}


void imageSenderRatethread::threadRelease() {

    outputPort.interrupt();
    outputPort.close();
}


