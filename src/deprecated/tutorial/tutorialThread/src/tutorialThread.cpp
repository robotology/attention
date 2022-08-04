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
    // initialization of the attributes
    width  = WIDTH;
    height = HEIGHT;
    idle   = false;

    inputImage = new ImageOf<PixelRgb>();
    inputImage->resize(width, height);

    // opening the port for direct input
    if (!inputPort.open(getName("/image:i").c_str())) {
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }  

    // opening the port for direct output
    if (!outputPort.open(getName("/result:o").c_str())) {
        yError("unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    //starting the plotterThread
    pt = new plotterThread();
    pt->setName(getName("").c_str());
    pt->start();

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void tutorialThread::setName(string str) {
    this->name=str;
}


std::string tutorialThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void tutorialThread::setInputPortName(string InpPort) {
    
}

void tutorialThread::visualizationResume(){
    pt->resume();
}

void tutorialThread::visualizationSuspend(){
    pt->suspend();
}

bool tutorialThread::test(){
    bool reply = true;

    if(pt==NULL){
        reply = false;
    }
    else {
        reply &= pt->test();
    }

    reply  &= Network::exists(getName("/image:i").c_str());

    return reply;
}

void tutorialThread::run() {    
    while (isStopping() != true) {
        int result;
        
        if(!idle){
            if(inputPort.getInputCount()) {
                checkImage.wait();
                inputImage = inputPort.read(true);   //blocking reading for synchr with the input
                result = processing();
                checkImage.post();
                
                //passing the image to the plotter
                checkImage.wait();
                pt->copyLeft(inputImage);
                checkImage.post();            
            }
            else {
                result = 0;
            }

            if (outputPort.getOutputCount()) {
                Bottle b = outputPort.prepare();
                b.addInt16(result);
                outputPort.write();  
            }
        
        }
    }               
}

int tutorialThread::processing(){
    return 1;
}

void tutorialThread::threadRelease() {
    // nothing    
}

void tutorialThread::onStop() {
    delete inputImage;
    if(pt!=NULL){
        yDebug("stopping the plotter thread");
        //printf("stopping the plotter thread \n");
        pt->stop();
    }
    yInfo("closing the ports");
    //printf("closing the ports \n");
    outputPort.interrupt();   
    outputPort.close();
}

