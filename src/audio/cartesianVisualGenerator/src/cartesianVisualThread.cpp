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
 * @file cartesianVisualThread.cpp
 * @brief Implementation of the eventDriven thread (see cartesianVisualThread.h).
 */

#include <iCub/cartesianVisualThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

double cartesianVisualThread::gaussianWeights[10] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0}; 

cartesianVisualThread::cartesianVisualThread() {
    robot = "icub";   
}

cartesianVisualThread::cartesianVisualThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

cartesianVisualThread::~cartesianVisualThread() {
    // do nothing
}

bool cartesianVisualThread::threadInit() {
    decayingWeight = 0;

    // initialization of the attributes
    width  = WIDTH;
    height = HEIGHT;
    idle   = false;

    processedImage = new ImageOf<PixelMono>();
    processedImage->resize(width, height);

    // opening the port for direct input
    if (!inputPort.open(getName("/event:i").c_str())) {
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

void cartesianVisualThread::setName(string str) {
    this->name=str;
}


std::string cartesianVisualThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void cartesianVisualThread::setInputPortName(string InpPort) {
    
}

void cartesianVisualThread::visualizationResume(){
    pt->resume();
}

void cartesianVisualThread::visualizationSuspend(){
    pt->suspend();
}

bool cartesianVisualThread::test(){
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

void cartesianVisualThread::run() {    
    while (isStopping() != true) {
        int result;
        counter++;
        if(!idle){
            if(inputPort.getInputCount()) {
                checkImage.wait();
                inputBottle = inputPort.read(false);   //blocking reading for synchr with the input
                if(inputBottle!=NULL){
                    updateSalience(inputBottle);
                }
                result = processing();
                checkImage.post();
                
                //passing the image to the plotter
                checkImage.wait();
                pt->copyLeft(processedImage);
                checkImage.post();            
            }
            else {
                result = 0;
            }

            if (outputPort.getOutputCount()) {
                Bottle b = outputPort.prepare();
                b.addInt(result);
                outputPort.write();  
            }
        
        }
    }               
}

void cartesianVisualThread::updateSalience(yarp::os::Bottle* b) {
    //get the value in the bottle
    yInfo("received bottle: %s ",b->toString().c_str());
    angle    = b->get(0).asDouble();
    salience = b->get(1).asDouble();
    yInfo("received angle %f %f", angle, salience);
    // the field of view is in the range [-25deg, 25deg]
    azimuthPixel = angle * (130/25) + 160;
    //salience = 10.0;
    decayingWeight = 255;
}

int cartesianVisualThread::processing(){
    //get the image associated to the outputport
    unsigned char* p = processedImage->getRawImage();
    int width   = processedImage->width();
    int height  = processedImage->height();
    int padding = processedImage->getPadding();
    int distanceAzimuth;
    int intensity;
    for (int r = 0; r < height; r++ ) {
        for (int c = 0; c < width; c++) {

            distanceAzimuth =  sqrt((double)((c - azimuthPixel) * (c - azimuthPixel)));
            if(distanceAzimuth < 10)
                intensity = gaussianWeights[distanceAzimuth];
            else
                intensity = 0.0;
            
            if (r == 0) intensity = 0.0;

            *p = (int) decayingWeight * intensity * salience;

            //if (c == azimuthPixel){
            //    *p = decayingWeight;
            //}
            //else {
            //    *p = 0;
            //}    
            p++;
        } 
        p += padding;
    }

    if((counter%400==0)) {
        if(decayingWeight>1) {
            decayingWeight--;
        }
    }
    
    return 1;
}

void cartesianVisualThread::threadRelease() {
    // nothing    
}

void cartesianVisualThread::onStop() {
    delete processedImage;
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

