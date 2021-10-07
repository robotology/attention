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
 * @file producer.cpp
 * @brief Implementation of the eventDriven thread (see producer.h).
 */

#include <iCub/producer.h>
#include <cstring>


using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

producer::producer() {
    robot = "icub";        
}

producer::producer(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

producer::~producer() {
    // do nothing
}

bool producer::threadInit() {
    // initialization of the attributes

	timepred_producer  = Time::now();

    //resource
	sharedResource=0;
	pSharedResource=&sharedResource;
	pSem=&sem;

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

    //starting the consumer
    pt = new consumer();
    pt->setName(getName("").c_str());
	pt->setSharedResource(pSharedResource);										  //dichiara met in  consumer.h dicendo che sto passando  punt,   cons.cpp
	pt->setSharedSemaphore(pSem);
	pt->start();									 

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void producer::setName(string str) {
    this->name=str;
}


std::string producer::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void producer::setInputPortName(string InpPort) {
    
}

void producer::visualizationResume(){
    pt->resume();
}

void producer::visualizationSuspend(){
    pt->suspend();
}

bool producer::test(){
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

void producer::run() {    
    while (isStopping() != true) {
  		processing();
    }               
}

int producer::processing(){
	timenow_producer = Time::now();
	timeDiff_producer = timenow_producer-timepred_producer;
	sem.wait();
	sharedResource++; //c is shared,we need a semaphore to synchronize
	printf ("Producer: %d %f \n", *pSharedResource, timeDiff_producer);
	sem.post(); 
	timepred_producer  = Time::now(); 
	Time::delay(0.01);
    return 1;
}



void producer::threadRelease() {
    // nothing    
}

void producer::onStop() {
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

