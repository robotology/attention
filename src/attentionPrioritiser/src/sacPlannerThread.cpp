// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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
 * @file sacPlannerThread.cpp
 * @brief Implementation of the thread of prioritiser Collector(see header sacPlannerThread.h)
 */

#include <iCub/sacPlannerThread.h>
#include <cstring>

using namespace iCub::logpolar;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10

sacPlannerThread::sacPlannerThread() {
    
}

sacPlannerThread::sacPlannerThread(string moduleName) {
    printf("setting the name in the saccPLanner Thread \n");
    setName(moduleName);
}

sacPlannerThread::~sacPlannerThread() {
    
}

bool sacPlannerThread::threadInit() {
    idle = false;
    printf("starting the thread.... \n");
    /* open ports */
    string rootName("");
    //rootName.append(getName("/cmd:i"));
    //printf("opening ports with rootname %s .... \n", rootName.c_str());
    //inCommandPort.open(rootName.c_str());
    
    rootName.append(getName("/corr:o"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    corrPort.open(rootName.c_str());

    //initializing logpolar mapping
    cout << "||| initializing the logpolar mapping" << endl;
    int numberOfRings = 252;
    int numberOfAngles = 152;
    int xSizeValue = 320;
    int ySizeValue = 240;
    double overlap = 1.0;
    if (!trsfL2C.allocLookupTables(L2C, numberOfRings, numberOfAngles, xSizeValue, ySizeValue, overlap)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "||| lookup table L2C allocation done" << endl;

    if (!trsfC2L.allocLookupTables(C2L,xSizeValue, ySizeValue, numberOfRings, numberOfAngles,  overlap)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "||| lookup table C2L allocation done" << endl;

    
    return true;
}

void sacPlannerThread::interrupt() {
    //inCommandPort.interrupt();
}

void sacPlannerThread::setName(string str) {
    this->name = str;
    printf("name: %s", name.c_str());
}

std::string sacPlannerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void sacPlannerThread::run() {
    while(isStopping() != true){        
        //Bottle* b=inCommandPort.read(true);        
        if(!idle) {
            if(corrPort.getOutputCount()) {
                ImageOf<PixelMono>& outputImage =  corrPort.prepare();
                //trsfL2C.logpolarToCart(outputImage,*inputImage);
                outputImage.copy(*inputImage); 
                corrPort.write();
            }
        }
        Time::delay(0.5);
    }
}

void sacPlannerThread::referenceRetina(ImageOf<PixelMono>* ref) {
    inputImage = ref;
}

void sacPlannerThread::onStop() {
    //inCommandPort.interrupt();
    //inCommandPort.close();
    corrPort.interrupt();
    corrPort.close();
}

void sacPlannerThread::threadRelease() {
    
}

void sacPlannerThread::setSaccadicTarget(int rho, int theta) {
    printf("setting Saccadic Planner \n");
    printf("rho %d theta %d \n", rho, theta);
}
