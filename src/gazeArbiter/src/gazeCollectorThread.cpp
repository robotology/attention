// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * Public License fo
 r more details
 */

/**
 * @file gazeCollectorThread.cpp
 * @brief Implementation of the thread of gaze Collector(see header gazeCollectorThread.h)
 */

#include <iCub/gazeCollectorThread.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10

gazeCollectorThread::gazeCollectorThread() {
    
}

gazeCollectorThread::~gazeCollectorThread() {
    
}

bool gazeCollectorThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    string rootName("/");
    rootName.append(getName("/cmd:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inCommandPort.open(rootName.c_str());
    return true;
}

void gazeCollectorThread::interrupt() {
    inCommandPort.interrupt();
}

void gazeCollectorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string gazeCollectorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void gazeCollectorThread::run() {
    while(isStopping() != true){
        Bottle* b=inCommandPort.read(true);
        if(b!=0) {
            //printf(" bottle received : %s \n",b->toString().c_str());
            setChanged();
            notifyObservers(b);
        }
    }
}

void gazeCollectorThread::onStop() {
    inCommandPort.interrupt();
    inCommandPort.close();
}

void gazeCollectorThread::threadRelease() {

}
