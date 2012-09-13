// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file prioCollectorThread.cpp
 * @brief Implementation of the thread of prioritiser Collector(see header prioCollectorThread.h)
 */

#include <iCub/prioCollectorThread.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10

prioCollectorThread::prioCollectorThread() {
    
}

prioCollectorThread::~prioCollectorThread() {
    
}

bool prioCollectorThread::threadInit() {
    printf("----------------------------------------prioCollectorThread::threadInit:starting the thread.... \n");
    /* open ports */
    string rootName("");
    rootName.append(getName("/cmd:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inCommandPort.open(rootName.c_str());
    printf("----------------------------------------prioCollectorThread::threadInit():end of the initialisation \n");
    return true;
}

void prioCollectorThread::interrupt() {
    inCommandPort.interrupt();
}

void prioCollectorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string prioCollectorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void prioCollectorThread::run() {
    while(isStopping() != true){
        Bottle* b=inCommandPort.read(true);
        if(b!=0) {
            //printf(" bottle received : %s ",b->toString().c_str());
            setChanged();
            notifyObservers(b);
        }
    }
}

void prioCollectorThread::onStop() {
    inCommandPort.interrupt();
    inCommandPort.close();
}

void prioCollectorThread::threadRelease() {

}
