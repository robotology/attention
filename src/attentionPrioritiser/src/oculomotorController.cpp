// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file oculomotorController.cpp
 * @brief Implementation of the thread of controller for oculomotor selection(see header oculomotorController.h)
 */

#include <iCub/oculomotorController.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

oculomotorController::oculomotorController() : RateThread(THRATE) {
    
}

oculomotorController::~oculomotorController() {
    
}

bool oculomotorController::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    string rootName("");
    rootName.append(getName("/cmd:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inCommandPort.open(rootName.c_str());
    return true;
}

void oculomotorController::interrupt() {
    inCommandPort.interrupt();
}

void oculomotorController::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string oculomotorController::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void oculomotorController::learningStep() {
    
}

void oculomotorController::run() {
    count++;
    learningStep();
}

void oculomotorController::threadRelease() {

}
