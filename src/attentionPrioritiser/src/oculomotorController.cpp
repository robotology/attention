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
#include <yarp/math/Math.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

oculomotorController::oculomotorController() : RateThread(THRATE) {
    count = 0;
}

oculomotorController::oculomotorController(attPrioritiserThread *apt) : RateThread(THRATE){
    ap =  apt;
    count = 0;
};

oculomotorController::~oculomotorController() {
    
}

bool oculomotorController::threadInit() {
    printf(" oculomotorController::threadInit:starting the thread.... \n");
    
    // open ports 
    string rootName("");
    rootName.append(getName("/cmd:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inCommandPort.open(rootName.c_str());

    // interacting with the attPrioritiserThread 
    ap->setAllowStateRequest(0,false);
    ap->setAllowStateRequest(1,false);
    ap->setAllowStateRequest(2,false);
    ap->setAllowStateRequest(3,false);    
    ap->setAllowStateRequest(4,false);

    // initialisation of the matrices necessary for computation
    printf("resetting rewardStateAction \n");
    rewardStateAction = new Matrix(11,6);
    double* val = rewardStateAction->data();
    for(int row = 0; row < 11; row++ ) {
        for(int col = 0; col < 6; col++) {
            *val = 0.1; val++;
        }
    }
    printf("initialisation of probability transition \n");
    Psa = new Matrix(66,11);
    val = Psa->data();
    for(int row = 0; row < 66; row++ ) {
        for(int col = 0; col < 11; col++) {
            *val = 0.01; val++;
        }
    }
    
    printf("initialisation of the learning machines \n");
    Q = new Matrix(11,6);
    Q->zero();
    V = new Matrix(1,11);
    V->zero();
    A = new Matrix(1,11);
    A->zero();
    
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

void oculomotorController::policyWalk(){

}


void oculomotorController::randomWalk() {
    int i = rand() % 100;
    
}

void oculomotorController::learningStep() {
    //updating the quality, value and policy    
    M = (*Psa) * V->transposed();
    //printf("V = \n");
    //printf("%s \n", V->toString().c_str());
    
    //printf("M = \n");
    //printf("%s \n", M.toString().c_str());
    
    for (int state = 0; state < 11; state++ ) {
        double maxQ  = 0;
        int actionMax = 0;
        
        for(int action = 0; action < 6; action++) {
            Q->operator()(state, action) = rewardStateAction->operator()(state, action) + j * M(state, action);
            if(Q->operator()(state, action) > maxQ) {
                maxQ = Q->operator()(state, action);
                actionMax = action;
            }
        }
        
        V->operator()(0,state) = maxQ;
        A->operator()(0,state) = actionMax;
        //printf("V = \n");
        //printf("%s \n", V->toString().c_str());
        //printf("state %d maxQ %f actionMax %d \n",state, maxQ, actionMax);
    }
    
    
    //printf("action selection section \n");
    // action selection
    if(count < 10) {
        printf("randomWalk \n");
        randomWalk();
    }
    else {
        printf("policyWalk \n");
        policyWalk();
    }
    printf("end of the learning step \n");
}

void oculomotorController::run() {
    count++;
    learningStep();
    
}

void oculomotorController::threadRelease() {
    inCommandPort.close();
}
