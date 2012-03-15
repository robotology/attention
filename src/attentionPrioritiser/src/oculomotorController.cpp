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
    iter  = 0;
    jiter = 1;
}

oculomotorController::oculomotorController(attPrioritiserThread *apt) : RateThread(THRATE){
    ap =  apt;
    count = 0;
    iter  = 0;
    jiter = 1;
    state_now = 0;
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
    double t;
    for(int row = 0; row < 66; row++ ) {
        for(int col = 0; col < 11; col++) {
            t = rand() / 10000000000.0 ;
            *val = t; val++;
        }
    }
    printf("%s \n", Psa->toString().c_str());
    Time::delay(5.00);
    
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

bool oculomotorController::policyWalk(){
    bool ret = false;
    printf("%d \n", A->operator()(0,state_now));
    action_now = A->operator()(0,state_now);
    printf("selected action %d %s \n",action_now,stateList[action_now].c_str());
    
    //looking at the Psa for this state and the selected action
    int pos = state_now * NUMACTION + action_now;
    printf("looking for position %d  : %d %d\n", pos, state_now, action_now);
    Vector v = Psa->getRow(pos);
    printf("v = %s \n", v.toString().c_str());
    double maxInVector = 0.0;
    int posInVector = 0;
    for(int j = 0; j < v.size(); j++) {
        if(v[j] > maxInVector) {
            maxInVector = v[j];
            posInVector = j;
            //Psa->operator()(pos,j) -= 0.01;
        }
        
    }
    printf("max value found in vector %f \n", maxInVector);
    state_next = posInVector;
    if(state_next == 10) {
        count++;
        ret = true;
    }
    printf("new state %d \n", state_next);
    return ret;
}


bool oculomotorController::randomWalk() {
    bool ret = false;
    double a = (rand() / 100000000) % NUMACTION ;
    action_now = (int) a;
    printf(" %f \n", a);
    printf("selected action %d %s \n",action_now,stateList[action_now].c_str());

    //looking at the Psa for this state and the selected action
    int pos = state_now * NUMACTION + action_now;
    printf("looking for position %d  : %d %d\n", pos, state_now, action_now);
    Vector v = Psa->getRow(pos);
    printf("v = %s \n", v.toString().c_str());
    double maxInVector = 0.0;
    int posInVector = 0;
    for(int j = 0; j < v.size(); j++) {
        if(v[j] > maxInVector) {
            maxInVector = v[j];
            posInVector = j;
            //Psa->operator()(pos,j) -= 0.01;
        }
        
    }
    printf("max value found in vector %f \n", maxInVector);
    if(state_next == 10) {
        count++;
    }
    state_next = posInVector;
    printf("new state %d \n", state_next);
    return ret;
}

void oculomotorController::learningStep() {
    iter++;
    //1 . updating the quality of the current state
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
    // 2 .action selection and observation of the next state
    bool sinkState;
    printf("-------------count % d------------------------------ \n", count);
    if(count < 30) {
        printf("randomWalk \n");
        sinkState = randomWalk();
    }
    else {
        printf("policyWalk \n");
        sinkState = policyWalk();
    }

    // 3 .updating the quality function of the next state: TD step
    Q->operator()(state_next,action_now) = 
        (1 - alfa) * Q->operator()(state_now,action_now) + 
        alfa * ( rewardStateAction->operator()(state_now,action_now) + j * V->operator()(0,state_now)) ;

    // 4. calculating the total Payoff
    totalPayoff = totalPayoff + rewardStateAction->operator()(state_now, action_now) * jiter;
    jiter  = jiter * j;

    // 5. moving to next state
    if(sinkState) {
        state_now = 0;
    }
    else {
        state_now = state_next;
    }

    printf("end of the learning step \n");
    printf("\n");
    printf("\n");
}

void oculomotorController::run() {
    if(count < 50) {
        learningStep();    
    }
}

void oculomotorController::threadRelease() {
    inCommandPort.close();
}
