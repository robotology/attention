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
    cUpdate = 0;
}

oculomotorController::oculomotorController(attPrioritiserThread *apt) : RateThread(THRATE){
    ap =  apt;
    count = 0;
    iter  = 0;
    jiter = 1;
    state_now = 0;
    cUpdate = 0;
};

oculomotorController::~oculomotorController() {
    
}

bool oculomotorController::threadInit() {
    printf(" oculomotorController::threadInit:starting the thread.... \n");
    
    // open ports 
    string rootName("");
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    rootName.append(getName("/cmd:i"));
    inCommandPort.open(rootName.c_str());
    scopePort.open(getName("/scope:o").c_str());

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
    printf("reading values from the file.... \n");
    PsaFile = fopen("psaFile.txt","a+");
    int n = 0; // number of bytes in the file
    if (NULL == PsaFile) { 
        perror ("Error opening file");
    }
    else {
        while (!feof(PsaFile)) {
            fgetc (PsaFile);
            n++;
        }
        //fclose (pFile);
        printf ("Total number of bytes: %d \n", n-1);
    }
    n = n -1;
    rewind(PsaFile);
    Psa = new Matrix(66,11);
    val = Psa->data();
    if(n == 0) {
        // creating new Psa values                
        printf("creating new Psa values \n");
        double t;
        for(int row = 0; row < 66; row++ ) {
            for(int col = 0; col < 11; col++) {
                //t = rand() / 10000000000.0 ;
                t = 1.0 / NUMSTATE;
                fprintf(PsaFile,"%f ",t);
                *val = t; val++;
            }
            fprintf(PsaFile,"\n");
        }
    }
    else {
        //reading values
        printf("reading values from file \n");
        // Reads from input file first segment of nsize samples into y:	
		unsigned int i;			
		int numRead = 0, countVal = 0;
        int nsize = 10;
        double y[10];
        float x;
        double* py = &y[0];
		while(numRead != -1){
			numRead = fscanf(PsaFile, "%f", &x);
			printf("numRead %d > %f \n",numRead,(double)x);
            *val = (double) x;
            val++; countVal++;
        }
         printf("saved %d \n", countVal);
    }

    Time::delay(1.00);
    
    printf("initialisation of the learning machines \n");
    Q = new Matrix(11,6);
    Q->zero();
    V = new Matrix(1,11);
    V->zero();
    A = new Matrix(1,11);
    A->zero();

    tp = new trajectoryPredictor();
    tp->setName(getName("").c_str()); 
    tp->start();
    
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

void oculomotorController::waitForActuator() {
    printf("----------------wait for actuator in stete %d----------------- \n", state_now);
    bool   outOfWait = false;
    double timestart = Time::now();
    double timediff  = 0;
    double timeout   = 3.0; // time necessary to perform transition even if action is not completed.
    
    switch (state_now) {
    case 0: { //null
        state_now = 0;
    }break;
    case 1: { //predict
        while ((!outOfWait) || (timediff < timeout)) {
            timediff =  Time::now() - timestart;
            tp->isPredict(outOfWait);
        }

        if(outOfWait) {
            state_now = 1;
        } 
        else {
            state_now = 0;
        }        
    }break;
    case 2: { //fixStableOK
        //ap->isSaccade(outOfWait);
        while ((!outOfWait) || (timediff < timeout)) {
            timediff =  Time::now() - timestart;
            ap->isSaccade(outOfWait);
        }

        if(outOfWait) {
            state_now = 2;
        } 
        else {
            state_now = 3;
        }
    }break;
    case 3: { //fixStableK0
        //ap->isSaccade(outOfWait);
        /*while ((!outOfWait) || (timediff < timeout)) {
            timediff =  Time::now() - timestart;
            ap->isSaccade(outOfWait);
        }

        if(outOfWait) {
            state_now = 1;
        } 
        else {
            state_now = 0;
        }
        */
        state_now = 3;
    }break;
    case 4: { //trackOK
        //ap->isSmoothPursuit(outOfWait);
        while ((!outOfWait) || (timediff < timeout)) {
            timediff =  Time::now() - timestart;
            ap->isSmoothPursuit(outOfWait);
        }

        if(outOfWait) {
            state_now = 4;
        } 
        else {
            state_now = 5;
        }
    }break;
    case 5: { //trackKO
        //ap->isSmoothPursuit(outOfWait);
        /*while ((!outOfWait) || (timediff < timeout)) {
            timediff =  Time::now() - timestart;
            ap->isSmoothPursuit(outOfWait);
        }

        if(outOfWait) {
            state_now = 1;
        } 
        else {
            state_now = 0;
            }*/
        state_now =  5;
    }break;
    case 6: { //anticipOk
        //ap->isAnticip(outOfWait);
        while ((!outOfWait) || (timediff < timeout)) {
            timediff =  Time::now() - timestart;
            ap->isAnticip(outOfWait);
        }

        if(outOfWait) {
            state_now = 6;
        } 
        else {
            state_now = 7;
        } 
    }break;
    case 7: { //anticipWait
        //ap->isAnticip(outOfWait);
        /*while ((!outOfWait) || (timediff < timeout)) {
            timediff =  Time::now() - timestart;
            ap->isAnticip(outOfWait);
        }

        if(outOfWait) {
            state_now = 1;
        } 
        else {
            state_now = 0;
            }*/
        state_now = 7;
    }break;
    case 8: { //vergenceOK
        //ap->isVergence(outOfWait);
        while ((!outOfWait) || (timediff < timeout)) {
            timediff =  Time::now() - timestart;
            ap->isVergence(outOfWait);
        }

        if(outOfWait) {
            state_now = 8;
        } 
        else {
            state_now = 9;
        }
    }break;
    case 9: { //vergenceKO
        //ap->isVergence(outOfWait);
        /*while ((!outOfWait) || (timediff < timeout)) {
            timediff =  Time::now() - timestart;
            ap->isVergence(outOfWait);
        }

        if(outOfWait) {
            state_now = 1;
        } 
        else {
            state_now = 0;
            }*/
        state_now = 9;
    }break;    
    case 10: { //fixating
        state_now = 10;
    }break;        
            
    }
    printf(" ------------------- state_now %d ----------- \n", state_now);
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
    waitForActuator();


    printf("actuator in the decided step \n");
    printf("\n");
    printf("\n");
}

void oculomotorController::run() {
    if(!idle) {
        if((count < 50) && (iter % 20 == 0)) {
            learningStep();    
        }
        
        Bottle& scopeBottle = scopePort.prepare();
        scopeBottle.clear();
        scopeBottle.addDouble(5.0);
        scopePort.write();
    }
}

void oculomotorController::update(observable* o, Bottle * arg) {
    cUpdate++;
    if (arg != 0) {
        //printf("bottle: %s ", arg->toString().c_str());
        int size = arg->size();
        if (0 == size) {
            return;
        }

        switch(arg->get(0).asVocab()) {
        case COMMAND_VOCAB_STAT :{
            printf("new state update arrived %f %f \n", arg->get(1).asDouble(), arg->get(2).asDouble());
            Vector statetmp(5);
            statetmp(0) = arg->get(1).asDouble();
            statetmp(1) = arg->get(2).asDouble();
            statetmp(2) = arg->get(3).asDouble();
            statetmp(3) = arg->get(4).asDouble();
            statetmp(4) = arg->get(5).asDouble();

            if(statetmp(0)) {
                printf("----------------------------------------------State 0 \n");                
                state_now = state_next;
                state_next = 0;
            }
            else if(statetmp(1)){
                printf("----------------------------------------------State 1 \n");
                state_now = state_next;
                state_next = 1;
            }
            else if(statetmp(2)){
                printf("----------------------------------------------State 2 \n");
                state_now = state_next;
                state_next = 2;
            }
            else if(statetmp(3)){
                printf("----------------------------------------------State 3 \n");
                state_now = state_next;
                state_next = 3;
            }
            else if(statetmp(4)){
                printf("----------------------------------------------State 4 \n");
                state_now = state_next;
                state_next = 4;
            }
            
            // updating the transition matrix once we switch state
            double sum = 0;
            for(int i = 0; i < NUMSTATE; i ++) {
                if(i != state_next) {
                    Psa->operator()(state_now * NUMACTION + action_now, i) -= 0.01;
                    sum += Psa->operator()(state_now * NUMACTION + action_now, i);
                }
                else {
                    Psa->operator()(state_now * NUMACTION + action_now, i) += 0.1;
                    sum += Psa->operator()(state_now * NUMACTION + action_now, i);
                }
            }
            if(sum > 1.0) {
                printf("the probability does not sum to 1!!!! \n");
            }
            
        } break;
        case COMMAND_VOCAB_ACT :{
            printf("new action update arrived %f %f \n", arg->get(1).asDouble(), arg->get(2).asDouble());
            Vector action(5);
            action(0) = arg->get(1).asDouble();
            action(1) = arg->get(2).asDouble();
            action(2) = arg->get(3).asDouble();
            action(3) = arg->get(4).asDouble();
            action(4) = arg->get(5).asDouble();

            if(action(0)) {
                printf("----------------------------------------------Action 0 \n");
                action_now = 0;
            }
            else if(action(1)){
                printf("----------------------------------------------Action 1 \n");
                action_now = 1;
            }
            else if(action(2)){
                printf("----------------------------------------------Action 2 \n");
                action_now = 2;
            }
            else if(action(3)){
                printf("----------------------------------------------Action 3 \n");
                action_now = 3;
            }
            else if(action(4)){
                printf("----------------------------------------------Action 4 \n");
                action_now = 4;
            }
        } break;
        default: {
            printf("Command not recognized \n");
        }break;
            
        }
        

        
    }
}



void oculomotorController::threadRelease() {
    fclose(PsaFile);
    PsaFile = fopen("psaFile.txt","w+");
    printf("saving updates in Psa \n");
    double t;
    double* val = Psa->data();
    for(int row = 0; row < 66; row++ ) {
        for(int col = 0; col < 11; col++) {
            t = *val;
            if(t > 0.1) printf("changed value from 0.09 to %f \n", t);
            fprintf(PsaFile,"%f ",t);
            val++;
        }
        fprintf(PsaFile,"\n");
    }
    inCommandPort.close();
    scopePort.close();
    tp->stop();
    
}
