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
    state_next = 0;
}

oculomotorController::oculomotorController(attPrioritiserThread *apt) : RateThread(THRATE){
    ap =  apt;
    count = 0;
    iter  = 0;
    jiter = 1;
    state_now = 0;
    cUpdate = 0;
    state_next = 0;
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
    firstCycle = true;
    ap->setAllowStateRequest(0,true);
    ap->setAllowStateRequest(1,true);
    ap->setAllowStateRequest(2,true);
    ap->setAllowStateRequest(3,true);    
    ap->setAllowStateRequest(4,true);

    // initialisation of the matrices necessary for computation
    printf("resetting rewardStateAction \n");
    rewardStateAction = new Matrix(NUMSTATE,NUMACTION);
    double* val = rewardStateAction->data();
    for(int row = 0; row < NUMSTATE; row++ ) {
        for(int col = 0; col < NUMACTION; col++) {
            *val = 0.1; val++;
        }
    }
    printf("initialisation of probability transition \n");
    logFile = fopen("logFile.txt","w+");
     
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
    Psa = new Matrix(NUMSTATE * NUMACTION,NUMSTATE);
    val = Psa->data();
    if(n == 0) {
        // creating new Psa values                
        printf("creating new Psa values \n");
        double t;
        for(int row = 0; row < NUMSTATE * NUMACTION; row++ ) {
            for(int col = 0; col < NUMSTATE; col++) {
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
			//printf("numRead %d > %f \n",numRead,(double)x);
            *val = (double) x;
            val++; countVal++;
        }
         printf("saved %d \n", countVal);
    }
    
    printf("initialisation of the learning machines \n");
    Q = new Matrix(NUMSTATE,NUMACTION);
    Q->zero();
    V = new Matrix(1,NUMSTATE);
    V->zero();
    A = new Matrix(1,NUMSTATE);
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
    //if(state_next == 10) {
    if(true) {
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
    printf("selected action number %d: %s \n",action_now,actionList[action_now].c_str());

    //looking at the Psa for this state and the selected action
    int pos = state_now * NUMACTION + action_now;
    printf("looking for position %d; State:%d,Action:%d\n", pos, state_now, action_now);
    Vector  v = Psa->getRow(pos);
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
    if(allowStateRequest(action_now)) {
        count++;
    }
    
    return ret;
}

bool oculomotorController::allowStateRequest(int state) {
    ap->setAllowStateRequest(state, true);
    bool ret;
    bool executed = ap->executeCommandBuffer(state);

    
    // if not executed because absent in the buffer, waits for few seconds
    if(!executed) {
        double timenow  = Time::now();
        double timediff = 0;
        double timeend;
        bool   validAction;
        
        ap->isValidAction(validAction);
        //waits for a valid action to occur;
        while ((timediff < 5.0)&&(!validAction)) {
            printf("\r%f \r", timediff);
            fflush(stdout); // Will now print everything in the stdout buffer
            timeend = Time::now();
            timediff = timeend - timenow;
            Time::delay(0.1);
        }
        printf("\n");
        if(timeend >= 5.0) {
            printf("the action has not been performed; Timeout occurred \n" );
        }
        else {
            printf("action performed!");
        }
    }
    ap->setValidAction(false);
    


    return ret;
}

void oculomotorController::waitForActuator() {
    printf("----------------wait for actuator in state %d----------------- \n", state_now);
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
    
    //1 . updating the quality of the current state
    M = (*Psa) * V->transposed();
    //printf("V = \n");
    //printf("%s \n", V->toString().c_str());
    
    //printf("M = \n");
    //printf("%s \n", M.toString().c_str());
    
    for (int state = 0; state < NUMSTATE; state++ ) {
        double maxQ  = 0;
        int actionMax = 0;
        
        for(int action = 0; action < NUMACTION; action++) {
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
    //if(count < 30) {
    if(true) {
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
    //waitForActuator(); <-- do not wait for actutor here; it is performed previously


    printf("oculomotorController::learningStep : step performed \n");
    printf("\n");
    printf("\n");
}

void oculomotorController::run() {
    if(!idle) {        
        iter++;   // main temporal counter for visualisation and active learning
         
        if(firstCycle) {
            // interacting with the attPrioritiserThread 
            ap->setAllowStateRequest(0,false);
            ap->setAllowStateRequest(1,false);
            ap->setAllowStateRequest(2,false);
            ap->setAllowStateRequest(3,false);    
            ap->setAllowStateRequest(4,false);  
            firstCycle = false;
        }      
        
        //printf("count %d iter %d \n", count, iter);
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
            
            int statevalue = arg->get(1).asInt();
            printf("                                                                   State %s \n", stateList[statevalue].c_str());
            state_now = state_next;
            state_next = statevalue;             
            printf( "state_now:%d -> state_next:%d \n", state_now, state_next);
            fprintf(logFile, "state_now:%s -> state_next:%s \n", stateList[state_now].c_str(), stateList[state_next].c_str());
            
            /*
            for (int j = 0; j < NUMSTATE; j++)  {
                if(statevalue == j) {
                    printf("                                                                   State %s \n", stateList[j].c_str());
                    state_now = state_next;
                    state_next = j; 
                    
                    fprintf(logFile, "state_now:%s -> state_next:%s \n", stateList[state_now].c_str(), stateList[state_next].c_str());
                    
                }
            }
            */            
            
            // updating the transition matrix once we switch state
            printf("updating the transition matrix \n");
            double sum = 0;
            double* point;
            
            for(int i = 0; i < NUMSTATE; i ++) {
                point = &Psa->operator()(state_now * NUMACTION + action_now, i);
                                
                if(i != state_next) {
                    if (*point >= 0.001) {
                        *point -= 0.001;
                    }
                    sum += *point;
                }
                else {
                    if(*point <= 1.0 - 0.01) {
                        *point += 0.01;
                    }
                    sum += *point;
                }
                
            }
            
            printf("checking if the row sums one \n");
            if(sum > 1.0) {
                printf("!!!!the probability does not sum to 1!!!! \n");
            }
                        
        } break;
        case COMMAND_VOCAB_ACT :{
            printf("new action update arrived %f %f \n", arg->get(1).asDouble(), arg->get(2).asDouble());
            Vector action(8);
            action.zero();
            //int a = (int) arg->get(1).asDouble();
            
            Vector a(6);
            a(0) = arg->get(1).asDouble();
            a(1) = arg->get(2).asDouble();
            a(2) = arg->get(3).asDouble();
            a(3) = arg->get(4).asDouble();
            a(4) = arg->get(5).asDouble();
            a(5) = arg->get(6).asDouble();
            
            /*         
            switch(a) {
            case 0: {action(0) = 1;} break;
            case 1: {action(1) = 1;} break;
            case 2: {action(2) = 1;} break;
            case 3: {action(5) = 1;} break;    
            case 4: {action(5) = 1;} break;
            case 5: {action(5) = 1;} break;
            case 6: {action(6) = 1;} break;
            case 7: {action(7) = 1;} break;
            }
            */

            // mapping from action in prioritiser to action in controller
            // mapping from dimension 6 to dimension 8
            int amplitudeId = 2;
            printf("action %f %f \n", action(0), action(1));
            action(0) = a(0);
            action(1) = a(1);
            action(2) = a(2);
            if(amplitudeId == 2) {
                action(3) = 0;
                action(4) = 0;
                action(5) = 1;
            }
            action(6) = a(4);
            action(7) = a(5);
            
            

            if(action(0)) {
                printf("                                                                  Action reset          \n");
                action_now = 0;
                fprintf(logFile, "action_now:%s ",actionList[action_now].c_str());
            }
            else if(action(1)){
                printf("                                                                  Action vergence       \n");
                action_now = 1;
                fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
            }
            else if(action(2)){
                printf("                                                                  Action smoothPursuit  \n");
                action_now = 2;
                fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
            }
            else if(action(3)){
                printf("                                                                  Action microSaccade   \n");
                action_now = 3;
                fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
            }
            else if(action(4)){
                printf("                                                                  Action mediumSaccade  \n");
                action_now = 4;
                fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
            }
            else if(action(5)){
                printf("                                                                  Action largeSaccade   \n");
                action_now = 5;
                fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
            }
            else if(action(6)){
                printf("                                                                  Action expressSaccade \n");
                action_now = 6;
                fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
            }
            else if(action(7)){
                printf("                                                                  Action predict        \n");
                action_now = 7;
                fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
            }
        } break;
        default: {
            printf("Command not recognized \n");
        }break;
            
        }                
    }
}



void oculomotorController::threadRelease() {
    fclose(logFile);
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
