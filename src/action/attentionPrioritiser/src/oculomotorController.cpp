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

using namespace cv;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

#define MAXCOUNTRAND     500.0   // #iteration after which the policy is only quality-based
#define GOALSTATE        14      // goal state success in episodic learning
#define COUNTVERGENCEMAX 5       // counter of the successive vergence command desired


inline bool isnan_fun(double x) {
    return x != x;
}


// Calculates log2 of number.   
double Log2( double n )   
{   
    // log(n)/log(2) is log2.   
    return std::log( (double) n ) / log( (double)2.0 );   
} 


oculomotorController::oculomotorController() : RateThread(THRATE) {
    j             = 0.99999;     // discount factor  default 0.9
    alfa          = 0.6;      // learning rate : how fast learns default 0.5
    countSucc     = 0;
    countStep     = 0;
    countVergence = 0;
    countEntropy  = 0;
  
    iter          = 0;
    jiter         = 1;
    cUpdate       = 0;
    state_next    = 0;
    totalPayoff   = 0;
    entropy       = 0;

    firstCount      = false;
    stateTransition = false;
    firstImage      = true;
    forceWait       = false;
}

oculomotorController::oculomotorController(attPrioritiserThread *apt) : RateThread(THRATE){
    j            = 0.9;
    alfa         = 0.5;
    ap           = apt;
    countSucc    = 0;
    countStep    = 0; 
    countVergence= 0;
    countEntropy = 0;
    iter         = 0;
    jiter        = 1;
    state_now    = 0;
    cUpdate      = 0;
    state_next   = 0;
    totalPayoff  = 0;
    entropy      = 0;
   

    firstCount      = false;
    stateTransition = false;
    firstImage      = true;
    forceWait       = false;
};

oculomotorController::~oculomotorController() {
    
}

bool oculomotorController::threadInit() {
    printf(" ------------------ oculomotorController::threadInit:starting the thread.... \n");

    
    // initialisation of relevant matrices
    Q                 = new Matrix(NUMSTATE,  NUMACTION);
    Psa               = new Matrix(NUMSTATE * NUMACTION, NUMSTATE);
    rewardStateAction = new Matrix(NUMSTATE,  NUMACTION);
    Q->zero();
    Psa->zero();
    rewardStateAction->zero();
    
    // open ports 
    string rootName("");
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    string cmdName = rootName; cmdName.append(getName("/cmd:i"));
    inCommandPort.open(cmdName.c_str());
    string entName = rootName; entName.append(getName("/entImage:i"));
    //entImgPort.open(entName.c_str());
    

    // interacting with the attPrioritiserThread 
    firstCycle = true;
    ap->setAllowStateRequest(0,true);
    ap->setAllowStateRequest(1,true);
    ap->setAllowStateRequest(2,true);
    ap->setAllowStateRequest(3,true);    
    ap->setAllowStateRequest(4,true);
    ap->setAllowStateRequest(5,true);
    ap->setAllowStateRequest(6,true);
    ap->setAllowStateRequest(7,true);
    ap->setAllowStateRequest(8,true);

    if(ap->isLearning()) {}

     
    
    // ------------- opening the logfile ----------------------
   
    //logFilePath = "logFile.txt";
    printf("printing the visited state and performance in a log files .... %s  %s \n", logStatePath.c_str(), logFilePath.c_str());
    logFile  = fopen(logFilePath.c_str(),"w+");
    if (logFile == NULL) {
        return false;
    }
    fprintf(logFile, "\n ");
    
    logState = fopen(logStatePath.c_str(),"w+");
    
    
    
    // --------- Reading Transition Matrix -------------------
    
    printf("initialisation of probability transition \n");
    
    //psaFilePath = "psaFile.txt";
    printf("reading transition Matrix from the file.... %s \n",psaFilePath.c_str());
    PsaFile = fopen(psaFilePath.c_str(),"r+");
    int n = 0; // number of bytes in the file
    double* valPsa = Psa->data();
        
        
    printf("opening psa file \n");
    if (NULL == PsaFile) { 
        printf ("Error opening psa file \n"); 
        return false;
    }
    else {
        
        while (!feof(PsaFile)) {
            fgetc (PsaFile);
            n++;
        }
        
        printf ("Total number of bytes: %d \n", n-1);
        
    }
    
    rewind(PsaFile);
    n = n - 1;
    
    if(n == 0) {
        fclose(PsaFile);
        // creating new Psa values                
        printf("creating new Psa values \n");
        double t;
        for(int row = 0; row < NUMSTATE * NUMACTION; row++ ) {
            for(int col = 0; col < NUMSTATE; col++) {
                //t = rand() / 10000000000.0 ;
                t = 1.0 / (double)NUMSTATE;
                fprintf(PsaFile,"%f ",t);
                *valPsa = t; valPsa++;
            }
            fprintf(PsaFile,"\n");
        }
    }
    else {        
        //reading values
        printf("reading values from file \n");
        // Reads from input file first segment of nsize samples into y:	
        unsigned int i;			
        int numRead = 1; int countVal = 0;
        int nsize = 10;
        double y[10];
        float x;
        double* py = &y[0];
        while((numRead != -1) && (countVal < NUMSTATE * NUMACTION * NUMSTATE)){
            numRead = fscanf(PsaFile, "%f", &x);
            if(numRead != -1) {
                //printf("psa : numRead %d > %f \n",numRead,x);
                *valPsa = (double) x;
                valPsa++; countVal++;
            }
        }
        printf("saved %d \n", countVal);        
        fclose(PsaFile);
    }
    
    
    
    // ------ Reading Reward State Action ------------------
    
    printf("reading reward Matrix from the file.... %s \n",rewardFilePath.c_str());
    
    double* val = rewardStateAction->data();
    
    //rewardFilePath = "rewardFile.txt";
    rewardFile = fopen(rewardFilePath.c_str() ,"r+");
    n = 0; // number of bytes in the file
    
    
    if (NULL == rewardFile) { 
        perror ("Error opening reward file");
        return false;
    }
    else {
        while (!feof(rewardFile)) {
            fgetc (rewardFile);
            n++;
        }
        //fclose (pFile);
        printf ("Total number of bytes: %d \n", n-1);
    }
    n = n - 1;
    rewind(rewardFile);
    
    
    if(n == 0) {
        fclose(rewardFile);
        // creating new reward values                
        printf("creating new reward values \n");
        double t;
        for(int row = 0; row < NUMSTATE ; row++ ) {
            for(int col = 0; col < NUMACTION; col++) {
                t = 0.1;
                fprintf(rewardFile,"%f ",t);
                *val = t; val++;
            }
            fprintf(rewardFile,"\n");
        }
    }
    else {        
        //reading values
        printf("reading values from file \n");
        // Reads from input file first segment of nsize samples into y:	
        unsigned int i;			
        int numRead = 0, countVal = 0;
        //int nsize = 10;
        //double y[10];
        float x;
        //double* py = &y[0];
        while(numRead != -1){
            numRead = fscanf(rewardFile, "%f", &x);
            if(numRead != -1) {
                //printf("reward : numRead %d > %f \n",numRead,x);
                *val = (double) x;
                val++; countVal++;
            }
        }
        
        printf("saved %d \n", countVal);        
        //fclose(rewardFile);    //TODO : check this istruction
    }
     
    
    printf("initialisation of the learning machines \n");
    
    
    
    // --------- Reading Quality Function -------------------         
    
    //qualityFilePath = "qualityFile.txt";
    printf("Quality Function : reading values from the file %s.... \n",qualityFilePath.c_str());
    qualityFile = fopen(qualityFilePath.c_str(),"r+");
    n = 0; // number of bytes in the file
    
    
    if (NULL == qualityFile) { 
        perror ("Error opening Quality file");
        return false;
    }
    else {
        while (!feof(qualityFile)) {
            fgetc (qualityFile);
            n++;
        }
        //fclose (pFile);
        printf ("Total number of bytes: %d \n", n-1);
    }
    n = n - 1;
    rewind(qualityFile);
    //V = new Matrix(NUMSTATE,NUMSTATE);   
    
    val = Q->data();
    
    if(n == 0) {
        fclose(qualityFile);
        // creating new quality values                
        printf("Value Function : creating new Values Function \n");
        double t; 
        for(int row = 0; row < NUMSTATE; row++ ) {
            for(int col = 0; col < NUMACTION; col++) {
                //t = rand() / 10000000000.0 ;
                t = 0.0;
                fprintf(qualityFile,"%f ",t);
                *val = t; val++;
            }
            fprintf(qualityFile,"\n");
        }
    }
    else {        
        
        //reading values
        printf("Value Function : reading values from file \n");
        // Reads from input file first segment of nsize samples into y:	
        unsigned int i;			
        int numRead = 0, countVal = 0;
        //int nsize = 10;
        //double y[10];
        float x;
        //double* py = &y[0];
        int col = 0, row =0;
        countVal =0;
        while((numRead != -1)&&(countVal < NUMACTION * NUMSTATE)){
            numRead = fscanf(qualityFile, "%f", &x);
            if(numRead != -1) {
                Q->operator()(row,col) = (double) x;
                //printf("quality : numRead %d (%d,%d) > %f \n",numRead,row,col,numRead,(double)x);
                
                col++;
                if(col == NUMACTION) {
                    row++;
                    col = 0;
                }
                countVal++;
                //*val = (double) x;
                //val++; coun                tVal++;
            }
        }
        printf("saved %d \n", countVal);
        fclose(qualityFile);
    }
        
    printf("init quality measure and action state \n");

    // other needed matrices
    V = new Matrix(1,NUMSTATE);
    V->zero();
    //V[0,GOALSTATE] = 100; //a-priori value of the state because never visited
    
    printf("initialisation of the P matrix %d %d \n", NUMSTATE, NUMACTION);
    Matrix* tP = new Matrix(NUMSTATE,NUMACTION);
    P = tP;
    P->zero();

    // max-reward action vector
    // vector populated with the action that maximise reward in a particular state
    // the column of the vector reference to the state
    printf("initialisation of the A matrix %d \n", NUMSTATE);
    Matrix* tA = new Matrix(2,NUMSTATE);
    A = tA;
    A->zero();
    
    //printf(" init of the trajectoryPredictor \n");
    //tp = new trajectoryPredictor();
    //tp->setResourceFinder(rf);
    //tp->setName(getName("").c_str()); 
    //tp->start();

    //printf(" init of the outing thread \n");
    //ot = new outingThread();
    //ot->setName(getName("").c_str()); 
    //ot->start();
        
    //idle = true; // TODO: removed this hard coded initialisation. Possible memory leak. 18/10/12

    if(idle)
        printf("oculomotorController : idleState = true \n");
    else
        printf("oculomotorController : idleState = false \n");
        

    printf("\n ------------------------oculomotorController::threadInit:initialisation correctly ended \n");
    
    
    
    return true;
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

void  oculomotorController::resize(int _width, int _height) {
    width  = _width;
    height = _height;
}

bool oculomotorController::policyWalk(double policyProb){
    countStep++;
    ap->setFacialExpression("R04");
    ap->setFacialExpression("L04");

    printf("---------------------- oculomotorController::policyWalk: just set facial expressions \n");
    
    bool ret = false;
    printf("---------------------- state_now : %d %f \n",state_now, A->operator()(0,state_now));
    action_now = A->operator()(0,state_now);
    yarp::sig::Vector a = A->getRow(0);
    printf("---------------------- selected action %d %s \n",action_now,stateList[action_now].c_str());
    printf("---------------------- a = %s \n", a.toString().c_str());
    
    
    //looking at the Psa for this state and the selected action
    int pos = state_now; // * NUMACTION + action_now;
    printf("looking for position %d  ; %d %d\n", pos, state_now, action_now);
    yarp::sig::Vector v = Q->getRow(pos);
    
    printf("v = %s \n", v.toString().c_str());
    double maxInVector = 0.0;
    yarp::sig::Vector c(v.size());
    double sum = 0;
    int posInVector = 0;
    for(int j = 0; j < v.size(); j++) {
        if(v[j] > maxInVector) {
            maxInVector = v[j];
            posInVector = j;
        }
        sum += v[j];
        c[j] = sum;
    }

    printf("c : %s \n", c.toString().c_str());

    double ifPolicyAction = Random::uniform() ;
    int j;
    if(ifPolicyAction > policyProb) {
        // performing completely random action
        double randAction   = Random::uniform() * c[c.size() - 1]; //random value time the last entry cumulative
        //double randAction = (rand() / 1000000000.0);        
        for (j = 0 ; j < c.size(); j++) {
            if (c[j] > randAction) break;
        }
        if (j>=NUMACTION) j = NUMACTION - 1;
        printf("in policy selection - randomAction %f . action selected %d \n", randAction, j);
    }
    else {
        // performing the action with greater quality measure
        j = posInVector; //best choice given previous run
        printf("in policy selection - policy selection \n");
    }

    
    // multiple vergence commands
    if ((j == 2)&&(countVergence == 0)) {
        countVergence = COUNTVERGENCEMAX;
    }
    if(countVergence > 0) {
        action_now = 2;
        printf("force action vergence; decrement %d \n", countVergence);
        countVergence--;
    }
    else {
        action_now = j;
    }

   
    /*
    printf("max value found in vector %f \n", maxInVector);
    state_next = posInVector;
    //if(state_next == 10) {
    //    count++;
    //    ret = true;
    //}
    printf("new state %d \n", state_next);
    */

    
    // trying to execute the selected action 
    // if successful the system moves to the next state
    //action_now = j;
    if(allowStateRequest(action_now)) {
        //count++;
        //statenext = state_next;
        //statevalue = posInVector;
        //printf("success in the action, probably sets a new statevalue %d \n", statevalue);
        ret = true;
    }


    return ret;

}


bool oculomotorController::randomWalk(int& statenext, double randomProb) {
    countStep++;
    ap->setFacialExpression("R01");
    ap->setFacialExpression("L01");
   
    
    bool ret = false;
    
    double a = Random::uniform() * NUMACTION;
    if((int) a == 2) {
        //printf("Counting vergence 3 \n");
        // initializing the counter of successive vergence commands to default max number
        countVergence = COUNTVERGENCEMAX;
    }
    
    //printf(" a =%f, countVergence=%d \n", a, countVergence);
    printf("if random, selects action number %d: %s \n",(int)a,actionList[(int)a].c_str());

    //looking at the Psa for this state and the selected action
    int pos = state_now; // * NUMACTION + action_now;
    printf("------------------- looking for position %d; State:%d,Action:%d\n", pos, state_now,(int) a);
    yarp::sig::Vector  v = Q->getRow(pos);
    //printf("v = %s \n", v.toString().c_str());
    
    // given the vector of transition and considering the transition probability
    // select the action and build the cumulative vector
    double maxInVector = 0.0;
    yarp::sig::Vector c(v.size());
    double sum = 0;
    int posInVector = 0;
    for(int j = 0; j < v.size(); j++) {
        if(v[j] > maxInVector) {
            maxInVector = v[j];
            posInVector = j;
        }
        sum += v[j];
        c[j] = sum;
    }

    //printf("c = %s \n", c.toString().c_str());
    //printf("max value found in position %d  of vector : %f  \n", posInVector, maxInVector);

    
    // selecting using probability density function for the state/action
    // the probability that the selection comes from stochastic measure is given by ifRandAction
    double ifRandAction = Random::uniform();
    int j;
    if(/*ifRandAction > randomProb*/true) {
        // performing completely random action
        double randAction   = Random::uniform();
        
        //double randAction = (rand() / 1000000000.0);        
        //for (j = 0 ; j < c.size(); j++) {
        //    if (c[j] > randAction) break;
        //}
        //printf("------------------- randomAction %f . action selected %d \n", randAction, j);


        if(countVergence > 0) {
            printf("countVergence %d \n", countVergence);
            action_now = 2;
            countVergence--;
        }
        else {
            action_now = (int) a;
        }
 
    }
    else {
        printf("------------------- performing policy selection in randomWalk \n");
        // performing the action with greater quality measure
        j = posInVector; //best choice given previous run
        action_now = j;
    }

    //if(forceWait) {
    //    forceWait  = false;
    //    action_now = 1;      //forcing wait action 
    //}
    

    // trying to execute the selected action 
    // if successful the system moves to the next state
    if(allowStateRequest(action_now)) {
        //count++;
        //statenext = state_next;
        statevalue = j;
        printf("success in the action, probably sets a new statevalue %d \n", statevalue);
        ret = true;
    }

    /*
    state_next = posInVector;
    if(state_next == 0) {
        if(firstCount) {
            count++;
            firstCount  = false;
            totalPayoff = 0;
        }
    }
    else {
        firstCount = true;
    }    
    printf("new state %d \n", state_next);
    */
    
    
    return ret;
}

bool oculomotorController::allowStateRequest(int action) {
   
   
    //setting flags in initialisation
    //ap->setAllowStateRequest(action, true);

  
    
    // executing command in buffer
    bool ret = false;
    bool executed;
    executed = ap->executeCommandBuffer(action);
    
    
    /*
    if(!executed) {
        printf("executing clone action \n");
        ap->executeClone(action);
        ret = true;
        logAction(action);
    }
    else {
        // action found and executed
        ret = true;
        //logging the action in the file
        logAction(action);
    }
    */

    
    // if not executed because absent in the buffer, waits for few seconds
    if(!executed) {
        //double timenow  = Time::now();
        //double timediff = 0;
        //double timeend;
        //bool   validAction;

 
        /*ap->isValidAction(validAction);
        printf("checking valid action %d \n", validAction);
        
        //waits for a valid action to occur;
        while ((timediff < 0.3)&&(!validAction)) {
            printf("\r%f \r", timediff);
            fflush(stdout); // Will now print everything in the stdout buffer
            timeend = Time::now();
            timediff = timeend - timenow;
            Time::delay(0.1);
        }
        printf("\n");

        if(timeend >= 0.3) {
            printf("the action has not been performed; Timeout occurred \n" );
            ret = false;
        }
        else {
            printf("action performed!");
            ret = true;
            //logging the action in the file
            logAction(action);
        }
        */
        printf("\n");
        ret = false;

    }
    else {
        // action found and executed
        ret = true;
        //logging the action in the file
        logAction(action);
    }
    
   
    //setting flags at the end of the function
    
    //printf("setting valid action to false (action %d) \n", action);
    ap->setValidAction(false);
    ap->setAllowStateRequest(action, false);

    return ret;
}



void oculomotorController::learningStep() {
    
    //1 . updating the quality of the current state
    M = (*Psa) * V->transposed();
    //printf("V = \n");
    //printf("%s \n", V->toString().c_str());
    
    //printf("M = \n");
    //printf("%s \n", M.toString().c_str());
    
    
    //calculating the V
    //printf("calculating the V..... \n");
    for (int state = 0; state < NUMSTATE; state++ ) {
        double maxQ  = 0;
        int actionMax = 0;
        
        for(int action = 0; action < NUMACTION; action++) {

            //P->operator()(state, action) = rewardStateAction->operator()(state, action) + j * M(state, action);
            //if(P->operator()(state, action) > maxQ) {
            //    maxQ = P->operator()(state, action);
            //    actionMax = action;
            //}

            //Q->operator()(state, action) = rewardStateAction->operator()(state, action) + j * M(state, action);
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
    
    
    bool _stateTransition;
    mutexStateTransition.wait();
    _stateTransition = stateTransition;
    mutexStateTransition.post();
    
    // stateTransition branch is executed when the stateTransion = false
    // this flag is set true when an action is performing
    // this flag is set false when the action is accomplished
    if(stateTransition) {
        printf("state in transition! skip learning step \n");
    }

    if(!_stateTransition) {
        //printf("!stateTransition branch \n");
        
        // 2 .action selection and observation of the next state
        bool actionPerformed;
        int state_next;

        double k = 1.0 -  ((double)countStep / MAXCOUNTRAND );
        if(k < 0) {
            k = 0;
        }
        double s = Random::uniform();

        printf(" executing learning step:  countSucc %d countStep %d  state_now %d s %f k %f______________ \n \n", countSucc,countStep, state_now, s, k);

       
        //if(countSucc == MAXCOUNTRAND) {
        //    printf("always IN THE POLICY!!!! \n");
        //    fprintf(logFile,"IN THE POLICY!!!! \n");
        //}


        
        /*if (countSucc < 1) {
           printf("randomWalk action selection \n");
           fprintf(logFile,"randomWalk > ");
           actionPerformed = randomWalk(state_next,0.1); 
        }
        else */

       
        
        
        printf("s=%f > k=%f? \n ", s, k);
        if(s >= k) {
            printf("------------------- policyWalk action selection \n");
            fprintf(logFile,"policyWalk > ");
            //printf("policyWalk > \n");
            // the probability as parameter introduces the probability
            // of a random action within the policy walk
            actionPerformed = policyWalk(0.75);
            //printf("success in policy action performed \n");
        }
        else {
            printf("------------------- randomWalk action selection \n");
            fprintf(logFile,"randomWalk > ");
            actionPerformed = randomWalk(state_next,0.1);
        }


        
    
        if(actionPerformed) {
            printf("waiting for transition after action selection \n");
            mutexStateTransition.wait();
            stateTransition = true;
            mutexStateTransition.post();

            
            // 3 .updating the quality function of the next state: TD step
            
            /*
            // calculating the reward using (amplitude, temporal aspects, accuracy)
            //r = rewardStateAction->operator()(state_now,action_now);
                        
            //Q->operator()(state_next,action_now) = 
            //    (1 - alfa) * Q->operator()(state_now,action_now) + 
            //    alfa * ( rewardStateAction->operator()(state_now,action_now) + j * V->operator()(0,state_now)) ;
            
            Q->operator()(state_now,action_now) = 
            Q->operator()(state_now,action_now) + 
            alfa * ( rewardStateAction->operator()(state_now,action_now) + j * V->operator()(0,state_next) 
            - Q->operator()(state_now,action_now)) ;
            
            // 4. calculating the total Payoff
            printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.adding the reward  in pos (%d, %d) %f %f ",state_now, action_now, rewardStateAction->operator()(state_now, action_now) * jiter,Q->operator()(state_now,action_now) );
            totalPayoff = totalPayoff + rewardStateAction->operator()(state_now, action_now) * jiter;
            printf("for the final totalPayoff %f \n", totalPayoff);
            jiter  = jiter * j;        
            
            // 5. moving to next state
            //state_now = state_next;
            */
            
        }
        else {
            //state_now = state_next;
            printf("action not performed in the learning. \n");
        } 
    } //end if(!_stateTransition)

    printf("========================oculomotorController::learningStep  performed ==================== \n\n\n");
    
}

double oculomotorController::calculateEntropy(yarp::sig::ImageOf<yarp::sig::PixelBgr>* entImg,double& entropy, int& counter) {
    //printf("calculating entropy.... %d %d \n", width, height);
    ImageOf<PixelRgb>* hsv = new ImageOf<PixelRgb>;
    hsv->resize(width, height);
    cvCvtColor(entImg->getIplImage(), hsv->getIplImage(), CV_BGR2HSV);

    //cv::Mat hsv;
    //IplImage hsvI;
    //cv::Mat* p = (cv::Mat*) entImg.getIplImage();
    //cvtColor(entImg->getIplImage()) , hsvI, CV_BGR2HSV);

    // let`s quantize the value to 30 levels
    // let's quantize the hue to 30 levels
    // and the saturation to 32 levels
    int hbins = 30, sbins = 32,  vbins = 30;
    //MatND* hist = new MatND();
    MatND hist;
    int histSize[] = {hbins, sbins, vbins};
    // hue varies from 0 to 179, see cvtColor
    float hranges[] = { 0, 180 };
    // saturation varies from 0 (black-gray-white) to
    // 255 (pure spectrum color)
    float sranges[] = { 0, 256 };
    
    float vranges[] = { 0, 256 } ;
    const float* ranges[] = { hranges, sranges, vranges };
    
    // we compute the histogram from the 0-th and 1-st channels
    int channels[] = {0, 1, 2};


    //cv::Mat matImage;
    const cv::Mat mat( cv::cvarrToMat(hsv->getIplImage()));
    //cvConvert(hsv->getIplImage(),mat);
    //    matImage.data = (char *)hsv->getIplImage().imageData;
    //nst cv::Mat* cvHsvP = (const cv::Mat*)hsv->getIplImage();
    calcHist(&mat,1,channels, Mat(), // do not use mask
        hist, 3, histSize, ranges,
        true, // the histogram is uniform
        false );
    


    //double maxVal=0;
    //minMaxLoc(hist, 0, &maxVal, 0, 0);
    yarp::sig::Matrix hsProb(hbins, sbins);    
    yarp::sig::Vector vProb(vbins);

    // calculating entropy in v
    double sumEntropyV = 0;

    for( int v = 0 ; v < vbins; v++) {
        // calculating entropy in h
        double sumEntropyH = 0;
        for( int h = 0; h < hbins; h++ ) {
            double sum = 0;
            double sumEntropyS =0;
            for( int s = 0; s < sbins; s++ )
                {
                    float binVal = hist.at<float>(h, s, v);
                    sum += binVal;
                    
                    //int intensity = cvRound(binVal*255/maxVal);
                }
            for( int s = 0; s < sbins; s++ )
                {
                    float binVal = hist.at<float>(h, s, v);
                    hsProb(h,s) = binVal / sum; // extracing the probability
                    double entropyS     =  -1 * hsProb(h,s) * Log2(hsProb(h,s)) ;
                    if(!isnan_fun(entropyS))
                        sumEntropyS += entropyS;
                    //printf("entropyS = %f upto 5.0 \n", entropyS,sumEntropyS);
                }
            sumEntropyH += sumEntropyS;
            //printf("sumEntropyS %f \n", sumEntropyS);
        }
        //printf("entropyH = %f upto 1500 \n", sumEntropyH);    
        sumEntropyV += sumEntropyH;
    }
    //printf("entropyV = %f upto 4800 \n",sumEntropyV);
    entropy += sumEntropyV;
    return (entropy / counter);
}

void oculomotorController::run() {
    
    if(!idle) {        
        iter++;   // main temporal counter for visualisation and active learning
         
        //printf(".............................................................cycle %d \n", iter);
        Time::delay(1.0);
        if(firstCycle) {
            // interacting with the attPrioritiserThread 
            // sets all flag that allow action to false
            ap->setAllowStateRequest(0,false);
            ap->setAllowStateRequest(1,false);
            ap->setAllowStateRequest(2,false);
            ap->setAllowStateRequest(3,false);    
            ap->setAllowStateRequest(4,false);  
            ap->setAllowStateRequest(5,false);
            ap->setAllowStateRequest(6,false);
            ap->setAllowStateRequest(7,false);
            ap->setAllowStateRequest(8,false);            
            firstCycle = false;
        } 

        // calculating the entropy
        /*
        printf("calculating the entropy \n");
        ImageOf<PixelBgr> *entImg = entImgPort.read(false);
        if(entImg!=NULL) {
            if(firstImage) {
                firstImage = false;                
                resize(entImg->width(), entImg->height());
            }
            countEntropy++;
            meanEntropy = calculateEntropy(entImg,entropy,countEntropy);
            
            //fprintf(logFile,"entropy: %f \n", entropy);
            //fprintf(logState, "%f ", entropy);            
        }
        */

        //printf("countSucc %d iter %d readyforAction %d \n",countSucc, iter, ap->readyForActions() );
        if((countSucc < 40) && (iter % 1 == 0) && (ap->readyForActions())) {
            printf("================================COUNTSUCC %d ================================ \n", countSucc);
            //printf("learning step \n");
            //fprintf(logFile,"%d ",iter);
            learningStep();
            
        }
        
        ot->setValue(totalPayoff);
        
        //Bottle& scopeBottle = scopePort.prepare();
        //scopeBottle.clear();
        //scopeBottle.addFloat32(totalPayoff);
        //scopePort.write();
    }
    
}

void oculomotorController::logAction(int a) {
    yarp::sig::Vector action(8);
    action.zero();
    // mapping from action in prioritiser to action in controller
    // mapping from dimension 6 to dimension 8
    int amplitudeId = 2;
    printf("######## Action %d --> ", a);

    
    // this must be  active only when it is not learning
    // when it is not learning idle is true
    if(idle){
        switch(a) {
        case 0 : {
            action(0) = 1;
            printf("log-action 0  \n");  //reset
        }
            break;
        case 1 : {
            action(1) = 1;
            printf("log-action 1  \n");  //wait
        }
            break;
        case 2 :{
            action(2) = 1;
            printf("log-action 2  \n");  //vergence
        }
            break;
        case 3 :
            action(3) = 1;
            printf("log-action 3  \n");  //SMP
            break;
        case 4 :
            if(amplitudeId == 2) {       //saccade->
                action(4) = 0;           //micro  saccade
                action(5) = 0;           //medium saccade
                action(6) = 1;           //large  saccade
            }
            printf("log-action 6  \n");
            break;
        case 5 : {
            action(7) = 1;
            printf("log-action 7  \n");  //express Saccade
        }
            break;
        case 6  :{
            action(8) = 1;
            printf("log-action 8  \n");  //prediction
        }
            break;
            
        }
    }// end idle
    else {
        action(a)= 1;
    }
   
    
    if(action(0)) {
        printf("                                                                  Action reset          \n");
        action_now = 0;
        fprintf(logFile, "action_now:%s ",actionList[action_now].c_str());
    }
    else if(action(1)){
        printf("                                                                  Action wait       \n");
        action_now = 1;       
        fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
    }
    else if(action(2)){
        printf("                                                                  Action vergence       \n");
        action_now = 2;       
        fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
    }
    else if(action(3)){
        printf("                                                                  Action smoothPursuit  \n");
        action_now = 3;       
        fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
    }
    else if(action(4)){
        printf("                                                                  Action microSaccade   \n");
        action_now = 4;       
        fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
    }
    else if(action(5)){
        printf("                                                                  Action mediumSaccade  \n");
        action_now = 5;
        fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
    }
    else if(action(6)){
        printf("                                                                  Action largeSaccade   \n");
        action_now = 6;        
        fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
    }
    else if(action(7)){
        printf("                                                                  Action expressSaccade \n");
        action_now = 7;       
        fprintf(logFile, "action_now:%s ", actionList[action_now].c_str());
    }
    else if(action(8)){
        printf("                                                                  Action predict        \n");
        action_now = 8;        
        fprintf(logFile, "action_now: %s ", actionList[action_now].c_str());
        //fprintf(logFile, "action_now: \n ");
    }
    printf("ultimated the logAction method \n");
}

double oculomotorController::estimateReward(double timing, double accuracy, double amplitude, double frequency) {
    double res = (accuracy / 10.0) 
        - timing /*s*/ * frequency /*#*/ * costAmplitude[action_now]/*1/degree*/ * amplitude /*degree*/ 
        - timing  * frequency * costEvent[action_now];

    //if(res < 0) res = 0;
    return res;
}

void oculomotorController::update(observable* o, Bottle * arg) {
    printf("update \n");
    cUpdate++;
    if (arg != 0) {
        printf("###############bottle: %s ", arg->toString().c_str());
        int size = arg->size();
        if (0 == size) {
            return;
        }

        switch(arg->get(0).asVocab32()) {
        case COMMAND_VOCAB_STAT :{
            // called from the observed class (attentioPrioritiser)
                     
            //int actionvalue = (int) arg->get(1).asFloat32();  // action that is just finalized
            int actionvalue = 0;
            int statevalueparam    = (int) arg->get(1).asFloat32();
            double timing          =       arg->get(2).asFloat32();
            double accuracy        =       arg->get(3).asFloat32();
            double amplitude       =       arg->get(4).asFloat32();
            double frequency       =       arg->get(5).asFloat32();
            
            printf("\n timing:%f accuracy:%f amplitude:%f frequency:%f \n", timing, accuracy, amplitude, frequency);           

            
            //printf("new state update arrived action: %d state :%f  \n", actionvalue, arg->get(2).asFloat32());
            // update for evaluation of pSA without learning
            state_next = statevalueparam;           // the update comes from the attPrioritiser with default state.
            
            printf("state  now = %d \n", state_now);
            printf("action now = %d \n", action_now);

            // --------------------------  updating the entire state of the learner -----------------------------
            //state_now = 0;
            // 3 .updating the quality function of the next state: TD step
            
            // calculating the quality of state function
            
            //estimate the reward 
            //double r = rewardStateAction->operator()(state_now,action_now) ;
            double   r = estimateReward(timing, accuracy, amplitude, frequency);
            //double r = accuracy / 10.0 - timing * cost[action_now] * amplitude;
            printf("calculated the accuracy for state, action %d,%d r= %f \n", state_now,action_now, r);

            if (statevalueparam == GOALSTATE) {
                r += 2000;
                printf("ENDED in FINAL GOALSTATE! \n");
            }
            if(statevalueparam == 12){
                forceWait = true;
            }
            
            
            //Q->operator()(state_next,action_now) = 
            // //    (1 - alfa) * Q->operator()(state_now,action_now) + 
            // //    alfa * ( rewardStateAction->operator()(state_now,action_now) + j * V->operator()(0,state_now)) ;
            double Q_prev = Q->operator()(state_now,action_now);
            Q->operator()(state_now,action_now) = 
                Q->operator()(state_now,action_now) + 
                alfa * ( r + j * V->operator()(0,state_next) 
                         - Q->operator()(state_now,action_now)) ;

            // std::isinf( value ) and std::isNan( value )
            
            if(Q->operator()(state_now,action_now) != Q->operator()(state_now,action_now)) {
                printf("UPDATING THE QUALITY with NAN!!!!!\n");
                printf("state_now %d \n action_now %d \n state_next = %d \n r=%f \n j=%f \n V=%f \n ",
                       state_now, action_now, state_next, r, j,V->operator()(0,state_next) );                
                Q->operator()(state_now,action_now) = Q_prev;
            }
			/*
            if(std::isinf(Q->operator()(state_now,action_now) )){
                printf("UPDATING THE QUALITY with INF!!!!!\n");
                printf("state_now %d \n action_now %d \n state_next = %d \n r=%f \n j=%f \n V=%f \n ",
                       state_now, action_now, state_next, r, j,V->operator()(0,state_next) );    
                Q->operator()(state_now,action_now) = Q_prev;
            }
			*/
            
            // // 4. calculating the total Payoff
            //printf("calculating accuracy \n");
            //printf("adding the reward %f  Q(%d,%d): %f \n", r * jiter,state_now, action_now, Q->operator()(state_now,action_now));
            

            totalPayoff = totalPayoff + r * jiter;
            printf("final totalPayoff %f with jiter %f \n", totalPayoff, jiter);
            jiter  = jiter * j;        
            
            // // 5. moving to next state
            // //state_now = state_next;


            // // !!!!!!!!!!!!!!!!!!!!!!!!!!   STATE TRANSITION  !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
            // -------------------------- updating the transition matrix once we switch state --------------------
            printf("updating the transition matrix; state_now %d action_now %d \n", state_now, action_now);
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
            //printf("checking if the row sums one \n");
            if(sum > 1.0) {
                printf("!!!!the probability does not sum to 1!!!! \n");
            }
            

            //---------------------------  state update arrived ------------------------------------------
            //printf("                                                 new State %s \n",  stateList[statevalue].c_str());
            state_prev = state_now;
            state_now  = state_next;
                         
            printf( "state_prev:%d -> state_now:%d \n", state_prev, state_now);
            fprintf(logFile, "state_prev:%s state_now:%s ", stateList[state_prev].c_str(), stateList[state_now].c_str());
            fprintf(logFile, " totalPayoff:%f / 10 - %f -%f => %f         \n ",accuracy,timing  *  costAmplitude[action_now] * amplitude * frequency,timing  * frequency * costEvent[action_now],totalPayoff);
            //fprintf(logFile, " entropy : %f \n", meanEntropy);
            fprintf(logState,"%d %f %f\n", state_now, totalPayoff, meanEntropy);

            
            // -------------------------- checking for successfull learning, fixating Reached!!!  -------------
            // after log file to save the final condition that brought success
            if(statevalueparam == GOALSTATE) {
                printf("SUCCESS IN FIXATING!!!!!!!!!!!! \n");
                printf("SUCCESS IN FIXATING!!!!!!!!!!!! \n");
                printf("SUCCESS IN FIXATING!!!!!!!!!!!! \n");
                fprintf(logFile,"SUCCESS IN FIXATING!!!!!!!!!!!! \n");
                countSucc++;
                
                state_now    = 0; //move to null state right after the success in fixating
                // resetting payoff and j term
                totalPayoff  = 0;
                iter         = 0;
                jiter        = 1;
                ap->setWaitType("ant");
            }         


            
            //printf("check after reset state; state_now %d action_now %d \n", state_now, action_now);
            


            // awaking action selection
            //printf("enabling back again the action selection \n");
            mutexStateTransition.wait();
            stateTransition = false;
            mutexStateTransition.post();
            //printf("enabled back again the action selection \n");
            
                        
        } break;
        case COMMAND_VOCAB_ACT :{
            //printf("new action update arrived %f %f \n", arg->get(1).asFloat32(), arg->get(2).asFloat32());
            //yarp::sig::Vector action(8);
            //action.zero();
            //int a = (int) arg->get(1).asFloat32();
            
            //extracting the allowTransition matrix
            yarp::sig::Vector a(7);
            a(0) = arg->get(1).asFloat32();
            a(1) = arg->get(2).asFloat32();
            a(2) = arg->get(3).asFloat32();
            a(3) = arg->get(4).asFloat32();
            a(4) = arg->get(5).asFloat32();
            a(5) = arg->get(6).asFloat32();
            a(6) = arg->get(7).asFloat32();
            
            // if it is not learning the action is trigger by these lines
            int i = 0;
            while (( a(i) == 0 ) && (i < 7)) {
                i++;
            }
            
            printf("Learning: command action %d \n", i);
            if (ap->isLearning() == false) {
                
                logAction(i);
            }
                       
        } break;
        default: {
            printf("Command not recognized \n");
        }break;
            
        }                
    }
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
    printf(" ------------------- state_now %9d ----------- \n", state_now);
}


void oculomotorController::interrupt(){
    printf("------------------------------- oculomotorController::threadRelease() : interrupt threads \n");
    inCommandPort.interrupt();
    //entImgPort.interrupt();
    printf("------------------------------- oculomotorController::threadRelease() : interrupt threads \n");
}


void oculomotorController::threadRelease() {
    printf("------------------------------- oculomotorController::threadRelease() : stopping threads \n");
    
    double t;
    idle = true;
    //closing ports
    
    //inCommandPort.interrupt();

    //   inCommandPort.close();
    
    //entImgPort.close();
    
    //stopping thread
    //if(0 != tp) {
    //    tp->stop();
    //}

    printf("oculomotorController::threadRelease() : about to stop outingThread \n");
    //ot->stop();
 
    //-------------------------------------------------------
    // TODO: trying to correcly close the files
    //closing files
    printf("closing the files... \n");
    
    //fclose(logFile);
    //fclose(logState);
    
    //fclose(PsaFile);
    //fclose(qualityFile);
    //fclose(rewardFile);
    printf("correctly closed the logFile \n");
    //--------------------------------------------------------

    
       
    // --- saving transition matrix ----------    
    PsaFile = fopen(psaFilePath.c_str(),"w+");
    printf("saving updates in Psa \n");
   
    if(0 == Psa) {
        printf("pointer to Psa null \n");
    }
    
    //double* valPsa = Psa->data();

    
    for(int row = 0; row < NUMSTATE * NUMACTION; row++ ) {
        for(int col = 0; col < NUMSTATE ; col++) {
            t = Psa->operator() (row,col);
            //if(t > 0.1) printf("changed value from 0.09 to %f \n", t);
            fprintf(PsaFile,"%f ",t);
            //valPsa++;
        }
        fprintf(PsaFile,"\n");
    }
    printf("correcly saved the PsaFile \n");
    
    // --- saving value function ----------
    
    qualityFile = fopen(qualityFilePath.c_str(),"w+");
    printf("saving updates in value \n");
    //if(0 == Q) {
    //    printf("pointer to value null \n");
    //}
    //double* valQ = Q->data();
    
    for(int row = 0; row < NUMSTATE; row++ ) {
        for(int col = 0; col < NUMACTION; col++) {
            t = Q->operator()(row,col);
            //printf("changed (%d,%d),%f \n",row, col,t);
            fprintf(qualityFile,"%f ",t);
            //valQ++;
        }
        fprintf(qualityFile,"\n");
    }
    printf("correctly closed the quality file \n");
    
    
    // --- saving reward function ----------
    rewardFile = fopen(rewardFilePath.c_str(),"w+");
    //printf("saving updates in value \n");
    //if(0 == V) {
    //    printf("pointer to value null \n");
    //}
    //double* valReward = rewardStateAction->data();
    
    for(int row = 0; row < NUMSTATE; row++ ) {
        for(int col = 0; col < NUMACTION; col++) {
            t = rewardStateAction->operator() (row,col);
            //if(t > 0.1) printf("changed value from 0.09 to %f \n", t);
            fprintf(rewardFile,"%f ",t);
            //valReward++;
        }
        fprintf(rewardFile,"\n");
    }
    printf("correctly saved the reward function \n");
    
    //printf("deleting the matrices \n");
    // delete Q;
    //delete V;
    //delete A;
    // delete P;

    printf("--------------------------- OculomotorController::threadRelease success \n  ");
}


