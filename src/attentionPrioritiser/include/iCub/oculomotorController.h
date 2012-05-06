// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file oculomotorController.h
 * @brief RateThread which collects implements Q-learning algorithm for the definition of the best policy in different situation.
 *  The policy determines the best action given the actual state of the controller in order to reach the final goal stage.
 *  In this state the stimulus is under fixation and therefor at zero disparity
 */

#ifndef _OCULOMOTOR_CONTROLLER_H_
#define _OCULOMOTOR_CONTROLLER_H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/all.h>
#include <yarp/math/Rand.h>

#include <cstdio>
#include <iostream>
#include <string>


//within project includes
#include <iCub/observer.h>
#include <iCub/observable.h>
#include <iCub/attPrioritiserThread.h>
#include <iCub/trajectoryPredictor.h>

#define THRATE 10
#define NUMSTATE 11
#define NUMACTION 8




static const std::string stateList[11] =  {
    "null",           //0
    "predict",        //1        
    "fixStableOK",    //2
    "fixStableKO",    //3
    "trackOK",        //4
    "trackKO",        //5
    "anticipOK",      //6
    "anticipWait",    //7
    "vergenceK0",     //8
    "vergenceOK",     //9
    "fixating"        //10    
};

static const std::string actionList[8]  = {
    "reset",           //0
    "vergence",        //1
    "smoothPursuit",   //2
    "microSaccade",    //3
    "mediumSaccade",   //4
    "largeSaccade",    //5
    "expressSaccade",  //6
    "predict"          //7
};

/*
// reward for the particular state-action condition
// dimensionality state(11) x action (6)
static const double rewardStateAction[66] = {
    0.1,0.1,0.1,0.1,0.1,0.1,  // 0
        0.1,0.1,0.1,0.1,0.1,0.1,  // 1
        0.1,0.1,0.1,0.1,0.1,0.1,  // 2
        0.1,0.1,0.1,0.1,0.1,0.1,  // 3
        0.1,0.1,0.1,0.1,0.1,0.1,  // 4
        0.1,0.1,0.1,0.1,0.1,0.1,  // 5
        0.1,0.1,0.1,0.1,0.1,0.1,  // 6
        0.1,0.1,0.1,0.1,0.1,0.1,  // 7
        0.1,0.1,0.1,0.1,0.1,0.1,  // 8
        0.1,0.1,0.1,0.1,0.1,0.1,  // 9
        0.1,0.1,0.1,0.1,0.1,0.1,  // 10
};
*/


class outingThread : public yarp::os::RateThread {
    
private:
    double value;
    yarp::os::Semaphore mutex;                                  // semaphore for concurrent programming
    yarp::os::BufferedPort<yarp::os::Bottle> scopePort;         // port where all the totalPayoff is going to be sent
    std::string name;                                           // rootname of all the ports opened by this thread
    
public:
    outingThread():yarp::os::RateThread(THRATE){
        value = 0;
    };

    ~outingThread() {};
    
    
    /**
    * function that initialise the thread
    */
    bool threadInit() {
        scopePort.open(getName("/scope:o").c_str());
    };

    /**
    * function called when the thread is stopped
    */
    void threadRelease() {
        scopePort.close();
    };

    void setName(string str) {
        this->name=str;
        printf("name: %s", name.c_str());
    }
    
    std::string getName(const char* p) {
        string str(name);
        str.append(p);
        return str;
    }
    
    /**
    * function called every time constant defined by rateThread
    */
    void run() {
        
        //if(scopePort.getOutputCount()) {
        
        yarp::os::Bottle& scopeBottle = scopePort.prepare();
        scopeBottle.clear();
        scopeBottle.addDouble(value);
        scopePort.write();
        //}
    }; 

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt() {
        scopePort.interrupt();
    };
    
    void setValue(double v){mutex.wait(); value = v; mutex.post();};
};

class oculomotorController : public yarp::os::RateThread, public observer {
private:
    bool idle;                 // flag that regulates when the active part is executed
    bool firstCycle;           // flga that triggers the initialisation of active part
    bool firstCount;           // first count of the starting state
    bool stateTransition;      // synchronisation betweee actionSelection and stateTransition
    
    int count;                 // step counter of successful learning step
    int cUpdate;               // counter of observable updates
    int iter;                  // counter of any iteration in learning
    std::string name;          // rootname of all the ports opened by this thread
    
    yarp::os::Semaphore mutexStateTransition;                   // semaphore for controlling the state transition

    yarp::os::BufferedPort<yarp::os::Bottle> inCommandPort;     // port where all the low level commands are sent
   
    attPrioritiserThread* ap;                                   // reference to the attention prioritiser
    trajectoryPredictor*  tp;                                   // reference to the trajectory predictor
    outingThread*         ot;                                   // reference to the thread for plotting 
    
    yarp::sig::Matrix* rewardStateAction;  // reward coming from a particular combination of state and action dim:NUMSTATE x ACTION
    yarp::sig::Matrix* Psa;                // probability of transition from a couple <state,action> to a state dim:NUMSTATE * NUMACTION x NUMSTATE
    yarp::sig::Matrix* Q;                  // quality measure of a particular state across different actions dim: NUMSTATE x NUMACTION
    yarp::sig::Matrix  M;                  // intermediate computation matrix
    yarp::sig::Matrix* V;                  // value matrix max value of quality measure with reference to one state dim NUMACTION x 1
    yarp::sig::Matrix* A;                  // action that generates max value of quality measure with reference to one state dim NUMACTION x 1
    yarp::sig::Matrix* P;                  // copy quality measure of a particular state across different actions dim: NUMSTATE x NUMACTION

    const static double j    = 0.99;       // discount factor
    const static double alfa = 0.1;        // learning rate  
    
    double totalPayoff;                    // total payoff of the learning process
    double jiter;                          // cumulative j ^ iter 

    int state_prev;                        // state in which the controller was previously
    int state_now;                         // state of the controller now
    int action_now;                        // action performed in this step
    int state_next;                        // state in which the controller will end
    
    FILE* PsaFile;                         // file that contains the Probability of Transitions
    FILE* logFile;                         // log file for actions and states
    FILE* qualityFile;                       // memory of value function through runs
    FILE* rewardFile;                      // file that stores the reward function for state action
public:
    /**
    * default constructor
    */
    oculomotorController();

    /**
    * default constructor
    */
    oculomotorController(attPrioritiserThread *apt);

    /**
     * destructor
     */
    ~oculomotorController();

    /**
    * function that initialise the thread
    */
    bool threadInit();

    /**
    * function called when the thread is stopped
    */
    void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    void run(); 

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();

    /**
    * function which is automatically executed when the stop function of the thread is called
    */
    void onStop();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that change the idle flage
    * @param value true/false idle/!idle the active method of the class
    */
    void setIdle(bool value) {idle = value;};

    /**
    * function that defines what has to be done once any observeble interrupts
    * @param o observable that has just interrupted the observer
    * @param arg Bottle that has passed from the observable to the observer
    */
    void update(observable* o, yarp::os::Bottle * arg);

    /**
    * @brief function that performs Q-learning using a perfect random decision of the action to take
    * @return true if and only if the next state (selected) is the sink state
    * @param state_next next state where the controller ends up
    */
    bool randomWalk(int& state_next);

    /**
    * @brief function that performs Q-learning using the built policy.
    * @return true if and only if the next state (selected) is the sink state
    */
    bool policyWalk();

    /**
    * @brief one single step of the learning which consists in selecting a path from the initial state to the final state.
    */
    void learningStep();

    /**
     * given the state where tha actuator is supposed to go 
     * wait for the response command of the attPrioritiserThread;
     */
    void waitForActuator();

    /**
     * @brief function that enables a particular action in the actuator
     * The function runs a loop to check whether the action is performed within a timeout
     * @param state state that is going to be enabled
     * @retur returns whether the request has been successful
     */
    bool allowStateRequest(int state);
    
};




#endif  //_OCULOMOTOR_CONTROLLER_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

