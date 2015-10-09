// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
  * Copyright (C)2015  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file handProfilerThread.cpp
 * @brief Implementation of the handProfiler thread (see handProfilerThread.h).
 */

#include <iCub/handProfilerThread.h>
#include <cstring>

#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     30.0    // [deg]

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

handProfilerThread::handProfilerThread(): RateThread(100) {
    robot = "icub"; 
    // we wanna raise an event each time the arm is at 20%
    // of the trajectory (or 70% far from the target)
    cartesianEventParameters.type="motion-ongoing";
    cartesianEventParameters.motionOngoingCheckPoint=0.2;       
}


handProfilerThread::handProfilerThread(string _robot, string _configFile): RateThread(100){
    robot = _robot;
    configFile = _configFile;
    // we wanna raise an event each time the arm is at 20%
    // of the trajectory (or 70% far from the target)
    cartesianEventParameters.type="motion-ongoing";
    cartesianEventParameters.motionOngoingCheckPoint=0.2;
}



handProfilerThread::~handProfilerThread() {
    // do nothing
}


bool handProfilerThread::threadInit() {
    
    /* open ports */ 

    Property option("(device cartesiancontrollerclient)");
    option.put("remote","/icub/cartesianController/left_arm");
    option.put("local","/handProfiler/left_arm");

    if (!client.open(option)) {
                return false;
    }
    
    // open the view
    client.view(icart);
    
    // latch the controller context in order to preserve
    // it after closing the module
    // the context contains the dofs status, the tracking mode,
    // the resting positions, the limits and so on.
    icart->storeContext(&startup_context_id);
    
    // set trajectory time
    icart->setTrajTime(1.0);

    // get the torso dofs
    Vector newDof, curDof;
    icart->getDOF(curDof);
    newDof=curDof;

    // enable the torso yaw and pitch
    // disable the torso roll
    newDof[0]=1;
    newDof[1]=0;
    newDof[2]=1;

    // impose some restriction on the torso pitch
    limitTorsoPitch();

    // send the request for dofs reconfiguration
    icart->setDOF(newDof,curDof);

    // print out some info about the controller
    Bottle info;
    icart->getInfo(info);
    fprintf(stdout,"info = %s\n",info.toString().c_str());

    // register the event, attaching the callback
    icart->registerEvent(*this);

    xd.resize(3);
    od.resize(4);

    yInfo("handProfiler thread correctly started");
    
    return true;
}

void handProfilerThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string handProfilerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void handProfilerThread::setInputPortName(string InpPort) {

}

void handProfilerThread::run() {    
    t=Time::now();

    generateTarget();

    // go to the target :)
    // (in streaming)
    icart->goToPose(xd,od);

    // some verbosity
    printStatus();             
}

void handProfilerThread::generateTarget() {   
    // translational target part: a circular trajectory
    // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
    // and frequency 0.1 Hz
    xd[0]=-0.3;
    xd[1]=-0.1;//+0.1*cos(2.0*M_PI*0.1*(t-t0));
    xd[2]=+0.1;//+0.1*sin(2.0*M_PI*0.1*(t-t0));            
            
    // we keep the orientation of the left arm constant:
    // we want the middle finger to point forward (end-effector x-axis)
    // with the palm turned down (end-effector y-axis points leftward);
    // to achieve that it is enough to rotate the root frame of pi around z-axis
    od[0] = 0.0; od[1] = 0.0; od[2] = 1.0; od[3] = M_PI;
}

void handProfilerThread::limitTorsoPitch() {
    int axis=0; // pitch joint
    double min, max;

    // sometimes it may be helpful to reduce
    // the range of variability of the joints;
    // for example here we don't want the torso
    // to lean out more than 30 degrees forward

    // we keep the lower limit
    icart->getLimits(axis,&min,&max);
    icart->setLimits(axis,min,MAX_TORSO_PITCH);
}

void handProfilerThread::processing() {
}

void handProfilerThread::threadRelease() {
    // we require an immediate stop
    // before closing the client for safety reason
    icart->stopControl();
    // it's a good rule to restore the controller
    // context as it was before opening the module
    icart->restoreContext(startup_context_id);
    client.close();   

    yInfo("success in thread release");
}

void handProfilerThread::printStatus() {        
    if (t-t1>=PRINT_STATUS_PER) {
        Vector x,o,xdhat,odhat,qdhat;

        // we get the current arm pose in the
        // operational space
        icart->getPose(x,o);

        // we get the final destination of the arm
        // as found by the solver: it differs a bit
        // from the desired pose according to the tolerances
        icart->getDesired(xdhat,odhat,qdhat);

        double e_x=norm(xdhat-x);
        double e_o=norm(odhat-o);

        fprintf(stdout,"+++++++++\n");
        fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
        fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
        fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
        fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
        fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
        fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
        fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
        fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
        fprintf(stdout,"---------\n\n");

        t1=t;
    }
}

void handProfilerThread::onStop() {
}

