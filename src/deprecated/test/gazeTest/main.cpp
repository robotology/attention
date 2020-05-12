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
 * @file main.cpp
 * @brief Implementation of the testGaze
 */


// #include <math.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <iCub/ctrl/kalman.h>
#include <iCub/attention/predModels.h>
#include <iCub/attention/evalThread.h>
// #include <yarp/os/Port.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/all.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;

//YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char *argv[]) {
    
    //YARP_REGISTER_DEVICES(icubmod)
        
    // Open the network
    Network yarp;
    
    
    yarp::os::Network::init();   
    Time::turboBoost();

    yarp::os::BufferedPort<Bottle> bottlePort;
    bottlePort.open("/testGaze/bottle:o");
    
    
    //initializing gazecontrollerclient
    bool noGazeController = true;
    printf("initialising gazeControllerClient \n");
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    std::string localCon("/client/gaze");
    localCon.append("/gazeTest");
    option.put("local",localCon.c_str());
    
    yarp::dev::PolyDriver* clientGazeCtrl = new yarp::dev::PolyDriver();
    clientGazeCtrl->open(option);
    
    
    
    yarp::dev::IGazeControl* igaze = NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
       noGazeController = false;
    }
    else{
        noGazeController = true;
        
        //return false;
    }
        

    if(!noGazeController) {
        printf("storing original context \n");
        int originalContext = 0;
        igaze->storeContext(&originalContext);
    }
    
    Vector px(2);
    px(0)  = 176.079;
    px(1)  = 129.139;

    
    printf("starting testGaze \n");
    while(true){
        if(!noGazeController){
            igaze->lookAtMonoPixel(0,px,0.5);    // look!
        }
        Time::delay(1.0);
        Bottle& b = bottlePort.prepare();
        b.clear();
        double dTime = Time::now();
        b.addDouble(dTime);
        bottlePort.write();
    }


     
     Network::fini();
    
}
