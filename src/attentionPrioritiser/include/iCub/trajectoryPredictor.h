// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file trajectoryPredictor.h
 * @brief thread that given as input the image of the region to be track, extract the centroid and determines velocity components and stopping position
 */

#ifndef _OCULOMOTOR_CONTROLLER_H_
#define _OCULOMOTOR_CONTROLLER_H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <string>

//within project includes
#include <iCub/observer.h>
#include <iCub/observable.h>

class trajectoryPredictor : public observable{
private:
    
    std::string name;       // rootname of all the ports opened by this thread
    yarp::os::BufferedPort<yarp::os::Bottle> inCommandPort;     //port where all the low level commands are sent

public:
    /**
    * default constructor
    */
    trajectoryPredictor();

    /**
     * destructor
     */
    ~trajectoryPredictor();

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

};

#endif  //_OCULOMOTOR_CONTROLLER_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

