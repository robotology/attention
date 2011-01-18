// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file gazeArbiterModule.h
 * @brief A module that starts the Observer and Observable(Observables) necessary to prioritise the gaze requests
 */

#ifndef _GAZE_ARBITER_MODULE_H_
#define _GAZE_ARBITER_MODULE_H_

/** 
 *
 * \defgroup icub_gazeArbiter gazeArbiter
 * @ingroup icub_logPolarAttentionSystem
 *
 * This is a module that creates the infrastructure Observer-Observables in order to handle a state machine for gaze control.
 * 
 *
 * \section Description
 * This machine swaps between state and every state represent a different vision behaviour (saccade, vergence, tracking, ect)
 * Every behaviour has a particular priority and the controller of this machine has to take the priority into account and send the correct output to the
 * kinematic control of the gaze.
 * The Observables receive gaze request from the lower level and report these to the Observer


 * \section lib_sec Libraries
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c gazeArbiter.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c logpolarAttention/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c gazeArbiter \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 *
 *
 * <b>Configuration File Parameters</b>
 *
 * The following key-value pairs can be specified as parameters in the configuration file 
 * (they can also be specified as command-line parameters if you so wish). 
 * The value part can be changed to suit your needs; the default values are shown below. 
 *   
 *
 * 
 * \section portsa_sec Ports Accessed
 * /iKinGazeCtrl
 *                          
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /gazeArbiter \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /visualFilter
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       

 *
 * <b>Output ports</b>
 *
 *  - \c /gazeArbiter \n
 *    see above
 *
 * <b>Port types</b>
 *
 *
 * \section in_files_sec Input Data Files
 *
 * None
 *
 * \section out_data_sec Output Data Files
 *
 * None
 *
 * \section conf_file_sec Configuration Files
 *
 * \c gazeArbiter.ini  in \c $ICUB_ROOT/app/logpolarAttention/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>gazeArbiter --name gazeArbiter --context logpolarAttention/conf --from gazeArbiter.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2010 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/colourSaliency/include/iCub/colourSaliencyModule.h
 * 
 */

/* CHANGE LOG
 * 21/12/10 : vergence only after the robot has kept the fixation point with the cyclopic eye   @author Rea
 * 04/01/11 : changed the algorithm that translates the vergence event into a saccadic event    @author Rea 
 * 04/01/11 : added flag that discriminates between vergence after a mono saccadic event        @author Rea 
 * 06/01/11 : added new check to avoid situation in which the saccadic event stuck              @author Rea
 * 10/01/11 : changed the fixation command after monoLeft from absolute to monoLeft fixed image @author Rea
 */

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <iCub/gazeArbiterThread.h>
#include <iCub/gazeCollectorThread.h>


class gazeArbiterModule:public yarp::os::RFModule {
    std::string moduleName;                     //name of the module (rootname of ports)
    std::string robotName;                      //name of the robot
    std::string robotPortName;                  //reference to the head of the robot
    std::string handlerPortName;                //name of the handler port (comunication with respond function)
    int ratethread;                             //time constant for ratethread

    yarp::os::Port handlerPort;                 // a port to handle messages 

    gazeArbiterThread* arbiter;               //agent that sends commands to the gaze interface
    gazeCollectorThread* collector;              //agent that collects commands from the lower level

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};


#endif // __GAZE_ARBITER_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

