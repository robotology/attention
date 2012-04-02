// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file attPrioritiserModule.h
 * @brief A module that starts the Observer and Observable(Observables) necessary to prioritise the gaze requests
 */

#ifndef _ATT_PRIORITISER_MODULE_H_
#define _ATT_PRIORITISER_MODULE_H_

/** 
 *
 * \defgroup icub_attentionalPrioritiser attentionalPrioritiser
 * @ingroup icub_logpolarAttention
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
 * - \c from \c attentionalPrioritiser.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c logpolarAttention/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c attentionalPrioritiser \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 * 
 * - \c width \c 320 \n
 *   specifies the dimension width of the input image
 * 
 * - \c height \c 240 \n
 *   specifies the dimension height of the input image
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
 *  - \c /attentionalPrioritiser \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *  -  \c suspend \n
 *  -  \c resume  \n
 *  -  \c seek    \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /visualFilter
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /attentionalPrioritiser/matchTracker/img:i
 *      port where the input image for imageTracking is sent
 *
 * <b>Output ports</b>
 *
 *  - \c /attentionalPrioritiser \n
 *    see above
 * 
 *  - \c /attentionalPrioritiser/status:o 
 *    port where the status of the controller is communicated
 *
 *  - \c /attentionalPrioritiser/matchTracker/img:o
 *      port where the result of the tracking is sent
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
 * \c attentionalPrioritiser.ini  in \c $ICUB_ROOT/app/logpolarAttention/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>attentionalPrioritiser --name attentionalPrioritiser --context logpolarAttention/conf --from attentionalPrioritiser.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/contrib/src/logpolarAttention/src/attentionPrioritiser/include/iCub/attPrioritiserModule.h
 * 
 */


/** 
 * /section  change_log CHANGE LOG:
 * 06/07/11 : created the module                                                                            author: Rea  \n
 * 07/07/11 : added the function that calculates the correlation between two log-polar images               author: Rea  \n
 * 11/07/11 : changed the cycle for exploring the image in the correlation function                         author: Rea  \n                                      
 * 19/08/11 : added a new feedback port to control earlier processes                                        author: Rea  \n
 * 07/10/11 : added the enable and disable commands for express saccades.                                   author: Rea  \n       
 * 26/10/11 : reduced all the temporal constants to 50ms                                                    author: Rea  \n  
 * 04/01/12 : added feedback commands to the low level visual attention                                     author: Rea  \n         
 * 15/02/12 : introduced top-down state for stable visual attention                                         author: Rea  \n
 * 18/02/12 : tested the top-down connection between earlyVision and saliencyBlobFinder                     author: Rea  \n
 * 24/02/12 : introduced extra values for command of relative vergence                                      author: Rea  \n
 * 15/03/12 : added two new classes : oculomotorController and trajectoryPredictor                          author: Rea  \n
 * 16/03/12 : implemented learning step and interaction with the attPrioritiserThread                       author: Rea  \n
 * 23/03/12 : added nofity process between prioritiser and controller                                       author: Rea  \n
*/



#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RpcServer.h>

//within project includes
#include <iCub/oculomotorController.h>
#include <iCub/attPrioritiserThread.h>
#include <iCub/prioCollectorThread.h>


// general command vocab's
#define COMMAND_VOCAB_HELP               VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SET                VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET                VOCAB3('g','e','t')
#define COMMAND_VOCAB_RUN                VOCAB3('r','u','n')
#define COMMAND_VOCAB_SUSPEND            VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME             VOCAB3('r','e','s')
#define COMMAND_VOCAB_FIX                VOCAB3('f','i','x')
#define COMMAND_VOCAB_IS                 VOCAB2('i','s')
#define COMMAND_VOCAB_OK                 VOCAB2('o','k')
#define COMMAND_VOCAB_FAILED             VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_SEEK               VOCAB4('s','e','e','k')
#define COMMAND_VOCAB_CENT               VOCAB4('c','e','n','t')


#define COMMAND_VOCAB_RED                VOCAB3('r','e','d')


class attPrioritiserModule:public yarp::os::RFModule {
    std::string moduleName;                     // name of the module (rootname of ports)
    std::string robotName;                      // name of the robot
    std::string robotPortName;                  // reference to the head of the robot
    std::string handlerPortName;                // name of the handler port (comunication with respond function)
    std::string configFile;                     // configuration file of cameras
    std::string mode;                           // string that indicates the modality of the mapping (if any)
    std::string configName;                     // name of the config file for camera that is going to be search
    int ratethread;                             // time constant for ratethread
    double xoffset;                             // offset for the 3D point along x
    double yoffset;                             // offset for the 3D point along y
    double zoffset;                             // offset for the 3D point along z
    double xmax, xmin;                          // limits for the allowed fixation point (x axis)
    double ymax, ymin;                          // limits for the allowed fixation point (y axis)
    double zmax, zmin;                          // limits for the allowed fixation point (z axis)
    double pitch;                               // desired angle for fixed pitch ( -1 not fixed angle)
    int onWings;
    int width, height;                          // parameter set by user dimensioning input image
    yarp::os::Port handlerPort;                 // a port to handle messages 
    yarp::os::RpcServer remoteProgPort;         // port associated with the remote programming
    
    yarp::os::Semaphore mutex;                  // semaphore for the respond function

    attPrioritiserThread* prioritiser;          //agent that sends commands to the gaze interface
    prioCollectorThread*  collector;            //agent that collects commands from the lower level
    oculomotorController* controller;           //agent that implements Q-learning process

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};


#endif // __ATT_PRIORITISER_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

