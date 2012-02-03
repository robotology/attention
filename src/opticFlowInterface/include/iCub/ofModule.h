// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/** 
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
 * @file ofModule.h
 * @brief A module that starts the Observer and Observable(Observables) necessary to prioritise the gaze requests
 */

#ifndef _OPTIC_FLOW_MODULE_H_
#define _OPTIC_FLOW_MODULE_H_

/** 
 *
 * \defgroup icub_gazeArbiter gazeArbiter
 * @ingroup icub_logpolarAttention
 *
 * This is a module that operates as interface between any module that extract optic flow and the attention prioriteser
 * 
 *
 * \section Description
 * The attention prioritiser selects action states and every state represent a different vision behaviour (saccade, vergence, tracking, smooth pursuit ect)
 * Concerning the smooth pursuit information about the caracteristic of themotion (direction and magnitude) has to be known. 
 * The interface provides a mean for collecting spatial information of the flow. On the other hand the module replies from queries of optic flow caracteristics.
 * The  Observables receive gaze request from the lower level and report these to the Observer, when a new information is updated from the observer the module updates the spatial memory
 *
 * \section lib_sec Libraries
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 %* 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c opticflowInterface.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c logpolarAttention/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c opticflowInterface \n 
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
 * - \c xmax \c -0.2 \n
 *   max allowed position of the fixation point ( x axis )
 *
 * - \c xmin \c -10.0 \n
 *   min allowed position of the fixation point ( x axis )
 *
 * - \c ymax \c 0.3 \n
 *   max allowed position of the fixation point ( y axis )
 *
 * - \c ymin \c -0.3 \n
 *   max allowed position of the fixation point ( y axis )
 *
 * - \c zmax \c 0.9 \n
 *   max allowed position of the fixation point ( y axis )
 *
 * - \c zmin \c -0.3 \n
 *   max allowed position of the fixation point ( y axis )
 *
 * - \c xoffset \c 0.0 \n
 *   max allowed position of the fixation point ( y axis )
 * 
 * - \c yoffset \c 0.0 \n
 *   max allowed position of the fixation point ( y axis )
 * 
 * - \c zoffset \c 0.0 \n
 *   max allowed position of the fixation point ( y axis )
 *
 * - \c onWings \c 0 \n
 *   1\0 when the camera do\don`t mount on the head
 *
 * - \c mode \c standard \n
 *   onWings\onDvs when the camera considered is on the head or on dvs
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
 *  - \c /opticflowInterface \n
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
 *  - \c /opticflowInterface/matchTracker/img:i
 *      port where the input image for imageTracking is sent
 *
 * <b>Output ports</b>
 *
 *  - \c /opticflowInterface \n
 *    see above
 * 
 *  - \c /opticflowInterface/status:o 
 *    port where the status of the controller is communicated
 *
 *  - \c /opticflowInterface/matchTracker/img:o
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
 * \c opticflowInterface.ini  in \c $ICUB_ROOT/app/logpolarAttention/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>opticflowInterface --name opticflowInterface --context logpolarAttention/conf --from opticflowInterface.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2010 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/colourSaliency/include/iCub/colourSaliencyModule.h
 * 
 */

/** \section change_log CHANGE LOG
 * 03/02/12 : the module is created                                                      author: Rea \n
*/

#define COMMAND_VOCAB_SET           VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET           VOCAB3('g','e','t')
#define COMMAND_VOCAB_SUSPEND       VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME        VOCAB3('r','e','s')
#define COMMAND_VOCAB_QUIT          VOCAB4('q','u','i','t')
#define COMMAND_VOCAB_OK            VOCAB2('o','k')
#define COMMAND_VOCAB_HELP          VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_FAILED        VOCAB4('f','a','i','l')


#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

//within project includes
#include <iCub/ofThread.h>
#include <iCub/ofCollector.h>


class ofModule:public yarp::os::RFModule {
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
    yarp::os::Semaphore respondLock;            // to lock updating through respond 
    ofThread* arbiter;                          //agent that sends commands to the gaze interface
    ofCollector* collector;                     //agent that collects commands from the lower level

public:
    bool   configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool   interruptModule();                       // interrupt, e.g., the ports 
    bool   close();                                 // close and shut down the module
    bool   respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool   updateModule();
    double getPeriod();
};


#endif // __OPTIC_FLOW_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

