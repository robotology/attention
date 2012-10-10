// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/** 
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
 * @ingroup icub_logpolarAttention
 *
 * This is a module that creates the infrastructure Observer-Observables in order to handle a state machine for gaze control.
 * 
 * 
 * \image html visualAttentionBUTD.jpg
 * 
 * \section Description
 * This machine swaps between state and every state represent a different vision behaviour (saccade, vergence, tracking, ect)
 * Every behaviour has a particular priority and the controller of this machine has to take the priority into account and send the correct output to the
 * kinematic control of the gaze.
 * The Observables receive gaze request from the lower level and report these to the Observer
 *
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
 *  - \c /gazeArbiter/matchTracker/img:i
 *      port where the input image for imageTracking is sent
 *
 * <b>Output ports</b>
 *
 *  - \c /gazeArbiter \n
 *    see above
 * 
 *  - \c /gazeArbiter/status:o 
 *    port where the status of the controller is communicated
 *
 *  - \c /gazeArbiter/matchTracker/img:o
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
 * This file can be edited at \c $ICUB_ROOT/contrib/src/logpolarAttention/src/gazeArbiter/include/iCub/gazeArbiterModule.h
 * 
 */

/** \section change_log CHANGE LOG
 * 21/12/10 : vergence only after the robot has kept the fixation point with the cyclopic eye       author: Rea \n
 * 04/01/11 : changed the algorithm that translates the vergence event into a saccadic event        author: Rea \n
 * 04/01/11 : added flag that discriminates between vergence after a mono saccadic event            author: Rea \n
 * 06/01/11 : added new check to avoid situation in which the saccadic event stuck                  author: Rea \n
 * 10/01/11 : changed the fixation command after monoLeft from absolute to monoLeft fixed image     author: Rea \n
 * 18/01/11 : introduced the right eye image as input of the vergence algorithm                     author: Rea \n
 * 26/01/11 : added resume/suspend and port for status communication                                author: Rea \n
 * 31/01/11 : checkpoint that controls whether there is a action performing and waits till not      author: Rea \n
 * 01/02/11 : added xoffset, yoffset and zoffset for 3D target                                      author: Rea \n
 * 30/02/11 : added new parameters for input image dimensioning                                     author: Rea \n
 * 30/02/11 : removed imprecision in the case the tracker does not initialise                       author: Rea \n
 * 24/04/11 : added fixed neck pitch option                                                         author: Rea \n
 * 28/03/11 : added the limits for the allowed fixation points in 3d space                          author: Rea \n
 * 29/03/11 : added new parameters to the config file and embedded previously added ones            author: Rea \n
 * 06/04/11 : added a new port for acquiring the template and sending it to the objectsColl.        author: Rea \n
 * 11/04/11 : introduced boolean flag for allowing refinement of the final position                 author: Rea \n
 * 11/04/11 : new limit for avoiding the robot to attend too close to the body                      author: Rea \n
 * 16/04/11 : moved the execution at the beginning of saccade and the table not vergence accompli   author: Rea \n
 * 16/04/11 : plotted the texture on the small pciture in the GUI                                   author: Rea \n
 * 16/04/11 : added original context restore                                                        author: Rea \n
 * 05/05/11 : added the mapping between the dvs camera and the original dimension in the eyes       author: Rea \n
 * 20/05/11 : put together the command for choosing between onDvs and onHead                        author: Rea \n
 * 03/06/11 : added new output image where the area around WTA is selected                          author: Rea \n
 * 09/06/11 : added the port in order to plot out the timing of the whole fixation deployment       author: Rea \n
 * 07/09/11 : corrected the values of the lookAtMonoPixel aligning it to change in iKinGaze         author: Rea \n 
 * 08/09/11 : added a counter for null vergence angle to integrate vergence commands                author: Rea \n
 * 10/09/11 : added the value -1 to write on the timing port when novel saccade on not accomplished author: Rea \n 
 * 05/10/11 : changed the name of the matchTracker to rootname of the module                        author: Rea \n              
 * 18/10/11 : changed the strings that indicates the vergence success to VER_ACC                    author: Rea \n                          
 * 20/10/11 : subtracted 1 to the y and x of the visual correction in saccadic event                author: Rea \n         
 * 26/10/11 : reduce all the temporal constant to 50ms. Trying to improve the response              author: Rea \n
 * 06/02/12 : added a novel thread for velocity tracking                                            author: Rea \n
 * 25/05/12 : added min Cumulative measure  in the tracker                                          author: Rea \n
 * 25/09/12 : introduced the periodic tracker thread                                                author: Rea \n
 * 02/10/12 : made the period tracker thread really work into the gazeArbiter system                author: Rea \n
 * 10/10/12 : made the periodTracker work with the matchTracker                                     author: Rea \n
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

    gazeArbiterThread* arbiter;                 //agent that sends commands to the gaze interface
    gazeCollectorThread* collector;             //agent that collects commands from the lower level

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    bool updateModule();
};


#endif // __GAZE_ARBITER_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

