// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2016  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Fabio Vannucci
  * email: fabio.vannucci@iit.it
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
 * @file skinProcessorModule.h
 * @brief Simple module as tutorial.
 */

#ifndef _SKINPROCESSOR_MODULE_H_
#define _SKINPROCESSOR_MODULE_H_

/** 
 *
 * This is a module that receives input from the skin and sends messages as a string in output on a yarp port
 * 
 * 
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP.
 *  OpenCV
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c skinProcessor.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c skinProcessor/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c skinProcessor \n 
 *   specifies the name of the tutorialThread (used to form the stem of tutorialThread port names)  
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
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *  
 * - None
 *
 * <b>Output ports</b>
 *
 *  - \c /skinProcessor \n
 *    see above
 *
 *  - \c /skinProcessor/skinTouch:o \n
 *
 * <b>Port types</b>
 *
 * The functional specification only names the ports to be used to communicate with the tutorialThread 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c yarp::os::BufferedPort<yarp::os::Bottle> outputPort; \n       
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
 * \c 
 * \c 
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>skinProcessorThread --name skinProcessorThread --context skinProcessorThread/conf --from skinProcessorThread.ini --robot icub</tt>
 *
 * \author Fabio Vannucci
 *
 * Copyright (C) 2016 Robotics Brain and Cognitive Science \n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/contrib/src/morphoGen/src/tutorialThread/include/iCub/tutorialThread.h
 * 
 */




#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/PeriodicThread.h>
//#include <iCub/attention/commandDictionary.h>
 
//within project includes  
#include <iCub/skinProcessorRateThread.h>


class skinProcessorModule:public yarp::os::RFModule {

    
    std::string moduleName;                  // name of the module
    std::string robotName;                   // name of the robot 
    std::string robotPortName;               // name of robot port
    std::string handlerPortName;             // name of handler port
    std::string configFile;                  // name of the configFile that the resource Finder will seek
    
    yarp::os::Port handlerPort;              // a port to handle messages 
    /*  */
    skinProcessorRateThread *rThread;                 // pointer to a new thread to be created and started in configure() and stopped in close()

    yarp::os::Semaphore respondLock;         // check in the case of the respond function


public:
    /**
    *  configure all the tutorial parameters and return true if successful
    * @param rf reference to the resource finder
    * @return flag for the success
    */
    bool configure(yarp::os::ResourceFinder &rf); 
   
    /**
    *  interrupt, e.g., the ports 
    */
    bool interruptModule();                    

    /**
    *  close and shut down the tutorial
    */
    bool close();

    /**
    *  to respond through rpc port
    * @param command reference to bottle given to rpc port of module, alongwith parameters
    * @param reply reference to bottle returned by the rpc port in response to command
    * @return bool flag for the success of response else termination of module
    */
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);

    /**
    *  unimplemented
    */
    double getPeriod();

    /**
    *  unimplemented
    */ 
    bool updateModule();
};


#endif // __R_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

