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
 * @file earlyMotionModule.h
 * @brief Simple module that implements logPolar detection of motion
 * It is light and naive implementation.
 */

#ifndef _EARLY_MOTION_MODULE_H_
#define _EARLY_MOTION_MODULE_H_

/** 
 *
 * \defgroup icub_earlyMotion earlyMotion
 * @ingroup icub_logpolarAttention
 *
 * This module extract motion from the logpolar input image in the most trivial way
 *
 * 
 * \section lib_sec Libraries
 *
 * YARP.
 *
 * \section parameters_sec Parameters
 * 
 * <b>Command-line Parameters</b> 
 * 
 * The following key-value pairs can be specified as command-line parameters by prefixing \c -- to the key 
 * (e.g. \c --from file.ini. The value part can be changed to suit your needs; the default values are shown below. 
 *
 * - \c from \c earlyMotion.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c earlyMotion/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c earlyMotion \n 
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
 * 
 * - None
 *                      
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /earlyMotion \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /earlyMotion
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /earlyMotion/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /earlyMotion \n
 *    see above
 *
 *  - \c /earlyMotion/image:o \n
 *
 * <b>Port types</b>
 *
 * The functional specification only names the ports to be used to communicate with the module 
 * but doesn't say anything about the data transmitted on the ports. This is defined by the following code. 
 *
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myInputPort; \n 
 * \c BufferedPort<ImageOf<PixelRgb> >   \c myOutputPort;       
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
 * \c earlyMotion.ini  in \c $ICUB_ROOT/app/earlyMotion/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/earlyMotion/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>earlyMotion --name earlyMotion --context logpolarAttention/conf --from earlyMotion.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/earlyMotion/include/iCub/earlyMotion.h
 * 
 */

/**
 * CHANGE LOG
 * 08/08/11 : added a factor to compensate for logpolar noise in fovea                  @Rea
 */




#include <iostream>
#include <string>


#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>

#include <yarp/sig/all.h>
 
//within project includes  
#include <iCub/earlyMotionThread.h>



class earlyMotionModule:public yarp::os::RFModule {

    
    /* module parameters */
    std::string moduleName;
    std::string robotName; 
    std::string robotPortName;  
    std::string inputPortName;
    std::string outputPortName;  
    std::string handlerPortName;
    std::string cameraConfigFilename;
    
    yarp::os::Port handlerPort;      // a port to handle messages 
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    earlyMotionThread *emThread;
    
public:
    

    
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
 
    bool updateModule();
};


#endif // __EARLY_MOTION_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

