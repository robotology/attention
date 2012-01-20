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
 * @file logOFModule.h
 * @brief Simple module that implements efficient computation of the opticflow in logpolar images;
 */

#ifndef _LOG_OF_MODULE_H_
#define _LOG_OF_MODULE_H_

/** 
 *
 * \defgroup icub_logOpticFlow logOpticFlow
 * @ingroup icub_logpolarAttention
 *
 * This is a module that computes optic flow in log polar images
 *
 * The computation is based on the paper Kruger (Estimation optic Flow in the log-polar plane) and a logpolar 
 * mapping Berton, Metta ( A brief introduction on log polar mapping)
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
 * - \c from \c logOpticFlow.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c logOpticFlow/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c logOpticFlow \n 
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
 *  - \c /logOpticFlow \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /logOpticFlow
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /logOpticFlow/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /logOpticFlow \n
 *    see above
 *
 *  - \c /logOpticFlow/image:o \n
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
 * \c logOpticFlow.ini  in \c $ICUB_ROOT/app/logOpticFlow/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/logOpticFlow/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>logOpticFlow --name logOpticFlow --context logOpticFlow/conf --from logOpticFlow.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/modules/logOpticFlow/include/iCub/logOpticFlow.h
 * 
 */


/**
 * \section changel_log CHANGE LOG
 * 13/01/12 : creation date                                                  \author Rea \n
 */

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
 
//within project includes  
#include <iCub/logOFThread.h>


//defining Vocabs
#ifndef _VOCAB_EARLY_VISION
#define _VOCAB_EARLY_VISION
#define COMMAND_VOCAB_SET           VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET           VOCAB3('g','e','t')
#define COMMAND_VOCAB_HELP          VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_QUIT          VOCAB4('q','u','i','t')
#define COMMAND_VOCAB_FAILED        VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_OK            VOCAB2('o','k')
#define COMMAND_VOCAB_SUSPEND       VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME        VOCAB3('r','e','s')
#define COMMAND_VOCAB_HOR           VOCAB3('h','o','r')
#define COMMAND_VOCAB_VER           VOCAB3('v','e','r')
#define COMMAND_VOCAB_45            VOCAB3('o','4','5')
#define COMMAND_VOCAB_M45           VOCAB4('o','M','4','5')
#define COMMAND_VOCAB_GHOR          VOCAB4('g','h','o','r')
#define COMMAND_VOCAB_GVER          VOCAB4('g','v','e','r')
#define COMMAND_VOCAB_BRIGHT        VOCAB3('b','r','t')
#define COMMAND_VOCAB_WEIGHT        VOCAB1('w')
#define COMMAND_VOCAB_CHROME_THREAD VOCAB3('c','h','r')
#define COMMAND_VOCAB_EDGES_THREAD  VOCAB3('e','d','g')

#endif



class logOFModule:public yarp::os::RFModule
{
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
   logOFThread *evThread;
   yarp::os::Semaphore respondLock; // to lock updating through respond 
   

public:
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   double getPeriod(); 
   bool updateModule();
   
};


#endif // __LOG_OF_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

