// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Shashank Pathak
 * email:   francesco.rea@iit.it, shashank.pathak@iit.it
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
 * @file earlyVisionModule.h
 * @brief Simple module that implements efficient visual feature maps extraction;
 */

#ifndef _EARLY_VISION_MODULE_H_
#define _EARLY_VISION_MODULE_H_

/** 
 *
 * \defgroup icub_earlyVision earlyVision
 * @ingroup icub_logpolarAttention
 *
 * This is a module that applies various bio-inspired (early visual feature cues) transformations on the input image:
 *
 * 1. extract color planes R,G,B,Y \n
 * 2. extract color opponency maps \n  
 * 3. computes YUV (chrominance) maps based on RGB colours \n
 * 4. extract orientation applying Kirsch operator over intensity \n
 * 5. extract edges based on sobel operator applied over intensity \n
 * where (1,2),(3,4) and 5 are done by earlyVision thread, chrominance thread and edges thread respectively. These thread \n
 * have different frequencies (motivated from biological observation in primate vision). \n
 *
 *
 * \image html visualAttentionBUTD.jpg 
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
 * - \c from \c earlyVision.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c earlyVision/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c earlyVision \n 
 *   specifies the name of the module (used to form the stem of module port names)  
 *
 * - \c robot \c icub \n 
 *   specifies the name of the robot (used to form the root of robot port names)
 * 
 * - \c whorizontal \c 2.0 \n
 *   specifies the value for the weight in the combination of orientation
 *
 * - \c wvertical \c 2.0 \n
 *   specifies the value for the weight in the combination of orientation
 *
 * - \c w45degrees \c 2.0 \n
 *   specifies the value for the weight in the combination of orientation
 *
 * - \c wM45degrees \c 2.0 \n
 *   specifies the value for the weight in the combination of orientation
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
 *  - \c /earlyVision \n
 *    This port is used to change the parameters of the module at run time or stop the module. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other modules but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /earlyVision
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /earlyVision/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /earlyVision \n
 *    see above
 *
 *  - \c /earlyVision/image:o \n
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
 * \c earlyVision.ini  in \c $ICUB_ROOT/app/earlyVision/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/earlyVision/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>earlyVision --name earlyVision --context earlyVision/conf --from earlyVision.ini --robot icub</tt>
 *
 * \author Rea Francesco, Shashank Pathak
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/contrib/src/logPolarAttention/src/earlyVision/include/iCub/earlyVisionModule.h
 * 
 *
 * \section change_log CHANGE LOG
 * 12/09/11 : added factor 2 for the combination of orientation and introduced parameters for weights configuration        author : Rea \n
 * 15/02/12 : added few function to extract value of orientatio feature maps valuated in fovea                             author : Rea \n
 * 01/03/12 : added response function for chrominace feature map in fovea                                                  author : Rea \n           
 * 01/03/12 : introduced the reinforcement object footprint (as collection of features)
 */

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
 
//within project includes  
#include <iCub/earlyVisionThread.h>


//defining Vocabs
#ifndef _VOCAB_EARLY_VISION
#define _VOCAB_EARLY_VISION


const int32_t COMMAND_VOCAB_HELP     = yarp::os::createVocab32('h','e','l','p');
const int32_t COMMAND_VOCAB_QUIT     = yarp::os::createVocab32('q','u','i','t');
const int32_t COMMAND_VOCAB_FAILED   = yarp::os::createVocab32('f','a','i','l');

const int32_t COMMAND_VOCAB_OK       = yarp::os::createVocab32('o','k');
const int32_t COMMAND_VOCAB_P0       = yarp::os::createVocab32('p','0');

const int32_t COMMAND_VOCAB_SET      = yarp::os::createVocab32('s','e','t');
const int32_t COMMAND_VOCAB_GET      = yarp::os::createVocab32('g','e','t');
const int32_t COMMAND_VOCAB_SUSPEND  = yarp::os::createVocab32('s','u','s');
const int32_t COMMAND_VOCAB_RESUME   = yarp::os::createVocab32('r','e','s');
const int32_t COMMAND_VOCAB_HOR      = yarp::os::createVocab32('h','o','r');
const int32_t COMMAND_VOCAB_VER      = yarp::os::createVocab32('v','e','r');
const int32_t COMMAND_VOCAB_45       = yarp::os::createVocab32('o','4','5');
const int32_t COMMAND_VOCAB_P45      = yarp::os::createVocab32('p','4','5');
const int32_t COMMAND_VOCAB_N45      = yarp::os::createVocab32('n','4','5');
const int32_t COMMAND_VOCAB_M45      = yarp::os::createVocab32('M','4','5');
const int32_t COMMAND_VOCAB_P90      = yarp::os::createVocab32('p','9','0');
const int32_t COMMAND_VOCAB_ORI      = yarp::os::createVocab32('o','r','i');
const int32_t COMMAND_VOCAB_CHR      = yarp::os::createVocab32('c','h','r');
const int32_t COMMAND_VOCAB_BRIGHT   = yarp::os::createVocab32('b','r','t');

const int32_t COMMAND_VOCAB_WEIGHT  = yarp::os::createVocab32('w');



const int32_t COMMAND_VOCAB_CHROME_THREAD  = yarp::os::createVocab32('c','h','r');
const int32_t COMMAND_VOCAB_EDGES_THREAD   = yarp::os::createVocab32('e','d','g');
#endif



class earlyVisionModule:public yarp::os::RFModule
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
   earlyVisionThread *evThread;
   yarp::os::Semaphore respondLock; // to lock updating through respond 
   

public:
   bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
   bool interruptModule();                       // interrupt, e.g., the ports 
   bool close();                                 // close and shut down the module
   bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
   double getPeriod(); 
   bool updateModule();
   
};


#endif // __EARLY_VISION_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

