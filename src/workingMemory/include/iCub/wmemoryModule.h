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
 * @file wmemoryModule.h
 * @brief A module that makes storing (working)memory for the attentionSystem 
 */

#ifndef _WORKING_MEMORY_MODULE_H_
#define _WORKING_MEMORY_MODULE_H_

/** 
 *
 * \defgroup icub_wmemoryModule wmemoryModule
 * @ingroup icub_logpolarAttention
 *
 * This is a module that is in charge of saving relevant information into the working memory
 * 
 * 
 * 
 *
 * \section Description
 * to be completed
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
 * - \c from \c workingMemory.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c logpolarAttention \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c workingMemory \n 
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
 *   /<objectPropertiesCollector>/rpc the remote procedure call port used to send requests to the database and receive replies
 *                          
 * \section portsc_sec Ports Created
 *
 *  <b>Input ports</b>
 *
 *  - \c /workingMemory \n
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
 *<b>Input-Output ports</b>
 *  -  \c /workingMemory/database:o \n
        This port is used to send queries to the objectPropertiesCollector
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
 * \c workingMemory.ini  in \c $attentionInstall/logPolarAttention/conf \n
 * 
 * \section tested_os_sec Tested OS
 *
 * Windows, Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>workingMemory --name workingMemory --context logpolarAttention/conf --from workingMemory.ini --robot icub</tt>
 *
 * \author Rea Francesco
 *
 * Copyright (C) 2014 RBCS Robotics Brain and Cognitive Science\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * 
 */



#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/RpcServer.h>

//within project includes
#include <iCub/attention/commandDictionary.h>
#include <iCub/wmemoryThread.h>

using namespace attention::dictionary;


class wmemoryModule:public yarp::os::RFModule {
    std::string moduleName;                     // name of the module (rootname of ports)
    std::string robotName;                      // name of the robot
    std::string robotPortName;                  // reference to the head of the robot
    std::string handlerPortName;                // name of the handler port (comunication with respond function)
    int ratethread;                             // time constant for ratethread
    yarp::os::Semaphore mutex;                  // semaphore for the respond function
    yarp::os::RpcServer handlerPort;                 // a port to handle messages 
    
    wmemoryThread* wmThread;                    // workingMemoryThread for processing of the information

public:
    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
    


};


#endif // _WORKING_MEMORY_MODULE_H__
//----- end-of-file --- ( next line intentionally left blank ) ------------------

