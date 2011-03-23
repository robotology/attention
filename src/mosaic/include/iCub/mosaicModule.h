// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
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
 * @file mosaicModule.h
 * @brief Simple module that implements creation of mosaic (larger image) from original camera image
 */

#ifndef _MOSAIC_MODULE_H_
#define _MOSAIC_MODULE_H_

/** 
 *
 * \defgroup icub_mosaic mosaic
 * @ingroup icub_logpolarAttention
 *
 * This is a module that makes mosaic large image from multiple small images. As of now, the input image can be arbitrarily
 * placed on mosaic image, retaining earlier non-over-written part. Mosaic is originally black.
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
 * - \c from \c mosaic.ini \n 
 *   specifies the configuration file
 *
 * - \c context \c mosaic/conf \n
 *   specifies the sub-path from \c $ICUB_ROOT/icub/app to the configuration file
 *
 * - \c name \c mosaic \n 
 *   specifies the name of the mosaic (used to form the stem of mosaic port names)  
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
 *  - \c /mosaic \n
 *    This port is used to change the parameters of the mosaic at run time or stop the mosaic. \n
 *    The following commands are available
 * 
 *  -  \c help \n
 *  -  \c size \n
 *  -  \c place \n
 *  -  \c quit \n
 *
 *    Note that the name of this port mirrors whatever is provided by the \c --name parameter value
 *    The port is attached to the terminal so that you can type in commands and receive replies.
 *    The port can be used by other mosaics but also interactively by a user through the yarp rpc directive, viz.: \c yarp \c rpc \c /mosaic
 *    This opens a connection from a terminal to the port and allows the user to then type in commands and receive replies.
 *       
 *  - \c /mosaic/image:i \n
 *
 * <b>Output ports</b>
 *
 *  - \c /mosaic \n
 *    see above
 *
 *  - \c /mosaic/image:o \n
 *
 * <b>Port types</b>
 *
 * The functional specification only names the ports to be used to communicate with the mosaic 
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
 * \c mosaic.ini  in \c $ICUB_ROOT/app/mosaic/conf \n
 * \c icubEyes.ini  in \c $ICUB_ROOT/app/mosaic/conf
 * 
 * \section tested_os_sec Tested OS
 *
 * Linux
 *
 * \section example_sec Example Instantiation of the Module
 * 
 * <tt>mosaic --name mosaic --context mosaic/conf --from mosaic.ini --robot icub</tt>
 *
 * \author Shashank
 *
 * Copyright (C) 2011 RobotCub Consortium\n
 * CopyPolicy: Released under the terms of the GNU GPL v2.0.\n
 * This file can be edited at \c $ICUB_ROOT/main/src/mosaics/mosaic/include/iCub/mosaic.h
 * 
 */

#include <iostream>
#include <string>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
 
//within project includes  
#include <iCub/mosaicThread.h>

class mosaicModule:public yarp::os::RFModule {
    int width_orig, height_orig;             // dimension of the input image
    int width, height;                       // dimension of the mosaic image

    std::string moduleName;                  // name of the module
    std::string robotName;                   // name of the robot 
    std::string robotPortName;               // name of robot port
    std::string inputPortName;               // name of the input port
    std::string outputPortName;              // name of output port
    std::string handlerPortName;             // name of handler port
    
    yarp::os::Port handlerPort;      // a port to handle messages 
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    mosaicThread *mThread;

public:
    /**
    *  configure all the mosaic parameters and return true if successful
    * @param rf reference to the resource finder
    * @return flag for the success
    */
    bool configure(yarp::os::ResourceFinder &rf); 
   
    /**
    *  interrupt, e.g., the ports 
    */
    bool interruptModule();                    

    /**
    *  close and shut down the mosaic
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


#endif // __MOSAIC_MODULE_H__

//----- end-of-file --- ( next line intentionally left blank ) ------------------

