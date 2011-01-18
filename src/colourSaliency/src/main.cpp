// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
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
*
@ingroup icub_logPolarAttentionSystem
\defgroup icub_colourSaliency Colour Saliency

Compute colour saliency.

\section intro_sec Description
...

\section lib_sec Libraries
List here dependencies. Often these are just YARP libraries.

\section parameters_sec Parameters
Provide a comprehensive list of the parameters you can pass to the module. For example:
 
\section portsa_sec Ports Accessed
This is important. List here ports accessed by the module. This is useful to build a list of dependencies between modules.

\section portsc_sec Ports Created
Provide the list of ports created by the module. Separate them in input and output ports, specify expected data format.
 
\section conf_file_sec Configuration Files
If parameters to your module can be passed through a txt file, describe it here. 

For example:
The module requires a description of the robot through the parameter 
--file.


\section tested_os_sec Tested OS
Specify the operating systems on which the module was tested
Example:

Linux and Windows.

\section example_sec Example Instantiation of the Module
Provide here a typical example of use of your module.
Example:


\author Francesco Rea

Copyright (C) 2010 RobotCub Consortium

CopyPolicy: Released under the terms of the GNU GPL v2.0.

This file can be edited at src/myModule/main.cpp.
**/

/**
 * @file main.cpp
 * @brief main code for the building of a saliency map based on colour information
 */

#include "iCub/colourSaliencyModule.h" 
#include <yarp/os/all.h>

using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[])
{
    Network yarp;
    
    Time::turboBoost();
    colourSaliencyModule module; 

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("colourSaliency.ini"); //overridden by --from parameter
    rf.setDefaultContext("logpolarAttention/conf");   //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
 
    module.runModule(rf);
    return 0;
}


