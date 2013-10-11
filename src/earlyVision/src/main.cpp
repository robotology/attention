// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Rea Francesco, Shashank Pathak
  * email:francesco.rea@iit.it, shashank.pathak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/
  
/**
 * @file main.cpp
 * @brief main code for the early vision module; this is part of the logpolar attention system.
 */

#include "iCub/earlyVisionModule.h" 

using namespace yarp::os;
using namespace yarp::sig;


int main(int argc, char * argv[]) {
    Network yarp;
    earlyVisionModule module; 

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("earlyVisionModule.ini"); //overridden by --from parameter
    rf.setDefaultContext("logpolarAttention");        //overridden by --context parameter
    //rf.configure("ICUB_ROOT", argc, argv);
    rf.configure(argc, argv);     // aligned the 10/10/13 REA
 
    module.runModule(rf);
    return 0;
}


