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
 * @file main.cpp
 * @brief main code for the building of a saliency map based on colour information
 */

#include "iCub/attPrioritiserModule.h" 
#include <yarp/os/all.h>

using namespace yarp::os;
using namespace yarp::sig;

//YARP_DECLARE_DEVICES(icubmod)

int main(int argc, char * argv[])
{
    Network yarp;
    //YARP_REGISTER_DEVICES(icubmod)
    Time::turboBoost();
    attPrioritiserModule module; 

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("attentionPrioritiser.ini");      //overridden by --from parameter
    rf.setDefaultContext("logpolarAttention/conf");           //overridden by --context parameter
    rf.configure("ICUB_ROOT", argc, argv);
 
    printf("running the module \n");
    module.runModule(rf);
    return 0;
}


