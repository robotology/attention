// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C)2020  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Omar Eldardeer
  * email: omar.eldardeer@iit.it
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

#include "iCub/egocentricAudioCropperModule.h"
int main(int argc, char *argv[]) {

    //initialize yarp network
    Network yarp;
    egocentricAudioCropperModule module;

    /* prepare and configure the resource finder */
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("egocentricAudioCropper.ini");    //overridden by --from parameter
    rf.setDefaultContext("logpolarAttention");    //overridden by --context parameter
    rf.configure(argc, argv);

    /* run the module: runModule() calls configure first and, if successful, it then runs */
    module.runModule(rf);
    return 0;
}

