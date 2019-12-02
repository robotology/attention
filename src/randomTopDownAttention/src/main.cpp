/*
  * Copyright (C)2019  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include "iCub/topDownAttentionModule.h"
int main(int argc, char *argv[]) {
    if (argc<3) {
        fprintf(stderr, "Please supply (1) a port name for the client\n");
        fprintf(stderr, "              (2) a port name for the server\n");
        return 1;
    }

    Network yarp;
    topDownAttentionModule module;

    ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultConfigFile("periodicAttiontionModule.ini");    //overridden by --from parameter
    rf.setDefaultContext("periodicAttiontion");    //overridden by --context parameter
    rf.configure(argc, argv);

    module.runModule(rf);
    return 0;
}
