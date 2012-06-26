// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file main.cpp
 * @brief Implementation of the kalmanTest
 */


// #include <math.h>
#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <iCub/ctrl/kalman.h>
#include "predModels.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;

 int main(int argc, char *argv[]) {
     //Network::init();
     // Open the network
     Network yarp;
     Matrix A,H,Q,R;
     
     
     
     printf("Creating prediction models \n");
     linVelModel* modelA = new linVelModel();
     modelA->init(0.5);
     printf("modelA\n %s \n", modelA->getB().toString().c_str());

     linAccModel* modelB = new linAccModel();
     modelB->init(0.5);
     printf("modelB\n %s \n", modelB->getB().toString().c_str());
     
     minJerkModel* modelC = new minJerkModel();
     modelC->init(0.5);
     printf("modelC\n %s \n", modelC->getA().toString().c_str());
    
     Network::fini();
    
}
