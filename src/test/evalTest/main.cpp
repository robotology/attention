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
#include <yarp/math/Math.h>
#include <yarp/math/Rand.h>
#include <iCub/ctrl/kalman.h>
#include <iCub/attention/predModels.h>
#include <iCub/attention/evalThread.h>
// #include <yarp/os/Port.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;
using namespace attention::evaluator;


 int main(int argc, char *argv[]) {
    
     // Open the network
     Network yarp;
     Time::turboBoost();

     Semaphore s;
     evalThread* eval;
     /*
     linAccModel* modelB = new linAccModel();
     modelB->init(1.0);
     int rowA = modelB->getA().rows();
     int colA = modelB->getA().cols();
     Vector z0(rowA);
     Vector x0(rowA);
     x0.zero();z0.zero();
     x0(0) = 1.0; 
     Matrix P0(rowA,colA);
     printf("initialisation of P0 %d %d \n", rowA, colA);
     for (int i = 0; i < rowA; i++) {
         for (int j = 0; j < colA; j++) { 
             P0(i,j) += 0.01;
         }      
     }
     printf("modelB\n %s \n %s \n", modelB->getA().toString().c_str(),modelB->getB().toString().c_str());    
     printf("P0\n %s \n", P0.toString().c_str());    
     genPredModel* mB = dynamic_cast<genPredModel*>(modelB);
     */
     
     
     printf(" creating eval thread \n");
     eval = new evalThread();
     //eval->init(z0,x0,P0);
     //printf("genPred model A \n %s \n",mB    ->getA().toString().c_str());
     //printf("lin acc model A \n %s \n",modelB->getA().toString().c_str());
     //printf("just initialised genPredModel %08X \n",&eval);
     s.wait();
     eval->start();
     s.post();
     
     printf("Press CTRL-C to stop the module \n");
     while(true) {}
     
     Network::fini();
    
}
