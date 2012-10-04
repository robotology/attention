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
// #include <yarp/os/Port.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::ctrl;


 int main(int argc, char *argv[]) {
    
     // Open the network
     Network yarp;
     Time::turboBoost();

     const static int numIteract = 60;

     FILE *stateDump = fopen("stateDump.txt", "w+");
     FILE *errorDump = fopen("errorDump.txt", "w+");

     //Matrix A,H,Q,R;
     Matrix zMemory(numIteract,2);
     
     Matrix A(2,2);
     A(0,0) = 1; A(0,1) = 0; A(1,0) = 0; A(1,1) = 1;
     Matrix H(2,2);
     H(0,0) = 1; H(0,1) = 0; H(1,0) = 0; H(1,1) = 1;
     Matrix Q(2,2);
     Q(0,1) = 1e-5;  Q(0,0) = 1e-5;  Q(1,0) = 1e-5;  Q(1,1) = 1e-5;    
     Matrix R(2,2);
     R(0,1) = 0.001; R(0,0) = 0.001; R(1,0) = 0.001; R(1,1) = 0.001; 
 
     Kalman kSolver(A,H,Q,R);
     
     Vector z0(2);
     z0(0) = 1; z0(1) = 1;
     Vector x0(2);
     x0(0) = 0; x0(1) = 0;     
     Matrix P0(2,2);
     P0(0,0) = 0; P0(0,1) = 0; P0(1,0) = 0; P0(1,1) =  0;
     kSolver.init (x0, P0);
          
   
     printf("estim.state %s \n", kSolver.get_x().toString().c_str());
     printf("estim.error covariance\n %s \n",kSolver.get_P().toString().c_str());

     Vector z(2);
     Vector x(2);
     
     for(int i = 0; i < numIteract ; i++) {
         printf("----------------------------------------------------------\n");
         z(0) = Random::uniform() + 0.5;
         z(1) = Random::uniform() + 0.5;
         printf("measure %s \n",z.toString().c_str());
         x = kSolver.filt(z,z);
         printf("estim.state %s \n", x.toString().c_str());
         fprintf(stateDump, "%s \n",x.toString().c_str() );
         printf("estim.state %s \n", kSolver.get_x().toString().c_str());
         printf("estim.error covariance\n %s \n",kSolver.get_P().toString().c_str());
         fprintf(errorDump,"%s \n",kSolver.get_P().getRow(1).toString().c_str());
         printf("----------------------------------------------------------\n");
     }
     
     printf("estim.state %s \n",kSolver.get_x().toString().c_str());
     printf("estim.error covariance\n %s \n",kSolver.get_P().toString().c_str());
     printf("Kalman Gain Matrix\n  %s \n",kSolver.get_K().toString().c_str());
    
     Network::fini();
    
}
