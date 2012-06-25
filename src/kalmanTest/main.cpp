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
// #include <yarp/os/Port.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;

 int main(int argc, char *argv[]) {
     //Network::init();
     // Open the network
     Network yarp;
     //Matrix A,H,Q,R;
     
     Matrix A(2,2);
     A(0,0) = 1; A(0,1) = 2; A(1,0) = 3; A(1,1) = 1;
     Matrix H(2,2);
     A(0,0) = 0; A(0,1) = 2; A(1,0) = 3; A(1,1) = 0;
     Matrix Q(2,2);
     Q(0,1) = 1; Q(0,0) = 1; Q(1,0) = 1; Q(1,1) = 1;    
     Matrix R(2,2);
     R(0,1) = 2; R(0,0) = 2; R(1,0) = 2; R(1,1) = 2;

     
 
     Kalman kSolver(A,H,Q,R);
     
     Vector z0(2);
     z0(0) = 1; z0(1) = 1;
     Vector x0(2);
     x0(0) = 2; x0(1) = 1;     
     Matrix P0(2,2);
     P0(0,0) = 1; P0(0,1) = 1; P0(1,0) = 1; P0(1,1) =  1;
     kSolver.init (z0, x0, P0);
          
     printf("estim.state %s \n", kSolver.get_x().toString().c_str());
     printf("estim.error covariance %s \n",kSolver.get_P().toString().c_str());
     
     Vector z(2);
     Vector x(2);
     
     for(int i = 0; i < 10 ; i++) {
         z(0) = 1.0;  
         z(1) = 1.5;
         x = kSolver.filt(z);
     }
     
     printf("estim.state %s \n",kSolver.get_x().toString().c_str());
     printf("estim.error covariance %s \n",kSolver.get_P().toString().c_str());
     printf("Kalman Gain Matrix  %s \n",kSolver.get_K().toString().c_str());
    
     Network::fini();
    
}
