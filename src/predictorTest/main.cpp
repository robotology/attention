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

#define numInter 10

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

    Matrix A = modelA->getA();
    Matrix B = modelA->getB();

    Kalman kSolver(A,B,H,Q,R);
    Matrix zMeasure(numIteract,2);
    Matrix Q(3,3);
    Q(0,0) = 1e-5;  Q(0,1) = 1e-5;  Q(0,2) = 1e-5;
    Q(1,0) = 1e-5;  Q(1,1) = 1e-5;  Q(1,2) = 1e-5;
    Q(2,0) = 1e-5;  Q(2,1) = 1e-5;  Q(2,2) = 1e-5;

            
    Matrix R(2,2);
    R(0,1) = 0.001; R(0,0) = 0.001; R(1,0) = 0.001; R(1,1) = 0.001; 

    Vector z0(3);
    z0(0) = 1; z0(1) = 0; z0(2) = 0
    Vector x0(3);
    x0(0) = 0; x0(1) = 0; x0(2) = 0;
    Matrix P0(3,3);
    P0(0,0) = 0; P0(0,1) = 0; P0(0,2) = 0;
    P0(1,0) = 0; P0(1,1) = 0; P0(1,2) = 0;
    P0(2,0) = 0; P0(2,1) = 0; P0(2,2) = 0;
    
    kSolver.init (z0, x0, P0);
    
    printf("estim.state %s \n", kSolver.get_x().toString().c_str());
    printf("estim.error covariance\n %s \n",kSolver.get_P().toString().c_str());

    Vector z(3);
    Vector x(3);
     
    for(int i = 0; i < numIteract ; i++) {
        printf("----------------------------------------------------------\n");
        //z(0) = Random::uniform() + 0.5;
        //z(1) = Random::uniform() + 0.5;
        z = zMeasure.getRow(i);
        printf("measure %s \n",z.toString().c_str());
        x = kSolver.filt(z);
        printf("estim.state %s \n", x.toString().c_str());
        fprintf(stateDump, "%s \n",x.toString().c_str() );
        printf("estim.state %s \n", kSolver.get_x().toString().c_str());
        printf("estim.error covariance\n %s \n",kSolver.get_P().toString().c_str());
        fprintf(errorDump,"%s \n",kSolver.get_P().getRow(1).toString().c_str());
        printf("----------------------------------------------------------\n");
    }

    delete modelA;
    delete modelB;
    delete modelC;

    Network::fini();
    
}
