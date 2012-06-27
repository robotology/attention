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
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <iCub/attention/predModels.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace yarp::math;
using namespace attention::predictor;

#define numIter 3

Matrix evaluateModel(genPredModel* model,Matrix zMeasure ) {
    printf(" \n\n\nEVALUATING THE MODEL: %s \n", model->getType().c_str());

    // initialisation of the karman filter
    Matrix A = model->getA();
    Matrix B = model->getB();

    Matrix H(3,3);
    H(0,0) = 1; H(0,1) = 0; H(0,2) = 0;
    H(1,0) = 0; H(1,1) = 1; H(1,2) = 0; 
    H(2,0) = 0; H(2,1) = 0; H(2,2) = 1;
    printf("pinv H : \n %s \n", pinv(H).toString().c_str());
    
    Matrix Q(3,3);
    Q(0,0) = 0.01;  Q(0,1) = 0.01;  Q(0,2) = 0.01;
    Q(1,0) = 0.01;  Q(1,1) = 0.01;  Q(1,2) = 0.01;
    Q(2,0) = 0.01;  Q(2,1) = 0.01;  Q(2,2) = 0.01;
            
    Matrix R(3,3);
    R(0,0) = 0.001; R(0,1) = 0.01; R(0,2) = 0.012;
    R(1,0) = 0.01; R(1,1) = 0.003; R(1,2) = 0.01;
    R(2,0) = 0.003; R(2,1) = 0.002; R(2,2) = 0.02;
    
    Kalman kSolver(A,B,H,Q,R);
   
    // initialisation of the initial state of the karman filter
    Vector z0(3);
    z0(0) = 0; z0(1) = 1; z0(2) = 0;
    Vector x0(3);
    x0(0) = 0; x0(1) = 1; x0(2) = 0;
    Matrix P0(3,3);
    P0(0,0) = 1; P0(0,1) = 1; P0(0,2) = 1;
    P0(1,0) = 1; P0(1,1) = 1; P0(1,2) = 1;
    P0(2,0) = 1; P0(2,1) = 1; P0(2,2) = 1;
    
    kSolver.init (z0, x0, P0);
    
    //printf("estim.state %s \n", kSolver.get_x().toString().c_str());
    //printf("estim.error covariance\n %s \n",kSolver.get_P().toString().c_str());

    double c = 1.0;
    Vector z(3);
    Vector x(3);
    Vector u(3); 
     
    for(int i = 0; i < numIter ; i++) {
        printf("----------------------------------------------------------\n");
        //z(0) = Random::uniform() + 0.5;
        //z(1) = Random::uniform() + 0.5;
        u(0) = c + Random::uniform() + 0.5 ; u(1) = c + Random::uniform() + 0.5; u(2) = c+ Random::uniform() + 0.5;
        z = zMeasure.getRow(i);
        printf("measure %s \n",z.toString().c_str());
        printf("input  %s \n", u.toString().c_str());
        x = kSolver.filt(u,z);
        printf("estim.state %s \n", x.toString().c_str());
        //fprintf(stateDump, "%s \n",x.toString().c_str() );
        printf("K \n %s \n", kSolver.get_K().toString().c_str());
        printf("estim.error covariance P:\n %s \n",kSolver.get_P().toString().c_str());
        //printf(errorDump,"%s \n",kSolver.get_P().getRow(1).toString().c_str());
        printf("----------------------------------------------------------\n");
    }
    return kSolver.get_P();
}

 int main(int argc, char *argv[]) {
    //Network::init();
    // Open the network
    Network yarp;
    
    printf("Creating prediction models \n");
    linVelModel* modelA = new linVelModel();
    modelA->init(0.5);
    printf("modelA\n %s \n %s \n", modelA->getA().toString().c_str(), modelA->getB().toString().c_str());

    linAccModel* modelB = new linAccModel();
    modelB->init(0.5);
    printf("modelB\n %s \n %s \n", modelB->getA().toString().c_str(),modelB->getB().toString().c_str());
    
    minJerkModel* modelC = new minJerkModel();
    modelC->init(0.5);
    printf("modelC\n %s \n %s \n", modelC->getA().toString().c_str(), modelC->getB().toString().c_str());

    modelQueue mQueue(false);
    //mQueue.push_back(modelA);
    //mQueue.push_back(modelB);
    mQueue.push_back(modelC);

    Matrix zMeasure(numIter,3);
    for(int i = 0; i < numIter; i++) {
        zMeasure(i,0) = 0.0;
        zMeasure(i,1) = 1.0 + Random::uniform();
        zMeasure(i,2) = 0.0;
    }
    
    for (size_t i = 0; i < mQueue.size(); i++) {        
            genPredModel* m = dynamic_cast<genPredModel*>(mQueue[i]);
            Matrix res = evaluateModel(m,zMeasure); 
            printf("error:\n  %s \n", res.toString().c_str());
    }
    
    delete modelA;
    delete modelB;
    delete modelC;

    Network::fini();
    
}
