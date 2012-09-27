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
#include <iCub/attention/evalThread.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::ctrl;
using namespace yarp::math;
using namespace attention;
//using namespace attention;


//___________________________________________________________________________________________________

Matrix evaluateModel(genPredModel* model,Matrix uMeasure,Matrix zMeasure ) {
    printf(" \n\n\nEVALUATING THE MODEL: %s \n", model->getType().c_str());
    
    int rowA = model->getRowA();
    int colA = model->getColA();        
    
    // initialisation of the karman filter
    Matrix A = model->getA();
    Matrix B = model->getB();
    Matrix H = model->getH();
    
    /*
    Matrix H(3,3);
    H(0,0) = 1; H(0,1) = 0; H(0,2) = 0;
    H(1,0) = 0; H(1,1) = 1; H(1,2) = 0; 
    H(2,0) = 0; H(2,1) = 0; H(2,2) = 1;
    */

    printf("pinv H : \n %s \n", pinv(H).toString().c_str());
                
    Matrix R (rowA,colA);
    Matrix Q (rowA,colA);
    Matrix P0(rowA,colA);
    
    Vector z0(rowA);
    Vector x0(rowA);
    Vector z(colA);
    Vector x(colA);
    Vector u(1);
    
    for (int i = 0; i < rowA; i++) {
        for (int j = 0; j < colA; j++) { 
            Q(i, j) += 0.01; 
            R(i, j) += 0.001;
            P0(i,j) += 0.01;
        }      
    }
    
    printf("initialising the model evalThread \n");
    evaluator::evalThread et;
    et.setModel(model);
    printf("just passed the predicting model \n");
    et.init(z0, x0, P0);
    printf("just initialised the prediction model \n");
    et.start();
    
    // initialisation of the initial state of the karman filter
    printf("creating a new kalman filter \n");
    Kalman kSolver(A,B,H,Q,R);
    kSolver.init (z0, x0, P0);
    Time::delay(2.0);
    
    //double c = 1.0;
    //Matrix uMeasure(numIter, 2);
    //uMeasure.zero();
   
    
    printf("setting measurements \n");
    et.setMeasurements(uMeasure,zMeasure);
    
    printf("estim.state %s \n", kSolver.get_x().toString().c_str());
    printf("estim.error covariance\n %s \n",kSolver.get_P().toString().c_str());
   
    while(!et.getEvalFinished()) {
        Time::delay(0.1);
    }
    printf("Stopping the thread \n");
    et.stop();

    return et.getP();
}


 int main(int argc, char *argv[]) {
    //Network::init();
    // Open the network
    Network yarp;
    
    printf("Creating prediction models \n");
    linVelModel* modelA = new linVelModel();
    modelA->init(1.0);
    printf("modelA\n %s \n \n %s \n", modelA->getA().toString().c_str(), modelA->getB().toString().c_str());

    linAccModel* modelB = new linAccModel();
    modelB->init(1.0);
    printf("modelB\n %s \n \n %s \n", modelB->getA().toString().c_str(),modelB->getB().toString().c_str());
    
    minJerkModel* modelC = new minJerkModel();
    modelC->init(1, 1);
    printf("modelC\n %s \n \n %s \n", modelC->getA().toString().c_str(), modelC->getB().toString().c_str());

    modelQueue mQueue(false);
    mQueue.push_back(modelA);
    mQueue.push_back(modelB);
    mQueue.push_back(modelC);
    Matrix zMeasure;
    Matrix uMeasure;

    int numIter = 3;
    for (size_t i = 0; i < mQueue.size(); i++) {        
        genPredModel* m = dynamic_cast<genPredModel*>(mQueue[i]);
        
        Matrix uMeasure(m->getRowA(),numIter);
        zMeasure.resize(numIter,m->getRowA());
        uMeasure.resize(m->getRowA(),numIter);
        printf("preparing the zMeasure and uMeasure %d x %d \n", numIter, m->getRowA());
        for(int j = 0; j < numIter; j++) {
            for (int k  =0 ; k < m->getRowA(); k ++) {
                zMeasure(k,j) = 1.0 + Random::uniform();
                uMeasure(k,j) = 1.0 + Random::uniform();
            }
        }
        printf("zMeasure \n %s \n", zMeasure.toString().c_str());
        printf("uMeasure \n %s \n", uMeasure.toString().c_str());
        Matrix res = evaluateModel(m,uMeasure,zMeasure); 
        printf("error:\n  %s \n", res.toString().c_str());

        
        
    }



    
    delete modelA;
    delete modelB;
    delete modelC;

    Network::fini();
    
}


