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

class evalThread : public yarp::os::Thread {
 protected:
    iCub::ctrl::Kalman* kSolver;
    
    bool ready;
    bool finished;
    Semaphore mutexR, mutexF;
    
    Vector u, x, z;
    
    Matrix zMeasure, uMeasure;   
    
 public:
    evalThread(){
        
    }

    evalThread(const Matrix &A,const Matrix &B,const Matrix &H,const Matrix &Q,const Matrix &R) {
        kSolver = new Kalman(A,B,H,Q,R);
    }

    ////////////////////////////////////////////////////////////////////

    virtual bool threadInit(){
        ready    = false;
        finished = false;

    }

    ////////////////////////////////////////////////////////////////////
    
    virtual void run() {
        while (!isStopping()) {
            while(!ready) {
                //Time::wait(0.1);
            }
            // running the filter for the number of measurements
            mutexR.wait();
            ready = false;
            mutexR.post();

            for(int i = 0; i < numIter ; i++) {
                printf("----------------------------------------------------------\n");
                
                u(0) = Random::uniform() + 0.5 ; 
                
                z = zMeasure.getRow(i);
                u = uMeasure.getRow(i);
                x = kSolver->filt(u,z);
                //printf("estim.state %s \n", x.toString().c_str()); 
                //printf("estim.error covariance P:\n %s \n",kSolver.get_P().toString().c_str());
                
                printf("----------------------------------------------------------\n");
            }

            //setting the result out
            mutexF.wait();
            finished = true;
            mutexF.post();
            
        }
    }

    ///////////////////////////////////////////////////////////////////

    virtual void onStop() {

    }

    ///////////////////////////////////////////////////////////////////

    virtual void threadRelease() {
        delete kSolver;
    }

    //////////////////////////////////////////////////////////////////

    void init(Vector z0, Vector x0, Matrix P0) {
        kSolver->init(z0, x0, P0);
    }

    /////////////////////////////////////////////////////////////////

    void setMeasurements(Matrix _u, Matrix _z) {
        ready = true;
        finished = false;
        uMeasure = _u; zMeasure = _z;
    }

    ////////////////////////////////////////////////////////////////

    Matrix getP() {
        mutexF.wait();
        while(!finished) {
            mutexF.post();
            Time::delay(0.1);
            mutexF.wait();
        }
        mutexF.post();

        return kSolver->get_P();
    }


};

Matrix evaluateModel(genPredModel* model,Matrix zMeasure ) {
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

    Kalman kSolver(A,B,H,Q,R);
   
    // initialisation of the initial state of the karman filter
    kSolver.init (z0, x0, P0);
    
    //printf("estim.state %s \n", kSolver.get_x().toString().c_str());
    //printf("estim.error covariance\n %s \n",kSolver.get_P().toString().c_str());

    double c = 1.0;
     
     
    for(int i = 0; i < numIter ; i++) {
        printf("----------------------------------------------------------\n");
        //z(0) = Random::uniform() + 0.5;
        //z(1) = Random::uniform() + 0.5;
        if(rowA == 2) {
            u(0) = c + Random::uniform() + 0.5 ; 
        }
        else {
            u(0) = c + Random::uniform() + 0.5 ;
        }
        
        z = zMeasure.getRow(i);
        //printf("measure %s \n",z.toString().c_str());
        //printf("input  %s \n", u.toString().c_str());
        x = kSolver.filt(u,z);
        printf("estim.state %s \n", x.toString().c_str());
        //fprintf(stateDump, "%s \n",x.toString().c_str() );
        //printf("K \n %s \n", kSolver.get_K().toString().c_str());
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
    modelC->init(2, 3);
    printf("modelC\n %s \n %s \n", modelC->getA().toString().c_str(), modelC->getB().toString().c_str());

    modelQueue mQueue(false);
    //mQueue.push_back(modelA);
    //mQueue.push_back(modelB);
    mQueue.push_back(modelC);

    Matrix zMeasure;
    
    
    for (size_t i = 0; i < mQueue.size(); i++) {        
        genPredModel* m = dynamic_cast<genPredModel*>(mQueue[i]);
        zMeasure.resize(numIter,m->getRowA());
        for(int j = 0; i < numIter; i++) {
            for (int k  =0 ; k < m->getRowA(); k ++) {
                zMeasure(j,k) = 1.0 + Random::uniform();
            }
        }
        Matrix res = evaluateModel(m,zMeasure); 
        printf("error:\n  %s \n", res.toString().c_str());
    }
    
    delete modelA;
    delete modelB;
    delete modelC;

    Network::fini();
    
}
