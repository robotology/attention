// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Francesco Rea
 * email:  francesco.rea@iit.it
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

#ifndef __EVALUATION_THREAD_H__
#define __EVALUATION_THREAD_H__

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


namespace attention
{

namespace evaluator
{

class evalQueue;

class evalThread : public yarp::os::Thread {
 protected:
    iCub::ctrl::Kalman* kSolver;
    genPredModel* gPredModel;

    int numIter;
    int a;
    
    bool dataReady;
    bool evalFinished;
   
    yarp::os::Semaphore mutexR;
    yarp::os::Semaphore    mutexF; 
    Vector u, x, z;    
    Matrix zMeasure, uMeasure;   
    
 public:
    evalThread(){
        numIter = 3;
        a = 0;
    }

    evalThread(const Matrix &A,const Matrix &B,const Matrix &H,const Matrix &Q,const Matrix &R) {
        numIter = 3;
        kSolver = new Kalman(A,B,H,Q,R);
    }
    
    
    evalThread(const attention::predictor::genPredModel& model) {
        numIter = 3;
        
        gPredModel = (attention::predictor::genPredModel*) &model;
        int rowA = model.getRowA();
        int colA = model.getColA();        
    
        // initialisation of the karman filter
        Matrix A = model.getA();
        Matrix B = model.getB();
        Matrix H = model.getH();
        
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
                Q (i, j) += 0.01; 
                R (i, j) += 0.001;
                P0(i, j) += 0.01;
            }      
        }

        kSolver = new Kalman(A,B,H,Q,R);
      
    }
    
    
    
    ////////////////////////////////////////////////////////////////////

    virtual bool threadInit(){
        printf("evalThread::threadInit::thread init \n");
        mutexR.wait();
        dataReady    = false;
        mutexR.post();
        mutexF.wait();
        evalFinished = false;
        mutexF.post();
        printf("evalThread::end of initialisation \n");
        return true;
    }

    ////////////////////////////////////////////////////////////////////
    
    virtual void run() {
        printf("in the run \n");
        bool dataR;
        while (!isStopping()) {

            //printf("evalThread cycle \n");
            //Time::delay(1.0);
            //}*/

            //printf(". \n");        
            //printf("inside the while \n");
            
            
            printf("pre mutex %d \n", dataR);
            //mutexR.wait();
            dataR = getDataReady();
            //mutexR.post();
            printf("after mutex %d \n", dataR);
            
           
            
            while(!dataR) {
                Time::delay(1.5);
                printf(". ");
        
                //mutexR.wait();
                dataR = getDataReady();                
                printf("dataR %d %d %d \n", dataR, dataReady, a);
                //mutexR.post();
            }
            

            printf("out of while %08X %d  \n",this, dataReady);
            //printf("out of while %d  \n", dataReady);
            //printf("out of while %d  \n", dataReady);
            //printf("out of while %d  \n", dataReady);
            
            
            // running the filter for the number of measurements
            // mutexR.wait();
            dataReady = false;
            //mutexR.post();
            
            for(int i = 0; i < numIter ; i++) {
                printf("%d < %d =>----------------------------------------------------------\n", i, numIter);
                
                double s = Random::uniform() + 0.5 ; 
                
                z = zMeasure.getRow(i);
                printf("just extracted z = \n %s \n", z.toString().c_str());
                u = uMeasure.getCol(i);
                u.resize(1,0);
                u(0) = 1.5;
                printf("just extracted u = \n %s \n", u.toString().c_str());
                x = kSolver->filt(u,z);
                printf("estim.state %s from % \n", x.toString().c_str()); 
                //printf("estim.error covariance P:\n %s \n",kSolver.get_P().toString().c_str());
                
            }
        
            //setting the result out
            printf("setting the result out \n");
            mutexF.wait();
            evalFinished = true;
            mutexF.post();
            
            
            
            Time::delay(1.5);
           
        }
        
    }



    ///////////////////////////////////////////////////////////////////

    virtual void onStop() {
        dataReady = true;
        numIter = 0;
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
    
    void setModel(genPredModel* model){
        int rowA = model->getRowA();
        int colA = model->getColA();        
    
        // initialisation of the karman filter
        Matrix A = model->getA();
        Matrix B = model->getB();
        Matrix H = model->getH();
        
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

        kSolver = new Kalman(A,B,H,Q,R);
    }

    /////////////////////////////////////////////////////////////////

    void setMeasurements(Matrix _u, Matrix _z) {
        printf("setMeasurement \n");
        //mutexR.wait();
        dataReady = true;
        printf("setMeasurements:dataReady %d \n", dataReady);
        //mutexR.post();
        //mutexF.wait();
        evalFinished = false;
        //mutexF.post();
        uMeasure = _u; zMeasure = _z;
        printf("_u %s uMeasure %s \n", _u.toString().c_str(), uMeasure.toString().c_str());
        a = 2;
    }

    ////////////////////////////////////////////////////////////////

    Matrix getP() {
        //mutexF.wait();
        //while(!finished) {
        //    mutexF.post();
        //    Time::delay(0.1);
        //    mutexF.wait();
        //}
        //mutexF.post();

        return kSolver->get_P();
    }

    ///////////////////////////////////////////////////////////////

    bool getEvalFinished() {
         mutexF.wait();
         bool ef = evalFinished;
         mutexF.post();

         return ef;
    }

     ///////////////////////////////////////////////////////////////

    bool getDataReady() {
        // mutexR.wait();
         bool dr = dataReady;
         // mutexR.post();

         return dr;
    }


};

/**************************************************************************/
class evalQueue : public std::deque<evalThread*>
{
private:
    evalQueue(const evalQueue&);
    evalQueue &operator=(const evalQueue&);

protected:
    bool owner;

public:
    evalQueue()                    { owner = true;        }
    evalQueue(const bool _owner)   { owner = _owner;      }
    void setOwner(const bool owner) { this->owner = owner; }
    bool getOwner()                 { return owner;      }
    ~evalQueue() {
        if (owner)
            for (size_t i=0; i<size(); i++)
                if ((*this)[i]!=NULL)
                    delete (*this)[i];
        
        clear();
    }
};


}

}

#endif


