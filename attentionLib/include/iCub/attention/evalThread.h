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

    static const int numIter    = 39;
    
    bool dataReady;
    bool evalFinished;
   
    yarp::os::Semaphore mutexR;
    yarp::os::Semaphore mutexF; 
    yarp::os::Semaphore mutexData;
    yarp::os::Semaphore mutexMSE;
    Vector* u;
    Vector* x;
    Vector* z;
    Vector z0, x0;                       //initial values
    Matrix P0;
    Matrix* zMeasure;
    Matrix* uMeasure;   

    int rowA;
    int colA;
    int id;

    double meanSquareError;
    
 public:
    evalThread(){
        //numIter = 3;
        printf("generic constructor %d \n", numIter);
        //Matrix tmp(numIter, 3);
        //tmp.zero();
        //uMeasure = tmp;
        //zMeasure = tmp;
        uMeasure = 0; //new Matrix(numIter,3);
        zMeasure = 0; //new Matrix(numIter,3);
        z = 0; //new Vector(3);
        x = 0; //new Vector(3);
        
        //mutexR    = new Semaphore();
        //mutexF    = new Semaphore();
        //mutexData = new Semaphore();

    }

    evalThread(const Matrix &A,const Matrix &B,const Matrix &H,const Matrix &Q,const Matrix &R) {
        //numIter = 3;
        printf("constructor from matrices \n");
        //uMeasure(numIter,3);
        //zMeasure(numIter,3);
        
        //mutexR    = new Semaphore();
        //mutexF    = new Semaphore();
        //mutexData = new Semaphore();

        rowA = A.rows();
        colA = A.cols();
        uMeasure = new Matrix(rowA,numIter);
        zMeasure = new Matrix(numIter,colA);
        z = new Vector(rowA);
        x = new Vector(rowA);
        kSolver = new Kalman(A,B,H,Q,R);
    }
    
    
    evalThread(const attention::predictor::genPredModel& model) {
        printf("constructor from generic predictive model \n");
        id    = 2;
        printf("id %d \n", id);
        //numIter = 3;
        //Matrix tmp(numIter, 3);
        //tmp.zero();

        //mutexR    = new Semaphore();
        //mutexF    = new Semaphore();
        //mutexData = new Semaphore();
        

        //printf("\n evalThread::evalThread:uMeasure %08x  \n%s \n",this, uMeasure->toString().c_str()); 
        
        //uMeasure(numIter,3);
        //zMeasure(numIter,3);
        
        gPredModel = (attention::predictor::genPredModel*) &model;
        rowA = model.getRowA();
        colA = model.getColA();        
    
        // initialisation of the karman filter
        Matrix A = model.getA();
        Matrix B = model.getB();
        Matrix H = model.getH();
        
        Matrix R (rowA,colA);
        Matrix Q (rowA,colA);
        Matrix P0(rowA,colA);
        
        Vector z0(rowA);
        Vector x0(rowA);
        z = new Vector(rowA);
        z->zero();
        printf("############################## z: %08X %d  \n", z, rowA);
        x = new Vector(rowA);
        x->zero();
        x->operator()(0) = 0.1;
        
        uMeasure = new Matrix(rowA,numIter);
        zMeasure = new Matrix(numIter,colA);        

        printf("############this %08X ####################  initialisation of the z(%08X)) and x size %d \n",this, z, z->length());
        printf("z : %s \n", z->toString().c_str());
        Vector u(1);
        
        for (int i = 0; i < rowA; i++) {
            for (int j = 0; j < colA; j++) { 
                Q (i, j) += 0.01; 
                R (i, j) += 0.001;
                P0(i, j) += 0.01;
            }      
        }

        kSolver = new Kalman(A,B,H,Q,R);
        //kSolver->init(*z,*x,P0);

        printf("evalTread:constructor successfully ended \n");
      
    }
    
    /////////////////////////////////////////////////////////////////////
    
    ~evalThread() {
        //delete mutexR;
        //delete mutexF;
        //delete mutexData;
        if (x!=0){
            delete x;
        }
        if (z!=0) {
            delete z;
        }
        if (uMeasure!=0) {
            delete uMeasure;
        }
        if (zMeasure != 0) {
            delete zMeasure;
        }
    }
    
    
    ////////////////////////////////////////////////////////////////////

    virtual bool threadInit(){
        
        printf("\n \n evalThread::threadInit:thread init \n");
        printf("thread threadinit pre mutex  \n");
        mutexR.wait();
        printf("thread init post mutex \n");
        dataReady    = false;
        printf("thread threadinit pre mutex  \n");
        mutexR.post();
        printf("thread threadinit post mutex \n");
        printf("evalThread::threadInit:correctly initialised the dataReady \n");
        
        
        mutexF.wait();
        evalFinished = false;
        mutexF.post();
        printf("evalThread:threadInit:end of initialisation \n \n \n");
        return true;
    }

    ////////////////////////////////////////////////////////////////////
    
    virtual void run() {
        printf("in the run of evalThread \n");
        bool dataR = false;

        dataR = getDataReady(); // no need for semaphore interaction because already embedded in the function
        
        while (!isStopping()) {           
            
            //printf("evalThread cycle \n");
            //printf("pre mutex %d \n", dataR);            
            dataR = getDataReady();
            //printf("afterGetDataReady %d \n", dataR); 
            //printf("after mutex %d \n", dataR);
                        
            while(!dataR) {
                Time::delay(0.5);       
                dataR = getDataReady();                    
            }

            // running the filter for the number of measurements
            mutexR.wait();
            dataReady = false;
            mutexR.post();
            
            printf("\n -------- the general model associated uses %f \n",gPredModel->getParamB());
            printf("zMeasure %s \n", zMeasure->toString().c_str());

            double sumError = 0;
            for(int i = 0; i < numIter ; i++) {
                //printf("%d < %d =>----------------------------------------------------------\n", i, numIter);
                //printf("%08X  %d \n", this, id);
                double s = Random::uniform() + 0.5 ; 
                Vector zTmp = zMeasure->getRow(i);

                Vector uTmp = uMeasure->getCol(i); //extracted u from the ith column 
                uTmp.resize(1,0);  // gives the vector the correct shape
                uTmp(0) = 1.5;
                

                printf("zTmp %s \n", zTmp.toString().c_str());
                Vector xTmp  = kSolver->filt(uTmp,zTmp);

                printf("estim.state %s \n", xTmp.toString().c_str()); 
                //printf("estim.error covariance P:\n %s \n",kSolver->get_P().toString().c_str());
                
                Vector diff  = zTmp - xTmp;
                double diffS = diff(0);
                double diffV = diff(1);
                double diffA = diff(2);
                sumError   += (diffS * diffS) + (diffV * diffV) + (diffA * diffA);
               
            }

            mutexMSE.wait();
            meanSquareError = sumError / numIter;
            mutexMSE.post();
            
            //Time::delay(5.0);
            
            //setting the result out
            printf("-------- setting the result out \n \n");
            mutexF.wait();
            evalFinished = true;
            mutexF.post();    
            
            //Time::delay(1.5);           
            
        }
    }



    ///////////////////////////////////////////////////////////////////

    virtual void onStop() {
        dataReady = true;
    }

    ///////////////////////////////////////////////////////////////////

    virtual void threadRelease() {
        delete kSolver;
    }

    //////////////////////////////////////////////////////////////////

    void init(Vector _z0, Vector _x0, Matrix _P0) {
        
        printf("evalThread::init %08X \n", kSolver);
        z0 = _z0;
        x0 = _x0;
        
        P0 = _P0;
        printf("_x0 \n %s \n", _x0.toString().c_str());
        printf("P0 \n %s \n", P0.toString().c_str());
        _x0.zero();
        kSolver->init(_x0, _P0);
        
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
        //printf(">>>>>>>>>this %08X >>>>>>>>>>>>>>>>>>>> %08x z  %d \n",this, z , z->length());
       
        //printf("setMeasurements: preparing measures \n");
        //printf("_u \n %s \n", _u.toString().c_str());
        //printf("%d %d \n", _u.rows(), _u.cols() );
        
        //delete uMeasure;
        // TODO : check possible memory leak
        uMeasure = new Matrix( _u.rows(), _u.cols());
        zMeasure = new Matrix( _z.rows(), _z.cols());
        //printf("%d %d \n", uMeasure->rows(), uMeasure->cols() );
        //uMeasure(2, 10);
        //resize(2,10);
        
        uMeasure->operator =(_u);
        zMeasure->operator =(_z);
        //printf("this %08x %d %d \n",this, uMeasure->rows(), zMeasure->cols());

        //printf(" uMeasure \n %s  \n",  uMeasure->toString().c_str());
        //printf("_u \n %s  \n", _u.toString().c_str());
        printf("_z \n %s \n zMeasure %s \n", _z.toString().c_str(), zMeasure->toString().c_str());

        mutexR.wait();
        dataReady = true;
        mutexR.post();
        //printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>. just set the dataReady %d \n");

        mutexF.wait();
        evalFinished = false;
        mutexF.post();
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

    ////////////////////////////////////////////////////////////////

    Vector getX() {
        //mutexF.wait();
        //while(!finished) {
        //    mutexF.post();
        //    Time::delay(0.1);
        //    mutexF.wait();
        //}
        //mutexF.post();
        Vector temp;
        mutexData.wait();
        temp = *x;
        mutexData.post();
        return temp;
    }

    ///////////////////////////////////////////////////////////////

    bool getEvalFinished() {
         mutexF.wait();
         bool ef = evalFinished;
         mutexF.post();
         //kSolver->init(z0, x0, P0); // reinitialisation of the filter after the evaluation of the state

         return ef;
    }

     ///////////////////////////////////////////////////////////////

    bool getDataReady() {
         mutexR.wait();
         bool dr = dataReady;
         mutexR.post();

         return dr;
    }

    ///////////////////////////////////////////////////////////////

    double getMSE() {
         mutexMSE.wait();
         double dr = meanSquareError;
         mutexMSE.post();

         return dr;
    }

    ///////////////////////////////////////////////////////////////

    int getRowA() {
         return rowA;
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



