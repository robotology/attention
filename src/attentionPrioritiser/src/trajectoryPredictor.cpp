// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file trajectoryPredictor.cpp
 * @brief Implementation of the thread of trajectory predictor(see header trajectoryPredictor.h)
 */


#include <iCub/trajectoryPredictor.h>

#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace attention::predictor;
using namespace attention::evaluator;
using namespace std;

#define THRATE 10

trajectoryPredictor::trajectoryPredictor() {
    tracker = 0;
    eQueue = new evalQueue();
}

trajectoryPredictor::~trajectoryPredictor() {
    delete eQueue;
}

bool trajectoryPredictor::threadInit() {
    printf("-------------------------------trajectoryPredictor::threadInit:starting the thread.... \n");
    
    

    /* open ports */
    string rootName("");
    rootName.append(getName("/blobImage:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inImagePort.open(rootName.c_str()); 
    
    
    printf(" \n \n ----------------- trajectoryPredictor::threadInit --------------------- \n");
    evalQueue::iterator it;
    evalThread* tmp;
    it = eQueue->begin();
    printf("got the pointer to the evalThread %08x \n", (*it));
   
    //Vector xCheck = (*it)->getX();
    //printf(" xCheck = \n %s \n", xCheck.toString().c_str());
    printf("---------------------------------------------------------------------------------\n");
    
    /*
    minJerkModel* modelC = new minJerkModel();
    modelC->init(1, 1);
    printf("modelC\n %s \n %s \n", modelC->getA().toString().c_str(), modelC->getB().toString().c_str());
    genPredModel* mC = dynamic_cast<genPredModel*>(modelC);
    evalThread etC(*mC);
    evalMJ1_T1 = etC;
    evalMJ1_T1.start();
    eQueue->push_back(&evalMJ1_T1);  
    */

    // ---------------------------------------------------------------------------
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
   
    printf(" creating eval thread \n");
    eval = new evalThread(*mB);
    eval->init(z0,x0,P0);
    printf("genPred model A \n %s \n",mB    ->getA().toString().c_str());
    printf("lin acc model A \n %s \n",modelB->getA().toString().c_str());
    printf("just initialised genPredModel %08X \n",&eval);
    eval->start();
    eQueue->push_back(eval); 
    

    //------------------------------------------------------------------------------
    printf("moving to the next predictor \n");
    modelB = new linAccModel();
    modelB->init(2.0);
    mB = dynamic_cast<genPredModel*>(modelB);
    eval = new evalThread(*mB);
    eval->init(z0,x0,P0);
    eval->start();
    eQueue->push_back(eval); 
    
    printf("------------------- trajectoryPredictor::threadInit: success in the initialisation \n");
        
    return true;
}

void trajectoryPredictor::interrupt() {
    inImagePort.interrupt();
}

void trajectoryPredictor::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string trajectoryPredictor::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void trajectoryPredictor::isPredict(bool& value) {
    mutex.wait();
    value = predictionAccompl;
    mutex.post();
}


void trajectoryPredictor::extractCentroid(yarp::sig::ImageOf<yarp::sig::PixelMono>* image, int& x, int& y) {
    x = 10.0;
    y = 10.0;
}

bool trajectoryPredictor::estimateVelocity(int x, int y, double& Vx, double& Vy, double& xPos, double& yPos, double& time, double& distance) {
    printf(" trajectoryPredictor::estimateVelocity in pos.%d,%d  \n", Vx, Vy);
    
    CvPoint p_curr, p_prev;

    double timeStart = Time::now();
    double timeStop, timeDiff;
    double dist, vel, acc;
    double velX, velY, velX_prev, velY_prev;
    double accX, accY, maxAccX = 0, maxAccY = 0;
    double meanVelX, meanVelY;
    int distX, distY;
    
    int nIter = 10;
    
    //for n times records the position of the object and extract an estimate
    // extracting the velocity of the stimulus; saving it into a vector 
    Matrix zMeasurements(nIter,2.0);
    Matrix uMeasurements(2.0, nIter);
    printf("entering the loop for necessary to perform high level tracking \n");
    for (short n = 0; n < nIter; n++) {
        p_prev =  p_curr;
        tracker->getPoint(p_curr);
        timeStop = Time::now();
        
        if (n > 0) {
            timeDiff = timeStop - timeStart;
            //printf("----------------- \n timeDiff %f \n", timeDiff );
            distX = p_curr.x - p_prev.x;
            distY = p_curr.y - p_prev.y;
            dist  = sqrt((double)distX * distX + distY * distY);
            zMeasurements(n - 1, 0) = dist;

            velX_prev = velX;
            velY_prev = velY;
            velX = distX / timeDiff;
            velY = distY / timeDiff;
            vel = sqrt( velX * velX + velY * velY);
            zMeasurements(n - 1, 1) = vel;

            accX = (velX - velX_prev) / timeDiff;
            accY = (velY - velY_prev) / timeDiff;
            acc = sqrt( accX * accX + accY * accY);
            //zMeasurements(n - 1, 2) = acc;

            if(accY > maxAccY) { 
                maxAccY = accY;
            }
            if(accX > maxAccX) {
                maxAccX = accX;
            }
            meanVelX += velX;
            meanVelY += velY;
        }
        timeStart = Time::now();
        Time::delay(0.05);
    }
    
    meanVelX /= nIter;
    meanVelY /= nIter;

    uMeasurements(1,0) = 2.0; uMeasurements(0,0) = 4.0;
    zMeasurements(0,0) = 3.0; zMeasurements(1,1) = 1.0;
    
    //estimate the predictor model that best fits the velocity measured
    //printf("setting measurements \n z = \n %s \n", zMeasurements.toString().c_str());
    //printf("u = \n %s \n", uMeasurements.toString().c_str());
    
    //eval->setMeasurements(uMeasurements,zMeasurements);     // alternative for debug

    // pointer to the beginning of the evalQueue
    evalQueue::iterator it;
    evalThread* tmp;
    it = eQueue->begin();
    //printf("got the pointer to the evalThread %08x \n", (*it));
    //Vector xCheck = (*it)->getX();
    //printf(" xCheck = \n %s \n", xCheck.toString().c_str());
    
    
    //tmp = *it;  // copy using pointer to the thread
    //tmp->setMeasurements(uMeasurements,zMeasurements);
    //printf("entering the loop for %08X with getdatReady %d \n",tmp, tmp->getDataReady());

    
    //starting different evalution threads
    while(it != eQueue->end() ) { 
        printf("____________________________________________________________________________________________________\n");
        tmp = *it;  // pointer to the thread
        printf("reading evalThread reference from the queue it = %08X \n", tmp);
        tmp->setMeasurements(uMeasurements,zMeasurements);
        printf("entering the loop with getdatReady %d \n", tmp->getDataReady());
        printf("getEvalFineshed value %d \n", tmp->getEvalFinished());
        it++;   
        printf("____________________________________________________________________________________________________\n \n \n");
    }

    printf("out of the loop \n");


    // waiting for the evaluation already started
    printf("entering the loop with eval \n");
    printf("---------------------------- GETEVALFINISHED %d \n",eval->getEvalFinished() );
    it = eQueue->begin();
    int finished = 0 ;
    while(finished < eQueue->size()) {
        printf("eval evaluation %d < %d \n",finished, eQueue->size() );
        Time::delay(0.1);
        while(it != eQueue->end() ) {
            if((*it)->getEvalFinished()){
                finished++;
                printf("state %s \n", (*it)->getX().toString().c_str());
            }
            it++;
        }
        it = eQueue->begin();
    }
    printf("eval evaluation %d >= %d \n",finished, eQueue->size() );
    printf("---------------------------- GETEVALFINISHED %d \n",eval->getEvalFinished() );

    
    tracker->getPoint(p_curr);
    distance = std::sqrt((double)(p_curr.x - 160) * (p_curr.x - 160) + (p_curr.y - 120) * (p_curr.y - 120));

    bool predictionAccompl = true;
    Vx = meanVelX;
    Vy = meanVelY;
    xPos = -1;
    yPos = -1;
   
    double maxAcc = maxAccX > maxAccY?maxAccX:maxAccY;
    //time = maxAcc / 5000; 
    time = 0.18;
    return predictionAccompl;
}

void trajectoryPredictor::run() {
    printf(" trajectoryPredictor::run %d %d \n", numIter, numEvalVel); 

    //Time::delay(5.0);
    
    
    
    //it2 = eQueue->begin();
    //printf("got the pointer to the evalThread %08x \n", (*it2));
    //Vector xCheck2 = (*it2)->getX();
    //printf(" xCheck2 = \n %s \n", xCheck2.toString().c_str());
    

    // trajectory predictor does not need active run anymore.
    // estimateVelocity function is called from the att.Prioritiser
    
    while(!isStopping()){

        /*
        it = eQueue->begin();
        printf("got the pointer %d  to the evalThread in run %08x \n",eQueue->size(),(*it));
        Vector xCheck = (*it)->getX();
        printf(" xCheck = \n %s \n", xCheck.toString().c_str());

        
        //ImageOf<PixelMono>* b = inImagePort.read(true);
        //  printf("after the imagePort \n");
        */
        
        /*
        printf("trajectoryPreditctor in run %d %d \n", numIter, numEvalVel);
        evalQueue::iterator it, it2;
        //it = eQueue->begin();
        //printf("got the pointer to the evalThread %08x \n", (*it));
        //Vector xCheck = (*it)->getX();
        //printf(" xCheck = \n %s \n", xCheck.toString().c_str());

        Vector xTraj = eval->getX();
        printf(" xTraj = \n %s \n", xTraj.toString().c_str());
        */
        
        //evalQueue::iterator it;
        //evalThread* tmp;
        //it = eQueue->begin();
        //printf("got the pointer to the evalThread %08x \n", (*it));
        //Vector xCheck = (*it)->getX();
        //printf(" xCheck = \n %s \n", xCheck.toString().c_str());
        
        
        /*
        // estimating velocity
        int x,y;
        double Vx, Vy, xPos, yPos, time, distance;
        //extractCentroid(b, x, y);
        estimateVelocity(x, y, Vx, Vy, xPos, yPos, time, distance);
        printf("estimateVelocity %f %f \n",Vx,Vy );
        */
        
        Time::delay(5.0);
    
    }

    
    
}

void trajectoryPredictor::onStop() {
    printf("trajectoryPredictor::onStop() : closing ports \n");
    inImagePort.interrupt();
    inImagePort.close();
    printf("trajectoryPredictor::onStop() : success in closing ports \n");
}

void trajectoryPredictor::threadRelease() {
    printf("trajectoryPredictor::threadRelease() : \n");
    //inImagePort.close();

    //if(0 != tracker) {
    //    printf("trajectoryPredictor::threadRelease:stopping the tracker \n");
    //    tracker->stop();
    //}

    //evalVel1.stop();
    //evalAcc1.stop();
    //evalMJ1_T1.stop();
    
}
