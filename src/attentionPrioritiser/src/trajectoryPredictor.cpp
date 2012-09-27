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
    Matrix zMeasurements(nIter,3.0);
    Matrix uMeasurements(nIter,3.0);
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
            dist  = sqrt( distX * distX + distY * distY);
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
            zMeasurements(n - 1, 2) = acc;

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

    uMeasurements(1,0) = 2.0; uMeasurements(2,0) = 4.0;
    zMeasurements(2,0) = 3.0; zMeasurements(1,1) = 1.0;
    
    //estimate the predictor model that best fits the velocity measured
    printf("setting measurements \n z = %s \n", zMeasurements.toString().c_str());
    printf("setting measurements \n u = %s \n", uMeasurements.toString().c_str());
    deque<evalThread*>::iterator it;
    evalThread* tmp;
    it = eQueue->begin();
    tmp = *it;  // pointer to the thread
    tmp->setMeasurements(uMeasurements,zMeasurements);
    printf("entering the loop for %08X with getdatReady %d \n",tmp, tmp->getDataReady());
    /*
    while(it != eQueue->end() ) { 
        printf("reading evalThread reference from the queue \n");
        tmp = *it;  // pointer to the thread
        tmp->setMeasurements(uMeasure,zMeasure);
        printf("entering the loop with getdatReady %d \n", tmp->getDataReady());
        //while(!(*it)->getEvalFinished()) {
        //    //printf("evalVel1 evaluation \n");
        //    Time::delay(0.1);
        //}
        
        printf("out of the loop \n");
        it++;
        
    }
*/
    
    tracker->getPoint(p_curr);
    distance = sqrt((p_curr.x - 160) * (p_curr.x - 160) + (p_curr.y - 120) * (p_curr.y - 120));

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
    
    while(!isStopping()){    
        printf("trajectoryPredictor::run \n");       
        ImageOf<PixelMono>* b = inImagePort.read(true);
        printf("after the imagePort \n");
        int x,y;
        double Vx, Vy, xPos, yPos, time, distance;
        extractCentroid(b, x, y);
        estimateVelocity(x, y, Vx, Vy, xPos, yPos, time, distance);
        printf("estimateVelocity %f %f \n",Vx,Vy );
        
        Time::delay(1.0);
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
