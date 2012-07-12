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
}

trajectoryPredictor::~trajectoryPredictor() {
    
}

bool trajectoryPredictor::threadInit() {
    printf("-------------------------------trajectoryPredictor::threadInit:starting the thread.... \n");
    /* open ports */
    string rootName("");
    rootName.append(getName("/blobImage:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inImagePort.open(rootName.c_str()); 
    
    //-----------------------------------------------------------------------------------
    int rowA,colA;

    Matrix R;
    Matrix Q;
    Matrix P0;
    
    Vector z0;
    Vector x0;
    Vector z;
    Vector x;
    Vector u;

    eQueue = new evalQueue(false);
    
    //-------------------------------------------------------------------------------------------------
    printf("Creating prediction models \n");
    linVelModel* modelA = new linVelModel();
    modelA->init(1.0);
    printf("modelA\n %s \n %s \n", modelA->getA().toString().c_str(), modelA->getB().toString().c_str());
    genPredModel* mA = dynamic_cast<genPredModel*>(modelA);
    printf("after dynamic_cast setting the model \n");
    attention::evaluator::evalThread evalVel1;
    evalVel1.setModel(modelA);
    rowA = modelA->getRowA();
    colA = modelA->getColA();
    printf("success in setting the model \n");

    R.resize (rowA,colA);
    Q.resize (rowA,colA);
    P0.resize(rowA,colA);
    
    z0.resize (rowA);
    x0.resize (rowA);
    z.resize (colA);
    x.resize (colA);
    u.resize (1);
    
    printf("preparing the set of measurements \n");
    zMeasure.resize(numIter, rowA);
    uMeasure.resize(numIter, rowA);
    
    for(int j = 0; j < numIter; j++) {
        for (int k  =0 ; k < rowA; k ++) {
            zMeasure(k,j) = 1.0 + Random::uniform();
            uMeasure(k,j) = 1.0 + Random::uniform();
        }
    }
        
    printf("initialising the matrices of the Kalman Filter \n");
    for (int i = 0; i < rowA; i++) {
        for (int j = 0; j < colA; j++) { 
            Q(i, j) += 0.01; 
            R(i, j) += 0.001;
            P0(i,j) += 0.01;
        }      
    }
    
    evalVel1.init(z0, x0, P0);
    evalVel1.start();
    //eQueue.push_back(evalVel1);

    /*
    linAccModel* modelB = new linAccModel();
    modelB->init(1.0);printf("modelB\n %s \n %s \n", modelB->getA().toString().c_str(),modelB->getB().toString().c_str());
    genPredModel* mB = dynamic_cast<genPredModel*>(modelB);
    evalThread etB(mB);
    evalAcc1 = etB;
    evalAcc1.start();
    eQueue.push_back(evalAcc1);
    
    minJerkModel* modelC = new minJerkModel();
    modelC->init(1, 1);
    printf("modelC\n %s \n %s \n", modelC->getA().toString().c_str(), modelC->getB().toString().c_str());
    genPredModel* mC = dynamic_cast<genPredModel*>(modelC);
    evalThread etC(mC);
    evalMJ1_T1 = etC;
    evalMJ1_T1.start();
    eQueue.push_back(evalMJ1_T1);  
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
    
    //// initialisation of the tracker
    //tracker->init(x,y);
    //printf("success in init \n");
    //tracker->r = 1;
    //printf("running %d \n", tracker->r);
    //tracker->waitInitTracker();
    //printf("tracker successfully initialised \n");
    
    double timeStart = Time::now();
    double timeStop, timeDiff;
    double vel;
    double velX, velY, velX_prev, velY_prev;
    double accX, accY, maxAccX = 0, maxAccY = 0;
    double meanVelX, meanVelY;
    int distX, distY;
    int nIter = 10;
    
    //for n times records the position of the object and extract an estimate
    // extracting the velocity of the stimulus; saving it into a vector 
    Vector zMeasurements(nIter,3);
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
            //printf("distance X: %d \n", distX);
            //printf("distance Y: %d \n", distY);
            velX_prev = velX;
            velY_prev = velY;
            velX = distX / timeDiff;
            velY = distY / timeDiff;
            vel = sqrt( velX * velX + velY * velY);
            accX = (velX - velX_prev) / timeDiff;
            accY = (velY - velY_prev) / timeDiff;
            if(accY > maxAccY) maxAccY = accY;
            if(accX > maxAccX) maxAccX = accX;
            //printf("velocity X: %f \n", velX);
            //printf("velocity Y: %f \n", velY);
            //printf("velocity diff X : %f \n",velX - velX_prev );
            //printf("velocity diff Y : %f \n",velY - velY_prev );
            //printf("accelar  X: %f \n", accX);
            //printf("accelar  Y: %f \n", accY);
            meanVelX += velX;
            meanVelY += velY;
        }
        timeStart = Time::now();
        Time::delay(0.05);
    }
    
    meanVelX /= nIter;
    meanVelY /= nIter;

    //estimate the predictor model that best fits the velocity measured
    printf("setting measurements \n");
    
    evalVel1.setMeasurements(uMeasure,zMeasure);
    
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
    
    while(isRunning()){

        printf("trajectory Predictor \n");
        
        /*
        ImageOf<PixelMono>* b=inImagePort.read(true);
        int x,y;
        extractCentroid(b, x, y);
        estimateVelocity(x,y, Vx, Vy);
        printf("estimateVelocity %f %f \n",Vx,Vy );
        */
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

    evalVel1.stop();
    evalAcc1.stop();
    evalMJ1_T1.stop();
    
}
