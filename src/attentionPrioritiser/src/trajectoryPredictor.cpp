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
using namespace std;

#define THRATE 10

trajectoryPredictor::trajectoryPredictor() {
    tracker = 0;
}

trajectoryPredictor::~trajectoryPredictor() {
    
}

bool trajectoryPredictor::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    string rootName("");
    rootName.append(getName("/blobImage:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inImagePort.open(rootName.c_str());


    //printf("starting the tracker in the trajectoryPredictor.... \n");
    //ResourceFinder* rf = new ResourceFinder();
    //tracker = new trackerThread(*rf);
    //tracker->setName(getName("/matchTracker").c_str());
    //tracker->start();
    //printf("trajectoryPredictor::threadInit:end of the threadInit \n");
    
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

bool trajectoryPredictor::estimateVelocity(int x, int y, double& Vx, double& Vy, double& xPos, double& yPos) {
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
    double velX, velY;
    double meanVelX, meanVelY;
    int distX, distY;
    int nIter = 3;
    
    //for n times records the position of the object and extract an estimate
    if(true) {
        printf("entering the for necessary to perform high level tracking \n");
        for (short n = 0; n < nIter; n++) {
            p_prev =  p_curr;
            tracker->getPoint(p_curr);
            timeStop = Time::now();
            
            if (n > 0) {
                timeDiff = timeStop - timeStart;
                printf("timeDiff %f \n", timeDiff );
                distX = p_curr.x - p_prev.x;
                distY = p_curr.y - p_prev.y;
                printf("distance X: %d \n", distX);
                printf("distance Y: %d \n", distY);
                velX = distX / timeDiff;
                velY = distY / timeDiff;
                printf("velocity X: %f \n", velX);
                printf("velocity Y: %f \n", velY);
                meanVelX += velX;
                meanVelY += velY;
            }
            timeStart = Time::now();
            Time::delay(0.1);
        }
        
        meanVelX /= nIter;
        meanVelY /= nIter;
        
    }

    bool predictionAccompl = true;
    Vx = meanVelX;
    Vy = meanVelY;
    xPos = -1;
    yPos = -1;
    return predictionAccompl;
}

void trajectoryPredictor::run() {
    
    while(isRunning()){
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
}

void trajectoryPredictor::threadRelease() {
    //printf("trajectoryPredictor::threadRelease() : \n");
    //inImagePort.close();
    if(0 != tracker) {
        printf("trajectoryPredictor::threadRelease:stopping the tracker \n");
        tracker->stop();
    }
    
}
