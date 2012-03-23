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

bool trajectoryPredictor::estimateVelocity(int x, int y, double& Vx, double& Vy) {
    predictionAccompl = true;
    Vx = 10.0;
    Vy = 10.0;
    return predictionAccompl;
}

void trajectoryPredictor::run() {
    while(isStopping() != true){
        ImageOf<PixelMono>* b=inImagePort.read(true);
        int x,y;
        extractCentroid(b, x, y);
        estimateVelocity(x,y, Vx, Vy);
        printf("estimateVelocity %f %f \n",Vx,Vy );
    }
}

void trajectoryPredictor::onStop() {
    inImagePort.interrupt();
    inImagePort.close();
}

void trajectoryPredictor::threadRelease() {

}
