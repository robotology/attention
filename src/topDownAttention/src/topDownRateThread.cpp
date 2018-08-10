// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file topDownRateThread.cpp
 * @brief Implementation of the eventDriven thread (see topDownRateThread.h).
 */

#include "../include/iCub/topDownRateThread.h"
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100 //ms

topDownRateThread::topDownRateThread(yarp::os::ResourceFinder &rf):RateThread(THRATE) {
    robot = "icub";

    outputPortBiologicalMotion =  rf.check("outputBiologicalMotion",
                                          Value("/oneMotionFeatExtractor/gmResult:o"),
                                          "module name (string)").asString();

    inputPortCartesianMap =  rf.check("inputCartesianMap",
                                           Value("/selectiveAttentionEngine/icub/left_cam/cart2:i"),
                                           "module name (string)").asString();
}

topDownRateThread::topDownRateThread(yarp::os::ResourceFinder &rf, string _robot, string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;

    outputPortBiologicalMotion =  rf.check("outputBiologicalMotion",
                                           Value("/oneMotionFeatExtractor/gmResult:o"),
                                           "module name (string)").asString();

    inputPortCartesianMap =  rf.check("inputCartesianMap",
                                      Value("/selectiveAttentionEngine/icub/left_cam/cart2:i"),
                                      "module name (string)").asString();
}

topDownRateThread::~topDownRateThread() {
    // do nothing
}

bool topDownRateThread::threadInit() {
    // opening the port for direct input
    if (!inputPortGazeCtrl.open(getName("/iKinGazeState:i").c_str())) {
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!outputPort.open(getName("/img:o").c_str())) {
        yError(": unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    outputImage = new ImageOf<yarp::sig::PixelRgb>();

    yInfo("Initialization of the processing thread correctly ended");

    restartMotion = false;

    return true;
}

void topDownRateThread::setName(string str) {
    this->name=str;
}


std::string topDownRateThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void topDownRateThread::setInputPortName(string InpPort) {
    
}

void topDownRateThread::run() {

    if (inputPortGazeCtrl.getInputCount()) {
        Bottle *gazeState = inputPortGazeCtrl.read();   //blocking reading for synchr with the input
        const string iKinGazeState = gazeState->get(0).asString();

        if( iKinGazeState == "motion-onset"){
            const bool resultDisconnection = NetworkBase::disconnect(outputPortBiologicalMotion, inputPortCartesianMap);

            if (outputPort.getOutputCount()) {
                *outputImage = outputPort.prepare();
                //outputImage->resize(inputImage->width(), inputImage->height());
                outputImage->zero();
                outputPort.write();

                restartMotion = true;
            }
            yInfo("Disconnecting biological motion detection to attention mechanism");

        }

        else if( iKinGazeState == "motion-done"){
            if (outputPort.getOutputCount()  && restartMotion) {
                *outputImage = outputPort.prepare();
                //outputImage->resize(inputImage->width(), inputImage->height());
                outputImage->zero();
                outputPort.write();

                restartMotion = false;
            }
            const bool resultConnection = NetworkBase::connect(outputPortBiologicalMotion, inputPortCartesianMap);
            yInfo("Reconnecting biological motion detection to attention mechanism");

        }


    }



}

bool topDownRateThread::processing(){
    // here goes the processing...
    return true;
}


void topDownRateThread::threadRelease() {
    // nothing
    inputPortGazeCtrl.interrupt();
    outputPort.interrupt();
    inputPortGazeCtrl.close();
    outputPort.close();
}


