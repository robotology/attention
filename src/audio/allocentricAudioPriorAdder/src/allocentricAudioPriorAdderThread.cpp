// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C)2020  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Omar Eldardeer
  * email: omar.eldardeer@iit.it
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
#include "iCub/allocentricAudioPriorAdderThread.h"

#define THPERIOD 0.01

allocentricAudioPriorAdderThread::allocentricAudioPriorAdderThread(string moduleName):PeriodicThread(THPERIOD){

    this->moduleName = moduleName;
    inputProbabilityAngleMapPortName = getName("/probabilityAngleMap:i");
    inputRawPowerPortName = getName("/rawPower:i");
    outputNormalizedAngleMapPortName = getName("/normalizedAngleMap:o");
    outputCutAngleMapPortName = getName("/cutAngleMap:o");
    outputSaliencyAngleMapPortName= getName("/saliencyAngleMap:o");
}

allocentricAudioPriorAdderThread::~allocentricAudioPriorAdderThread(){

}


string allocentricAudioPriorAdderThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}


bool allocentricAudioPriorAdderThread::configure(yarp::os::ResourceFinder &rf){

    rawPowerThreshold = rf.findGroup("powerPriors").check("rawThreshold",    yarp::os::Value(0.9), "the threshold of the total raw power (double)").asDouble();
    sideWindowWidth = rf.findGroup("anglePriors").check("windowSideWidth",    yarp::os::Value(2), "the width of the prior angles for each side (int)").asInt();
    priorAngles  = rf.findGroup("anglePriors").findGroup("anglesIdx").tail();
    saliencyGain = rf.findGroup("saliencyTransformation").check("saliencyGain",    yarp::os::Value(1), "the threshold of the total raw power (double)").asDouble();



    priorAnglesCount = priorAngles.size();
    for(int i = 0; i< priorAnglesCount;i++){
        priorAnglesIdxList.push_back(priorAngles.get(i).asInt()+180);
    }

    probabilityAngleMapMatrix.resize(1,360);
    probabilityAngleMapMatrix.zero();


    rawPowerMatrix.resize(2,1);
    rawPowerMatrix.zero();

    normalizedAngleMapMatrix.resize(1, 360);
    normalizedAngleMapMatrix.zero();

    cutAngleMapMatrix.resize(1, 360);
    cutAngleMapMatrix.zero();

    saliencyPowerNormalizedAngeMatrix.resize(1, 360);
    saliencyPowerNormalizedAngeMatrix.zero();

    saliencyPowerNormalizedAngelImg->resize(1, 360);
    saliencyPowerNormalizedAngelImg->zero();

    return true;
}


bool allocentricAudioPriorAdderThread::threadInit() {

    /* ===========================================================================
	 *  Initialize all ports. If any fail to open, return false to
	 *    let RFModule know initialization was unsuccessful.
	 * =========================================================================== */

    if (!inputProbabilityAngleMapPort.open(inputProbabilityAngleMapPortName.c_str())) {
        yError("Unable to open port /probabilityAngleMap:i to receive input.");
        return false;
    }
    if (!inputRawPowerPort.open(inputRawPowerPortName.c_str())) {
        yError("Unable to open port /rawPower:i for receiving audio raw power");
        return false;
    }

    if (!outputNormalizedAngleMapPort.open(outputNormalizedAngleMapPortName.c_str())) {
        yError("Unable to open port /normalizedAngleMap:o to send output Normalized angle map.");
        return false;
    }

    if (!outputCutAngleMapPort.open(outputCutAngleMapPortName.c_str())) {
        yError("Unable to open port /cutAngleMap:o to send output cut angle map.");
        return false;
    }

    if (!outputSaliencyPowerNormalizedAngelPort.open(outputSaliencyAngleMapPortName.c_str())) {
        yError("Unable to open port /saliencyAngleMap:o to send output saliency angle map.");
        return false;
    }

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}

void allocentricAudioPriorAdderThread::threadRelease() {

    //-- Stop all threads.
    inputProbabilityAngleMapPort.interrupt();
    inputRawPowerPort.interrupt();
    outputNormalizedAngleMapPort.interrupt();
    outputCutAngleMapPort.interrupt();
    outputSaliencyPowerNormalizedAngelPort.interrupt();

    //-- Close the threads.
    inputProbabilityAngleMapPort.close();
    inputRawPowerPort.close();
    outputNormalizedAngleMapPort.close();
    outputCutAngleMapPort.close();
    outputSaliencyPowerNormalizedAngelPort.close();

}

void allocentricAudioPriorAdderThread::run() {

    if (inputProbabilityAngleMapPort.getInputCount()) {
        //-- Get Input.
        probabilityAngleMapMatrix = *inputProbabilityAngleMapPort.read(true);
    }

    if (inputRawPowerPort.getInputCount()) {
        //-- Get Input.
        rawPowerMatrix = *inputRawPowerPort.read(true);
        rawPowerTotal = rawPowerMatrix[0][0] +  rawPowerMatrix[1][0];
    }


    double sumTemp = 0;
    for(int i = 0;i<priorAnglesCount;i++){
        for(int j = -1*sideWindowWidth;j<=sideWindowWidth;j++){
            sumTemp += probabilityAngleMapMatrix[0][priorAnglesIdxList.at(i) + j] ;
        }
    }
    for(int i = 0;i<priorAnglesCount;i++){
        for(int j = -1*sideWindowWidth;j<=sideWindowWidth;j++){
            saliencyPowerNormalizedAngeMatrix[0][priorAnglesIdxList.at(i) + j]   = probabilityAngleMapMatrix[0][priorAnglesIdxList.at(i) + j]  / sumTemp * 255 * saliencyGain * rawPowerTotal;
            if(rawPowerTotal>rawPowerThreshold){
                normalizedAngleMapMatrix[0][priorAnglesIdxList.at(i) + j]   = probabilityAngleMapMatrix[0][priorAnglesIdxList.at(i) + j]  / sumTemp;
                cutAngleMapMatrix[0][priorAnglesIdxList.at(i) + j]  = probabilityAngleMapMatrix[0][priorAnglesIdxList.at(i) + j] ;
            }
            else{
                normalizedAngleMapMatrix.zero();
                cutAngleMapMatrix.zero();
            }
        }
    }


    publishOutPorts();
}


void allocentricAudioPriorAdderThread::publishOutPorts() {

    //-- Write to Active Ports.
    if (outputNormalizedAngleMapPort.getOutputCount()) {
        outputNormalizedAngleMapPort.prepare() = normalizedAngleMapMatrix;
        outputNormalizedAngleMapPort.write();
    }

    if (outputCutAngleMapPort.getOutputCount()) {
        outputCutAngleMapPort.prepare() = cutAngleMapMatrix;
        outputCutAngleMapPort.write();
    }

    if (outputSaliencyPowerNormalizedAngelPort.getOutputCount()) {
        saliencyPowerNormalizedAngelImg = &outputSaliencyPowerNormalizedAngelPort.prepare();
        saliencyPowerNormalizedAngelImg->resize(1,360);
        saliencyPowerNormalizedAngelImg->zero();
        unsigned char* rowImage = saliencyPowerNormalizedAngelImg->getRawImage();
        for(int i = 0;i<priorAnglesCount;i++){
            for(int j = -1*sideWindowWidth;j<=sideWindowWidth;j++){
                rowImage[priorAnglesIdxList.at(i) + j]  = saliencyPowerNormalizedAngeMatrix[0][priorAnglesIdxList.at(i) + j] ;
            }
        }
        outputSaliencyPowerNormalizedAngelPort.write();
    }

}
