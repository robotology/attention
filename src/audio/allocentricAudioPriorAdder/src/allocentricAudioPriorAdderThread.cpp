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
    outputSaliencyAngleMapPortName_redLine = getName("/saliencyAngleMapWithRedLine:o");
}

allocentricAudioPriorAdderThread::~allocentricAudioPriorAdderThread(){

}


string allocentricAudioPriorAdderThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}


bool allocentricAudioPriorAdderThread::configure(yarp::os::ResourceFinder &rf){

    rawPowerThreshold = rf.findGroup("powerPriors").check("rawThreshold",    yarp::os::Value(0.9), "the threshold of the total raw power (double)").asFloat32();
    sideWindowWidth = rf.findGroup("anglePriors").check("windowSideWidth",    yarp::os::Value(2), "the width of the prior angles for each side (int)").asInt16();
    priorAngles  = rf.findGroup("anglePriors").findGroup("anglesIdx").tail();
    saliencyGain = rf.findGroup("saliencyTransformation").check("saliencyGain",    yarp::os::Value(1), "the threshold of the total raw power (double)").asFloat32();

    

    priorAnglesCount = priorAngles.size();
    for(int i = 0; i< priorAnglesCount;i++){
        priorAnglesIdxList.push_back(priorAngles.get(i).asInt16()+180);
        avgProbabilitiesList.push_back(0);
    }

    rawPowerTotal = 0;

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

    
    yInfo("Configuration parameters done");

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
    if (!outputSaliencyPowerNormalizedAngel_redLine.open(outputSaliencyAngleMapPortName_redLine.c_str())) {
        yError("Unable to open port /saliencyAngleMapWithRedLine:o to send output saliency angle map.");
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
    outputSaliencyPowerNormalizedAngel_redLine.interrupt();

    //-- Close the threads.
    inputProbabilityAngleMapPort.close();
    inputRawPowerPort.close();
    outputNormalizedAngleMapPort.close();
    outputCutAngleMapPort.close();
    outputSaliencyPowerNormalizedAngelPort.close();
    outputSaliencyPowerNormalizedAngel_redLine.close();

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
    maxAngleIdx = 0;
    maxAvg = 0;
    for(int i = 0;i<priorAnglesCount;i++){
        avgProbabilitiesList[i] = 0;
        for(int j = -1*sideWindowWidth;j<=sideWindowWidth;j++){
            avgProbabilitiesList[i] += probabilityAngleMapMatrix[0][priorAnglesIdxList.at(i) + j];
            sumTemp += probabilityAngleMapMatrix[0][priorAnglesIdxList.at(i) + j] ;
        }
        if (avgProbabilitiesList[i] > maxAvg){
            maxAvg = avgProbabilitiesList[i];
            maxAngleIdx = i;
        }
    }
    for(int i = 0;i<priorAnglesCount;i++){
        for(int j = -1*sideWindowWidth;j<=sideWindowWidth;j++){
            if(rawPowerTotal>rawPowerThreshold){
                normalizedAngleMapMatrix[0][priorAnglesIdxList.at(i) + j]   = avgProbabilitiesList[i];
                cutAngleMapMatrix[0][priorAnglesIdxList.at(i) + j]  = probabilityAngleMapMatrix[0][priorAnglesIdxList.at(i) + j] ;
                saliencyPowerNormalizedAngeMatrix[0][priorAnglesIdxList.at(i) + j]   = probabilityAngleMapMatrix[0][priorAnglesIdxList.at(i) + j]  * 255 * saliencyGain * rawPowerTotal;
            }
            else{
                normalizedAngleMapMatrix.zero();
                cutAngleMapMatrix.zero();
                saliencyPowerNormalizedAngeMatrix.zero();
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
        saliencyPowerNormalizedAngelImg->resize(360,1);
        saliencyPowerNormalizedAngelImg->zero();
        unsigned char* rowImage = saliencyPowerNormalizedAngelImg->getRawImage();
        for(int i = 0;i<priorAnglesCount;i++){
            for(int j = -1*sideWindowWidth;j<=sideWindowWidth;j++){
                rowImage[priorAnglesIdxList.at(i) + j]  = (unsigned char) saliencyPowerNormalizedAngeMatrix[0][priorAnglesIdxList.at(i) + j] ;
            }
        }
        outputSaliencyPowerNormalizedAngelPort.write();
    }
    if (outputSaliencyPowerNormalizedAngel_redLine.getOutputCount()) {
        saliencyPowerNormalizedAngelImg_redLine = &outputSaliencyPowerNormalizedAngel_redLine.prepare();
        saliencyPowerNormalizedAngelImg_redLine->resize(360,1);
        saliencyPowerNormalizedAngelImg_redLine->zero();
        unsigned char* rowImage = saliencyPowerNormalizedAngelImg_redLine->getRawImage();
        for(int i = 0;i<priorAnglesCount;i++){
            for(int j = -1*sideWindowWidth;j<=sideWindowWidth;j++){
                rowImage[(priorAnglesIdxList.at(i) + j)*3]  = (unsigned char) saliencyPowerNormalizedAngeMatrix[0][priorAnglesIdxList.at(i) + j] ;
                rowImage[(priorAnglesIdxList.at(i) + j)*3 + 1]  = (unsigned char) saliencyPowerNormalizedAngeMatrix[0][priorAnglesIdxList.at(i) + j] ;
                rowImage[(priorAnglesIdxList.at(i) + j)*3 + 2]  = (unsigned char) saliencyPowerNormalizedAngeMatrix[0][priorAnglesIdxList.at(i) + j] ;

            }
            if(maxAngleIdx == i){
                rowImage[(priorAnglesIdxList.at(i) )*3]      = 255 ;
                rowImage[(priorAnglesIdxList.at(i) )*3 + 1]  = 0 ;
                rowImage[(priorAnglesIdxList.at(i) )*3 + 2]  = 0 ;
            }
        }
        outputSaliencyPowerNormalizedAngel_redLine.write();
    }

}
