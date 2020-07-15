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
#include "iCub/attentionManagerThread.h"

using namespace yarp::sig;
using namespace yarp::cv;
using namespace attention::dictionary;

#define THPERIOD 0.01

attentionManagerThread::attentionManagerThread(string moduleName):PeriodicThread(THPERIOD){

    //initialize names
    this->moduleName = moduleName;
    combinedImagePortName = getName("/combinedImage:i");
    hotPointPortName = getName("/hotPoint:o");
    engineControlPortName =  getName("/engineControl:oi");
    gazeArbiterControlPortName = getName("/gazeArbiterControl:oi");


    //initialize data
    combinedImage = new ImageOf<PixelRgb>;


    //initialize processing variables

}

attentionManagerThread::~attentionManagerThread(){
    delete combinedImage;
}


bool attentionManagerThread::configure(yarp::os::ResourceFinder &rf){
    max_thresholdVal = rf.findGroup("processingParam").check("max_threshold",    yarp::os::Value(200), "the threshold value to execute the action").asDouble();
    return true;
}


bool attentionManagerThread::threadInit() {

    /* ===========================================================================
	 *  Initialize all ports. If any fail to open, return false to
	 *    let RFModule know initialization was unsuccessful.
	 * =========================================================================== */

    if (!combinedImagePort.open(combinedImagePortName.c_str())) {
        yError("Unable to open /combinedImage:i port ");
        return false;
    }
    if (!hotPointPort.open(hotPointPortName.c_str())) {
        yError("Unable to open /hotPoint:o port ");
        return false;
    }

    if (!engineControlPort.open(engineControlPortName.c_str())) {
        yError("Unable to open /engineControl:oi port ");
        return false;
    }
    if (!gazeArbiterControlPort.open(gazeArbiterControlPortName.c_str())) {
        yError("Unable to open /gazeArbiterControl:oi port ");
        return false;
    }

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}
void attentionManagerThread::run() {
    if(combinedImagePort.getInputCount()){
        combinedImage = combinedImagePort.read(true);
        if(combinedImage!=NULL){
            unsigned char* pImage = combinedImage->getRawImage();
            maxValue = 0;
            vector<unsigned char> imageMatrix;
            float sumImageMatrixMinusMeanSqared = 0;
            for(int y = 0;y<240;y++){
                for(int x = 0;x<320;x++){
                    pImage++;
                    pImage++;
                    imageMatrix.push_back(*pImage);
                    if(*pImage > maxValue){

                        maxValue = *pImage;
                        idxOfMax.x = x;
                        idxOfMax.y = y;
                    }
                    pImage++;
                }
            }
            float meanVal = accumulate(imageMatrix.begin(),imageMatrix.end(),0.0)/(float)imageMatrix.size();
            for(auto & pix: imageMatrix){
                sumImageMatrixMinusMeanSqared  += pow(((float)pix-meanVal),2) ;
            }
            float var = sumImageMatrixMinusMeanSqared/(float)imageMatrix.size();
            float imageStd = sqrt(var);
            yInfo("Max= %d  Mean = %.3f , std = %0.3f var = %.3f  (max-mean)-3s = %0.3f",maxValue,meanVal,imageStd,var,maxValue-meanVal-3*imageStd);
            if(maxValue > max_thresholdVal){
                if(!sendMaxPointToLinker(idxOfMax,maxValue)){
                    yDebug("max point port not connected to any output");
                }
            }
        }
    }
}

void attentionManagerThread::threadRelease() {

    //-- Stop all threads.
    combinedImagePort.interrupt();
    hotPointPort.interrupt();
    engineControlPort.interrupt();
    gazeArbiterControlPort.interrupt();


    //-- Close the threads.
    combinedImagePort.close();
    hotPointPort.close();
    engineControlPort.close();
    gazeArbiterControlPort.close();

}


string attentionManagerThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}

bool attentionManagerThread::sendMaxPointToLinker(cv::Point maxPoint, int val) {
    if(hotPointPort.getOutputCount()){
        Bottle msg;
        Bottle& coordinatesList = msg.addList();
        coordinatesList.addInt(maxPoint.x);
        coordinatesList.addInt(maxPoint.y);
        msg.addInt(val);
        hotPointPort.prepare() = msg;
        hotPointPort.write();
        return true;
    }
    return false;
}

bool attentionManagerThread::resetAttentionState() {
    bool engineState = resumeEngine(); 
    bool arbiterState = resumeArbiter();
    resume();
    if(!engineState)
        yError("Couldn't resume the engine");

    if(!arbiterState)
        yError("Couldn't resume the gaze arbiter");
    
    return (engineState && arbiterState);
}

bool attentionManagerThread::suspendAttentionState() {

    bool engineState = suspendEngine(); 
    bool arbiterState = suspendArbiter();
    suspend();
    if(!engineState)
        yError("Couldn't suspend the engine");
    if(!arbiterState)
        yError("Couldn't suspend the gaze arbiter");
    return (engineState && arbiterState);
}

bool attentionManagerThread::suspendEngine() {
    if(engineControlPort.getOutputCount()){
        Bottle command;
        Bottle reply;
        command.addVocab(COMMAND_VOCAB_SUSPEND);
        engineControlPort.write(command,reply);
        if(reply.get(0).asVocab()==COMMAND_VOCAB_OK)
            return true;
    }
    return false;
}

bool attentionManagerThread::resumeEngine() {
    if(engineControlPort.getOutputCount()){
        Bottle command;
        Bottle reply;
        command.addVocab(COMMAND_VOCAB_RESUME);
        engineControlPort.write(command,reply);
        if(reply.get(0).asVocab()==COMMAND_VOCAB_OK)
            return true;
    }
    return false;
}

bool attentionManagerThread::suspendArbiter() {
    if(gazeArbiterControlPort.getOutputCount()){
        Bottle command;
        Bottle reply;
        command.addVocab(COMMAND_VOCAB_SUSPEND);
        gazeArbiterControlPort.write(command,reply);
        if(reply.get(0).asVocab()==COMMAND_VOCAB_OK)
            return true;
    }
    return false;
}

bool attentionManagerThread::resumeArbiter() {
    if(gazeArbiterControlPort.getOutputCount()){
        Bottle command;
        Bottle reply;
        command.addVocab(COMMAND_VOCAB_RESUME);
        gazeArbiterControlPort.write(command,reply);
        if(reply.get(0).asVocab()==COMMAND_VOCAB_OK)
            return true;
    }
    return false;
}

void attentionManagerThread::setThreshold(int val) {
    max_thresholdVal = val;
}

int attentionManagerThread::getThreshold() {
    return max_thresholdVal;
}


