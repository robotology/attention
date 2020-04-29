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

#define THPERIOD 0.01

attentionManagerThread::attentionManagerThread(string moduleName):PeriodicThread(THPERIOD){

    //initialize names
    this->moduleName = moduleName;
    combinedImagePortName = getName("/combinedImage:i");
    hotPointPortName = getName("/hotPoint:o");


    //initialize data
    //combinedImage = nullptr;
    combinedImage = new ImageOf<PixelMono>;


    //initialize processing variables
    attentionProcessState = ATTENTION_PROCESS_STATE::PROCESSING;

}

attentionManagerThread::~attentionManagerThread(){
    delete combinedImage;
}


bool attentionManagerThread::configure(yarp::os::ResourceFinder &rf){
    thresholdVal = rf.findGroup("processingParam").check("threshold",    yarp::os::Value(200), "the threshold value to execute the action").asDouble();
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

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}
void attentionManagerThread::run() {

    if (attentionProcessState == ATTENTION_PROCESS_STATE::PROCESSING){
        if(combinedImagePort.getInputCount()){

            combinedImage = combinedImagePort.read(true);
            if(combinedImage!=NULL){
                combinedImageMat = toCvMat(*combinedImage);
                cv::minMaxLoc( combinedImageMat, &minValue, &maxValue, &idxOfMin, &idxOfMax );
                if(maxValue > thresholdVal){
                    if(!sendMaxPointToLinker(idxOfMax)){
                        yDebug("max point port not connected to any output");
                    }
                }
            }
        }
    }

}

void attentionManagerThread::threadRelease() {

    //-- Stop all threads.
    combinedImagePort.interrupt();
    hotPointPort.interrupt();


    //-- Close the threads.
    combinedImagePort.close();
    hotPointPort.close();

}


string attentionManagerThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}

bool attentionManagerThread::sendMaxPointToLinker(cv::Point maxPoint) {
    if(hotPointPort.getOutputCount()){
        Bottle msg;
        Bottle& coordinatesList = msg.addList();
        coordinatesList.addInt(maxPoint.x);
        coordinatesList.addInt(maxPoint.y);
        hotPointPort.prepare() = msg;
        hotPointPort.write();
        return true;
    }
    return false;
}

void attentionManagerThread::resetAttentionState() {
    attentionProcessState = ATTENTION_PROCESS_STATE::PROCESSING;
}

void attentionManagerThread::suspendAttentionState() {
    attentionProcessState = ATTENTION_PROCESS_STATE::SUSPENDED;

}


