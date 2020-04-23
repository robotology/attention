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
    getCartesianCoordinatesPortName = getName("/getCartesianCoordinates:oi");
    pointActionPortName = getName("/pointAction:oi");

    //initialize data
    //combinedImage = nullptr;
    combinedImage = new ImageOf<PixelMono>;


    //initialize processing variables
    attentionProcessState = ATTENTION_PROCESS_STATE::PROCESSING;
    float xCartOfMax = 0;
    float yCartOfMax = 0;
    float zCartOfMax = 0;

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

    if (!getCartesianCoordinatesPort.open(getCartesianCoordinatesPortName.c_str())) {
        yError("Unable to open /getCartesianCoordinate:oi port ");
        return false;
    }

    if (!pointActionPort.open(pointActionPortName.c_str())) {
        yError("Unable to open /pointAction:oi port ");
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
                    yInfo(to_string(maxValue).c_str());
                    expectCartesian3dLocation(idxOfMax.x,idxOfMax.y,xCartOfMax,yCartOfMax,zCartOfMax);
                }
            }
        }
    }

}

void attentionManagerThread::threadRelease() {

    //-- Stop all threads.
    combinedImagePort.interrupt();
    getCartesianCoordinatesPort.interrupt();
    pointActionPort.interrupt();


    //-- Close the threads.
    combinedImagePort.close();
    getCartesianCoordinatesPort.close();
    pointActionPort.close();

}


string attentionManagerThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}

bool attentionManagerThread::expectCartesian3dLocation(int u,int v,float &x,float &y, float &z) {
    if(getCartesianCoordinatesPort.getOutputCount()){
        Bottle command;
        Bottle reply;
        command.addVocab(CMD_GET);
        command.addVocab(GET_S2C);
        Bottle &target = command.addList();
        target.addString("left");
        target.addInt(u);
        target.addInt(v);
        getCartesianCoordinatesPort.write(command,reply);
        if(reply.get(0).asList()->size() == 3){
            x = reply.get(0).asList()->get(0).asFloat64();
            y = reply.get(0).asList()->get(0).asFloat64();
            z = reply.get(0).asList()->get(0).asFloat64();
            string responced = "responce: "
                               + to_string(reply.get(0).asList()->get(0).asFloat64())  +  "  "
                               + to_string(reply.get(0).asList()->get(1).asFloat64())  +  "  "
                               + to_string(reply.get(0).asList()->get(2).asFloat64());
            yInfo(responced.c_str());
            return true;
        }else{
            yInfo("No ACK");
        }
    }
    return false;
}

bool attentionManagerThread::pointToCartesian3dLocation(float x, float y, float z) {
    if(pointActionPort.getOutputCount()){
        Bottle command;
        Bottle reply;
        command.addVocab(CMD_POINT);
        Bottle &target = command.addList();
        target.addString("cartesian");
        target.addFloat64(x);
        target.addFloat64(y);
        target.addFloat64(z);

        pointActionPort.write(command,reply);
        if(reply.get(0).asVocab()==ACK)
            return true;
    }
    return false;
}

bool attentionManagerThread::isInsideTheBoard(float x, float y, float z) {
    return false;
}

bool attentionManagerThread::refineLocation(float &x, float &y, float &z) {
    return false;
}


