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
    inputHeadAnglesPortName = getName("/headAngles:i");
    hotPointPortName = getName("/hotPoint:o");
    visualizationPortName = getName("/visualizedImage:o");
    engineControlPortName =  getName("/engineControl:oi");
    gazeArbiterControlPortName = getName("/gazeArbiterControl:oi");
    sceneAnalysisPortName = getName("/sceneAnalysis:o");
    inhibitionControlPortName = getName("/inhibitionControl:oi");

    //initialize data
    combinedImage = new ImageOf<PixelRgb>;
    visualizedImage = new ImageOf<PixelRgb>;


    //initialize processing variables
    lastHotPointTime = Time::now();
    resetRequired = false;


}

attentionManagerThread::~attentionManagerThread(){
    delete combinedImage;
}


bool attentionManagerThread::configure(yarp::os::ResourceFinder &rf){
    max_thresholdVal = rf.findGroup("processingParam").check("max_threshold",    yarp::os::Value(200), "the max threshold value to execute the action").asDouble();
    mean_thresholdVal = rf.findGroup("processingParam").check("mean_threshold", yarp::os::Value(0), "the mean threshold value to execute the action").asFloat64();
    std_thresholdVal = rf.findGroup("processingParam").check("std_threshold",    yarp::os::Value(5), "the std threshold value to execute the action").asFloat64();
    threeSigma_thresholdVal = rf.findGroup("processingParam").check("three_sigma_threshold",    yarp::os::Value(95), "the 3 sigma threshold value to execute the action").asFloat64();
    marginTime = rf.findGroup("resettingInhibition").check("marginTime",    yarp::os::Value(0.5)).asFloat32();


    init_max_thresholdVal = max_thresholdVal;
    init_mean_thresholdVal = mean_thresholdVal;
    init_std_thresholdVal = std_thresholdVal;
    init_threeSigma_thresholdVal = threeSigma_thresholdVal;

    yInfo( " " );
    yInfo( "\t               [processingParam]              "                               );
    yInfo( "\t ============================================ "                                );
    yInfo( "\t max_threshold                : %.3f m",       max_thresholdVal                );
    yInfo("\t mean_threshold                : %.3f m",       mean_thresholdVal               );
    yInfo( "\t std_threshold                : %.3f m",       std_thresholdVal                );
    yInfo( "\t three_sigma_threshold        : %.3f m",       threeSigma_thresholdVal         );
    yInfo( " " );
    yInfo( "\t          [resettingInhibition]               "                               );
    yInfo( "\t ============================================ "                                );
    yInfo( "\t marginTime                : %.3f m",       marginTime                );
    return true;
}


bool attentionManagerThread::threadInit() {

    /* ===========================================================================
	 *  Initialize all ports. If any fail to open, return false to
	 *    let RFModule know initialization was unsuccessful.
	 * =========================================================================== */

    if (!combinedImagePort.open(combinedImagePortName)) {
        yError("Unable to open /combinedImage:i port ");
        return false;
    }
    if (!inputHeadAnglesPort.open(inputHeadAnglesPortName)) {
        yError("Unable to open port %s " , inputHeadAnglesPortName.c_str());
        return false;
    }
    if (!hotPointPort.open(hotPointPortName)) {
        yError("Unable to open /hotPoint:o port ");
        return false;
    }

    if (!engineControlPort.open(engineControlPortName)) {
        yError("Unable to open /engineControl:oi port ");
        return false;
    }
    if (!gazeArbiterControlPort.open(gazeArbiterControlPortName)) {
        yError("Unable to open /gazeArbiterControl:oi port ");
        return false;
    }
    if (!sceneAnalysisPort.open(sceneAnalysisPortName)) {
        yError("Unable to open /sceneAnalysis:o port ");
        return false;
    }

    if (!visualizationPort.open(visualizationPortName)) {
        yError("Unable to open %s  port ", visualizationPortName.c_str());
        return false;
    }

    if (!inhibitionControlPort.open(inhibitionControlPortName)) {
        yError("Unable to open %s port ",inhibitionControlPortName.c_str());
        return false;
    }
    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}
void attentionManagerThread::run() {
    if(checkMovement()){
        resetInhibition();
    }
    if(resetRequired && Time::now() >(lastHotPointTime+marginTime) ){
        if(resetInhibition())
            yInfo("reset inhibition Done");
        else
            yError("reset inhibition error");
    }
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
            meanVal = accumulate(imageMatrix.begin(),imageMatrix.end(),0.0)/(float)imageMatrix.size();
            for(auto & pix: imageMatrix){
                sumImageMatrixMinusMeanSqared  += pow(((float)pix-meanVal),2) ;
            }
            float var = sumImageMatrixMinusMeanSqared/(float)imageMatrix.size();
            stdVal= sqrt(var);
            threeSigmaVal =maxValue-meanVal-3*stdVal;
            publishAnalysis();
            yInfo("Max= %d  Mean = %.3f , std = %0.3f var = %.3f  (max-mean)-3s = %0.3f",maxValue,meanVal,stdVal,var,threeSigmaVal);

            if(visualizationPort.getOutputCount()){
                computeAndPublishVisualizedImage();
            }
//            if((maxValue >= max_thresholdVal) && (meanVal <= mean_thresholdVal) && (stdVal >= std_thresholdVal) && (threeSigmaVal >= threeSigma_thresholdVal)){
//                if(!sendMaxPointToLinker(idxOfMax,maxValue,meanVal,stdVal)){
//                    yDebug("max point port not connected to any output");
//                }
//            }
            if(threeSigmaVal >= threeSigma_thresholdVal){
                if(!sendMaxPointToLinker(idxOfMax,maxValue,meanVal,stdVal)){
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
    sceneAnalysisPort.interrupt();
    inhibitionControlPort.interrupt();
    visualizationPort.interrupt();
    inputHeadAnglesPort.interrupt();


    //-- Close the threads.
    combinedImagePort.close();
    hotPointPort.close();
    engineControlPort.close();
    gazeArbiterControlPort.close();
    sceneAnalysisPort.close();
    inhibitionControlPort.close();
    visualizationPort.close();
    inputHeadAnglesPort.close();

}


string attentionManagerThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}

bool attentionManagerThread::sendMaxPointToLinker(cv::Point maxPoint, int val,float imgMean,float imgStd){
    if(hotPointPort.getOutputCount()){
        Bottle msg;
        Bottle& coordinatesList = msg.addList();
        coordinatesList.addInt(maxPoint.x);
        coordinatesList.addInt(maxPoint.y);
        msg.addInt(val);
        msg.addFloat64(imgMean);
        msg.addFloat64(imgStd);
        hotPointPort.prepare() = msg;
        hotPointPort.write();

        // reset signal trigger
        lastHotPointTime = Time::now();
        resetRequired = true;

        return true;
    }
    return false;
}

bool attentionManagerThread::resetAttentionState(int statType) {
    bool ret = false;
    switch (statType){
        case 1:
            ret = resumeEngine();
            if(!ret)
                yError("Couldn't resume the engine");
            break;
        case 2:
            ret = resumeArbiter();
            if(!ret)
                yError("Couldn't resume the gaze arbiter");
            break;
        default:
            bool engineState = resumeEngine();
            bool arbiterState = resumeArbiter();
            resume();
            if(!engineState)
                yError("Couldn't resume the engine");

            if(!arbiterState)
                yError("Couldn't resume the gaze arbiter");

            ret= (engineState && arbiterState);
            break;
    }
    return ret;

}

bool attentionManagerThread::suspendAttentionState(int statType) {
    bool ret = false;
    switch (statType){
        case 1:
            ret = suspendEngine();
            if(!ret)
                yError("Couldn't suspend the engine");
            break;
        case 2:
            ret = suspendArbiter();
            if(!ret)
                yError("Couldn't suspend the gaze arbiter");
            break;
        default:
            bool engineState = suspendEngine();
            bool arbiterState = suspendEngine();
            resume();
            if(!engineState)
                yError("Couldn't suspend the engine");

            if(!arbiterState)
                yError("Couldn't suspend the gaze arbiter");

            ret= (engineState && arbiterState);
            break;
    }
    return ret;
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

bool attentionManagerThread::setThreshold(const int32_t mode,const int32_t type,float val){
    bool ret = false;
    switch (type){
        case COMMAND_VOCAB_MAX:
            switch (mode){
                case COMMAND_VOCAB_ABS:
                    max_thresholdVal = val;
                    ret = true;
                    break;
                case COMMAND_VOCAB_REL:
                    max_thresholdVal += val;
                    ret = true;
                    break;
                case COMMAND_VOCAB_INIT:
                    init_max_thresholdVal = val;
                    ret = true;
                    break;
                default:
                    yError("wrong mode type");
                    break;
            }
            break;
        case COMMAND_VOCAB_MEAN:
            switch (mode){
                case COMMAND_VOCAB_ABS:
                    mean_thresholdVal = val;
                    ret = true;
                    break;
                case COMMAND_VOCAB_REL:
                    mean_thresholdVal += val;
                    ret = true;
                    break;
                case COMMAND_VOCAB_INIT:
                    init_mean_thresholdVal = val;
                    ret = true;
                    break;
                default:
                    yError("wrong mode type");
                    break;
            }
            break;
        case COMMAND_VOCAB_STD:
            switch (mode){
                case COMMAND_VOCAB_ABS:
                    std_thresholdVal = val;
                    ret = true;
                    break;
                case COMMAND_VOCAB_REL:
                    std_thresholdVal += val;
                    ret = true;
                    break;
                case COMMAND_VOCAB_INIT:
                    init_std_thresholdVal = val;
                    ret = true;
                    break;
                default:
                    yError("wrong mode type");
                    break;
            }
            break;
        case COMMAND_VOCAB_3SIGMA:
            switch (mode){
                case COMMAND_VOCAB_ABS:
                    threeSigma_thresholdVal = val;
                    ret = true;
                    break;
                case COMMAND_VOCAB_REL:
                    threeSigma_thresholdVal += val;
                    ret = true;
                    break;
                case COMMAND_VOCAB_INIT:
                    init_threeSigma_thresholdVal = val;
                    ret = true;
                    break;
                default:
                    yError("wrong mode type");
                    break;
            }
            break;
        default:
            yError("wrong threshold type");
            break;
    }
    return ret;

}

float attentionManagerThread::getThreshold(const int32_t mode,const int32_t type) {
    switch (type){
        case COMMAND_VOCAB_MAX:
            switch (mode){
                case COMMAND_VOCAB_INIT:
                    return init_max_thresholdVal;
                case COMMAND_VOCAB_CURRENT:
                    return max_thresholdVal;
                default:
                    yError("wrong threshold mode");
                    return -1;
            }
        case COMMAND_VOCAB_MEAN:
            switch (mode){
                case COMMAND_VOCAB_INIT:
                    return init_mean_thresholdVal;
                case COMMAND_VOCAB_CURRENT:
                    return mean_thresholdVal;
                default:
                    yError("wrong threshold mode");
                    return -1;
            }
        case COMMAND_VOCAB_STD:
            switch (mode){
                case COMMAND_VOCAB_INIT:
                    return init_std_thresholdVal;
                case COMMAND_VOCAB_CURRENT:
                    return std_thresholdVal;
                default:
                    yError("wrong threshold mode");
                    return -1;
            }
        case COMMAND_VOCAB_3SIGMA:
            switch (mode){
                case COMMAND_VOCAB_INIT:
                    return init_threeSigma_thresholdVal;
                case COMMAND_VOCAB_CURRENT:
                    return threeSigma_thresholdVal;
                default:
                    yError("wrong threshold mode");
                    return -1;
            }
        default:
            yError("wrong threshold type");
            return -1;
    }
}

bool attentionManagerThread::publishAnalysis() {
    if(sceneAnalysisPort.getOutputCount()){
        Bottle msg;
        msg.addInt(maxValue);
        msg.addFloat64(meanVal);
        msg.addFloat64(stdVal);
        msg.addFloat64(threeSigmaVal);
        msg.addInt(idxOfMax.x);
        msg.addInt(idxOfMax.y);
        msg.addFloat64(threeSigma_thresholdVal);
        sceneAnalysisPort.prepare() = msg;
        sceneAnalysisPort.write();
        return true;
    }
    return false;
}

bool attentionManagerThread::resetThreshold(const int32_t type) {
    bool ret = false;
    switch (type){
        case COMMAND_VOCAB_MAX:
            max_thresholdVal = init_max_thresholdVal;
            ret = true;
            break;
        case COMMAND_VOCAB_MEAN:
            mean_thresholdVal = init_mean_thresholdVal;
            ret = true;
            break;
        case COMMAND_VOCAB_STD:
            std_thresholdVal = init_std_thresholdVal;
            ret = true;
            break;
        case COMMAND_VOCAB_3SIGMA:
            threeSigma_thresholdVal = init_threeSigma_thresholdVal;
            ret = true;
            break;
        default:
            yError("wrong threshold type");
            break;

    }
    return ret;
}

bool attentionManagerThread::resetInhibition() {
    if(inhibitionControlPort.getOutputCount()){
        Bottle command;
        Bottle reply;
        command.addString("reset");
        inhibitionControlPort.write(command,reply);
        if(reply.get(0).asString()=="ok"){
            resetRequired = false;
            return true;
        }
    }
    return false;
}

void attentionManagerThread::computeAndPublishVisualizedImage() {

    visualizedImageMat =  toCvMat(*combinedImage);
    line(visualizedImageMat,Point(idxOfMax.x,0),Point(idxOfMax.x,visualizedImageMat.cols), Scalar(255,0,0));
    line(visualizedImageMat,Point(0,idxOfMax.y),Point(visualizedImageMat.rows,idxOfMax.y), Scalar(255,0,0));
    string summary = "max:" + to_string(maxValue) + " 3S: " + to_string(threeSigmaVal) ;
    putText(visualizedImageMat,summary,Point(10,10),FONT_HERSHEY_DUPLEX,2,Scalar(0,0,255),2);

    *visualizedImage = fromCvMat<PixelRgb>(visualizedImageMat);
    if(visualizationPort.getOutputCount()){
        visualizationPort.prepare() = *visualizedImage;
        visualizationPort.write();
    }

}

bool attentionManagerThread::checkMovement() {

    if(inputHeadAnglesPort.getInputCount()){
        headAnglesBottle = inputHeadAnglesPort.read(false);
        if(headAnglesBottle->size()){
            currentAngles.clear();
            double val = 0;
            for(int i=0; i<headAnglesBottle->size();i++){
                currentAngles.push_back(headAnglesBottle->get(i).asFloat32());
                val = val + pow(headAnglesBottle->get(i).asFloat32() - prevAngles.at(i),2);
            }
            val = sqrt(val);
            if(val > marginDisplacement){
                prevAngles = currentAngles;
                return true;
            }
        }
        return true;
    }
    return false;
}

