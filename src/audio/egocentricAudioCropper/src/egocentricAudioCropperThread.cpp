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
#include "iCub/egocentricAudioCropperThread.h"

#define THPERIOD 0.01

egocentricAudioCropperThread::egocentricAudioCropperThread(string moduleName):PeriodicThread(THPERIOD){

    this->moduleName = moduleName;
    inputPortName = getName("/map:i");
    inputGazeAnglesPortName = getName("/gazeAngles:i");
    outputPortName = getName("/map:o");
    outputImgPortName = getName("/cartImg:o");
    outputScaledImgPortName = getName("/cartScaledImg:o");
    azimuthAngle = 0.0;

}

egocentricAudioCropperThread::~egocentricAudioCropperThread(){

}
bool egocentricAudioCropperThread::configure(yarp::os::ResourceFinder &rf){

    cameraFileName = rf.findGroup("cameraParams").check("fileName",    yarp::os::Value("icubEyes.ini"), "the file name of the camera parameters (string)").asString();
    cameraSide = rf.findGroup("cameraParams").check("side",    yarp::os::Value("left"), "the side  of the used camera (string)").asString();
    cameraContextName  = rf.findGroup("cameraParams").check("context",    yarp::os::Value("logpolarAttention"), "the context  of the  camera file (string)").asString();
    azimuthIndex =  rf.findGroup("cameraParams").check("azimuthIndex",    yarp::os::Value(0), "the index of the  azimuth angle in the angles input port (int)").asInt();

    conversionGain =  rf.findGroup("maximisation").check("conversionGain",    yarp::os::Value(-1.0), "the coversion gain").asFloat64();


    ResourceFinder iCubEyesRF;
    iCubEyesRF.setVerbose(true);
    iCubEyesRF.setDefaultConfigFile(cameraFileName);
    iCubEyesRF.setDefaultContext(cameraContextName);
    iCubEyesRF.configure(0, nullptr);


    if (cameraSide == "right"){
        cameraWidth = rf.findGroup("CAMERA_CALIBRATION_RIGHT").check("w",    yarp::os::Value(320), "the width of the camera (double)").asDouble();
        cameraFocalLength= rf.findGroup("CAMERA_CALIBRATION_RIGHT").check("fx",yarp::os::Value(200),"the focal length of the camera" ).asDouble();
        cameraHeight = rf.findGroup("CAMERA_CALIBRATION_RIGHT").check("h", yarp::os::Value(240), "the height of the camera (double)").asDouble();

    }
    else{
        cameraWidth = rf.findGroup("CAMERA_CALIBRATION_LEFT").check("w",    yarp::os::Value(320), "the width of the camera (double)").asDouble();
        cameraFocalLength= rf.findGroup("CAMERA_CALIBRATION_LEFT").check("fx",yarp::os::Value(200),"the focal length of the camera" ).asDouble();
        cameraHeight = rf.findGroup("CAMERA_CALIBRATION_LEFT").check("h", yarp::os::Value(240), "the height of the camera (double)").asDouble();

    }
    cameraAOV = atan(cameraWidth/(2*cameraFocalLength))*(180.0/M_PI)*2;
    cameraSideAOV = cameraAOV/2;

    return true;
}


bool egocentricAudioCropperThread::threadInit() {

    /* ===========================================================================
	 *  Initialize all ports. If any fail to open, return false to
	 *    let RFModule know initialization was unsuccessful.
	 * =========================================================================== */

    if (!inputPort.open(inputPortName.c_str())) {
        yError("Unable to open port to receive input.");
        return false;
    }
    if (!inputGazeAnglesPort.open(inputGazeAnglesPortName.c_str())) {
        yError("Unable to open port for receiving robot head angle.");
        return false;
    }

    if (!outputPort.open(outputPortName.c_str())) {
        yError("Unable to open port to send output.");
        return false;
    }

    if (!outputImgPort.open(outputImgPortName.c_str())) {
        yError("Unable to open port to send output image.");
        return false;
    }
    if (!outputScaledImgPort.open(outputScaledImgPortName.c_str())) {
        yError("Unable to open port to send scaled output image.");
        return false;
    }

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}
void egocentricAudioCropperThread::run() {

    if (inputPort.getInputCount()) {

        yMatrix* mat = inputPort.read(true);   //blocking reading for synchr with the input

        if (inputGazeAnglesPort.getInputCount()) {
            gazeAnglesBottle = inputGazeAnglesPort.read(false);
            azimuthAngle = gazeAnglesBottle->get(azimuthIndex).asDouble();
        }

        if (mat != NULL) {
            if (outputPort.getOutputCount()) {
                double * pMat = mat->data();
                yMatrix resizedMat;
                resizedMat.resize(1,cameraAOV);
                double *pResizedMat = resizedMat.data();
                pMat += (int) (179-azimuthAngle-cameraSideAOV);
                int maxStartIdx = 0;
                int maxCount = 0;
                double maxValue = 0;
                for(int i=0;i<cameraAOV;i++){
                    *pResizedMat = *pMat;
                    if(*pMat > maxValue){
                        maxValue = *pMat;
                        maxStartIdx = i;
                        maxCount = 1;
                    }
                    else if(*pMat == maxValue){
                        maxCount ++;
                    }
                    pMat ++;
                    pResizedMat ++;

                }
                yInfo("max Value = %lf",maxValue);
                outputPort.prepare() = resizedMat;
                outputPort.write();
                if (outputImgPort.getOutputCount()){
                    outputImg = &outputImgPort.prepare();
                    outputImg->resize(resizedMat.cols(),resizedMat.rows());
                    unsigned char* rowImage = outputImg->getRawImage();
                    for (int i = 0; i<cameraAOV;i++){
                        if(i>= maxStartIdx && i<maxStartIdx + maxCount){
                            if(conversionGain >0)
                                rowImage[i] = maxValue*conversionGain*255.0;
                            else{
                                rowImage[i] = 255.0;
                            }
                        }
                        else{
                            rowImage[i] = 0;
                        }

                    }
                    if (outputScaledImgPort.getOutputCount()){
                        outputScaledImg = &outputScaledImgPort.prepare();
                        outputScaledImg->copy(*outputImg,cameraWidth,cameraHeight);
                        outputScaledImgPort.write();
                    }
                    outputImgPort.write();
                }

            }
        }
    }
}

void egocentricAudioCropperThread::threadRelease() {

    //-- Stop all threads.
    inputGazeAnglesPort.interrupt();
    inputPort.interrupt();
    outputPort.interrupt();
    outputImgPort.interrupt();

    //-- Close the threads.
    inputGazeAnglesPort.close();
    inputPort.close();
    outputPort.close();
    outputImgPort.close();
}


string egocentricAudioCropperThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}
