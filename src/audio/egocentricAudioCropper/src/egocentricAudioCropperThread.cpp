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
    outputImgPortName = getName("/cartImg:o");
    outputScaledImgPortName = getName("/cartScaledImg:o");
    maxAngleStatePortName = getName("/maxAngleState:o");
    azimuthAngle = 0.0;

}

egocentricAudioCropperThread::~egocentricAudioCropperThread(){

}
bool egocentricAudioCropperThread::configure(yarp::os::ResourceFinder &rf){

    cameraFileName = rf.findGroup("cameraParams").check("fileName",    yarp::os::Value("icubEyes.ini"), "the file name of the camera parameters (string)").asString();
    cameraSide = rf.findGroup("cameraParams").check("side",    yarp::os::Value("left"), "the side  of the used camera (string)").asString();
    cameraContextName  = rf.findGroup("cameraParams").check("context",    yarp::os::Value("logpolarAttention"), "the context  of the  camera file (string)").asString();
    azimuthIndex =  rf.findGroup("cameraParams").check("azimuthIndex",    yarp::os::Value(0), "the index of the  azimuth angle in the angles input port (int)").asInt();


    ResourceFinder iCubEyesRF;
    iCubEyesRF.setVerbose(true);
    iCubEyesRF.setDefaultConfigFile(cameraFileName);
    iCubEyesRF.setDefaultContext(cameraContextName);
    iCubEyesRF.configure(0, nullptr);


    if (cameraSide == "right"){
        cameraWidth = iCubEyesRF.findGroup("CAMERA_CALIBRATION_RIGHT").check("w",    yarp::os::Value(320), "the width of the camera (double)").asDouble();
        cameraFocalLength= iCubEyesRF.findGroup("CAMERA_CALIBRATION_RIGHT").check("fx",yarp::os::Value(200),"the focal length of the camera" ).asDouble();
        cameraHeight = iCubEyesRF.findGroup("CAMERA_CALIBRATION_RIGHT").check("h", yarp::os::Value(240), "the height of the camera (double)").asDouble();

    }
    else{
        cameraWidth = iCubEyesRF.findGroup("CAMERA_CALIBRATION_LEFT").check("w",    yarp::os::Value(320), "the width of the camera (double)").asDouble();
        cameraFocalLength= iCubEyesRF.findGroup("CAMERA_CALIBRATION_LEFT").check("fx",yarp::os::Value(200),"the focal length of the camera" ).asDouble();
        cameraHeight = iCubEyesRF.findGroup("CAMERA_CALIBRATION_LEFT").check("h", yarp::os::Value(240), "the height of the camera (double)").asDouble();

    }
    cameraAOV = atan(cameraWidth/(2*cameraFocalLength))*(180.0/M_PI)*2;
    cameraSideAOV = cameraAOV/2;

    cutImg = new yImgPixelMono;
    cutImg->resize(cameraAOV,1);

    yInfo( "\t                [cameraParams]               "                                );
    yInfo( "\t ============================================ "                               );
    yInfo( "\t Camera file Name          : %s",        cameraFileName.c_str()               );
    yInfo( "\t Camera context            : %s",        cameraContextName.c_str()            );
    yInfo( "\t Azimuth Index in port     : %d",        azimuthIndex                         );
    yInfo( "\t used Camera               : %s",        cameraSide.c_str()                   );
    yInfo( " " );
    yInfo( " " );
    yInfo( "\t                  [icubEyes.ini]             "                               );
    yInfo( "\t ============================================ "                              );
    yInfo( "\t Camera hight                : %f",        cameraHeight                      );
    yInfo( "\t Camera width                : %f",        cameraWidth                       );
    yInfo( "\t Camera focal length         : %f",        cameraFocalLength                 );
    yInfo( "\t Camera AOV                  : %f",        cameraAOV                         );
    yInfo( "\t Camera Side AOV             : %d",        cameraSideAOV                     );
    yInfo( " " );


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


    if (!outputImgPort.open(outputImgPortName.c_str())) {
        yError("Unable to open port to send output image.");
        return false;
    }
    if (!outputScaledImgPort.open(outputScaledImgPortName.c_str())) {
        yError("Unable to open port to send scaled output image.");
        return false;
    }
    if(!maxAngleStatePort.open(maxAngleStatePortName.c_str())){
        yError("Unable to open port /maxAngleState:o");
        return false;
    }

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}
void egocentricAudioCropperThread::run() {

    if (inputPort.getInputCount()) {

        inputImg = inputPort.read(true);   //blocking reading for synchr with the input

        if (inputGazeAnglesPort.getInputCount()) {
            gazeAnglesBottle = inputGazeAnglesPort.read(false);
            if(gazeAnglesBottle->size())
                azimuthAngle = gazeAnglesBottle->get(azimuthIndex).asDouble();
        }

        if (inputImg != NULL) {
            unsigned char* rowOutImage = cutImg->getRawImage();
            unsigned char* rowInImage = inputImg->getRawImage();
            double maxValue = 0;
            int maxCount = 0;
            int maxStartIdx = 0;
            for(int i=0;i<cameraAOV;i++){
                rowOutImage[i] = rowInImage[(int) (i + 179 - azimuthAngle - cameraSideAOV)];
                if(rowOutImage[i] > maxValue){
                    maxValue = rowOutImage[i];
                    maxStartIdx = i;
                    maxCount = 1;
                }
                else if(rowOutImage[i] == maxValue){
                    maxCount ++;
                }
            }

            publishMaxAngleState(maxValue,maxStartIdx,maxStartIdx - (int) (azimuthAngle+cameraSideAOV),azimuthAngle);
            yInfo("max Value = %lf",maxValue);

            if (outputImgPort.getOutputCount()) {
                outputImg = &outputImgPort.prepare();
                outputImg->copy(*cutImg,cameraAOV,1);
                outputImgPort.write();
            }
            if (outputScaledImgPort.getOutputCount()){
                outputScaledImg = &outputScaledImgPort.prepare();
                outputScaledImg->copy(*cutImg,cameraWidth,cameraHeight);
                outputScaledImgPort.write();
            }
        }
    }
}

void egocentricAudioCropperThread::threadRelease() {

    //-- Stop all threads.
    inputGazeAnglesPort.interrupt();
    inputPort.interrupt();
    outputImgPort.interrupt();
    outputScaledImgPort.interrupt();
    maxAngleStatePort.interrupt();

    //-- Close the threads.
    inputGazeAnglesPort.close();
    inputPort.close();
    outputImgPort.close();
    outputScaledImgPort.close();
    maxAngleStatePort.close();

    delete cutImg;
}


string egocentricAudioCropperThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}

bool egocentricAudioCropperThread::publishMaxAngleState(double maxVal,int egoMAxAngle,int aloMaxAngle, double azimuthAngle) {
    if(maxAngleStatePort.getOutputCount()){
        Bottle msg;
        msg.addDouble(maxVal);
        msg.addInt(egoMAxAngle);
        msg.addInt(aloMaxAngle);
        msg.addDouble(azimuthAngle);
        maxAngleStatePort.prepare() = msg;
        maxAngleStatePort.write();
        return true;
    }
    return false;
}
