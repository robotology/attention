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
    outputPortName = getName("/map:o");
    outputImgPortName = getName("/img:o");
}

egocentricAudioCropperThread::~egocentricAudioCropperThread(){

}
bool egocentricAudioCropperThread::configure(yarp::os::ResourceFinder &rf){

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

    if (!outputPort.open(outputPortName.c_str())) {
        yError("Unable to open port to send output.");
        return false;
    }

    if (!outputImgPort.open(outputImgPortName.c_str())) {
        yError("Unable to open port to send output image.");
        return false;
    }

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}
void egocentricAudioCropperThread::run() {

    if (inputPort.getInputCount()) {

        yMatrix* mat = inputPort.read(false);   //blocking reading for synchr with the input

        if (mat != NULL) {
            yDebug("matrix is not null");
            if (outputPort.getOutputCount()) {
                yMatrix resizedMat = mat->submatrix(1,1,1,50);
                outputPort.prepare() = resizedMat;
                outputPort.write();
                if (outputImgPort.getOutputCount()){
                    outputImg = &outputImgPort.prepare();
                    outputImg->resize(resizedMat.cols(),resizedMat.rows());
                    outputImgPort.write();
                }
            }
        }
    }
}

void egocentricAudioCropperThread::threadRelease() {

    //-- Stop all threads.
    inputPort.interrupt();
    outputPort.interrupt();
    outputImgPort.interrupt();

    //-- Close the threads.
    inputPort.close();
    outputPort.close();
    outputImgPort.close();
}


string egocentricAudioCropperThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}