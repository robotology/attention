// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
  * Copyright (C)2015  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file handProfilerThread.cpp
 * @brief Implementation of the handProfiler thread (see handProfilerThread.h).
 */

#include <iCub/handProfilerThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

handProfilerThread::handProfilerThread():inputCbPort() {
    robot = "icub";        
}

handProfilerThread::handProfilerThread(string _robot, string _configFile):inputCbPort(){
    robot = _robot;
    configFile = _configFile;
}

handProfilerThread::~handProfilerThread() {
    // do nothing
}

bool handProfilerThread::threadInit() {
    
    /* open ports */ 
    //inputCbPort.hasNewImage = false;
    //inputCbPort.useCallback();          // to enable the port listening to events via callback

    if (!inputCbPort.open(getName("/in").c_str())) {
        cout <<": unable to open port for reading events  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputPort.open(getName("/out").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
}

void handProfilerThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string handProfilerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void handProfilerThread::setInputPortName(string InpPort) {
    this->inputPortName = InpPort;
}

void handProfilerThread::run() {    
    while (isStopping() != true) {
        if ((inputCbPort.getInputCount()) && (outputPort.getOutputCount())) {
            timeStart = Time::now();
            inputImage = inputCbPort.read(true);
            Stamp timestamp;
            inputCbPort.getEnvelope(timestamp);
            timeEnd = Time::now();
            double time = 1 / (timeEnd-timeStart);
            //printf("time interval %f fps\n", time);
            
            inputHeight = inputImage->height();
            inputWidth  = inputImage->width();
            widthRatio  = floor(inputWidth / outputWidth);
            heightRatio = floor(inputHeight / outputHeight);
      
            //outputPort.prepare() = *inputImage;
            outputImage = &outputPort.prepare();
            processing();
            outputPort.setEnvelope(timestamp);
            outputPort.write();  
            
        }
    }               
}


void handProfilerThread::processing() {
    outputImage->resize(outputWidth, outputHeight);
    unsigned char* pOut = outputImage->getRawImage();
    int outPadding = outputImage->getPadding();

    cv::Mat inputMatrix((IplImage*)  inputImage->getIplImage(), false);
    cv::Mat outputMatrix((IplImage*) outputImage->getIplImage(), false);

    cv::resize(inputMatrix, outputMatrix,outputMatrix.size(), 0, 0, CV_INTER_LINEAR);
    //interpolation – interpolation method:
    //INTER_NEAREST - a nearest-neighbor interpolation
    //INTER_LINEAR - a bilinear interpolation (used by default)
    //INTER_AREA - resampling using pixel area relation. It may be a preferred method for image decimation, 
    //INTER_CUBIC - a bicubic interpolation over 4x4 pixel neighborhood
    //INTER_LANCZOS4 - a Lanczos interpolation over 8x8 pixel neighborhood

    IplImage tempIpl = (IplImage) outputMatrix;
    char* pMatrix     = tempIpl.imageData;
    int matrixPadding = tempIpl.widthStep - tempIpl.width * 3; 
    
    //making a copy of it
    for (int r = 0; r < outputHeight; r ++) {
        for(int c = 0 ; c < outputWidth; c++) {            
            *pOut++ = *pMatrix++;
            *pOut++ = *pMatrix++;
            *pOut++ = *pMatrix++;            
        }
        pOut     += outPadding;
        pMatrix  += matrixPadding;
    }
}

void handProfilerThread::threadRelease() {
    // nothing     
        inputCallbackPort.interrupt();
    outputPort.interrupt();

    inputCallbackPort.close();
    outputPort.close();
}

void handProfilerThread::onStop() {
    //inputCallbackPort.interrupt();
    //outputPort.interrupt();

    //inputCallbackPort.close();
    //outputPort.close();
}
