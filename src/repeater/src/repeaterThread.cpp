// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email: shashank.pathak@iit.it
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

/**
 * @file repeaterThread.cpp
 * @brief Implementation of the eventDriven thread (see repeaterThread.h).
 */

#include <iCub/repeaterThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

repeaterThread::repeaterThread():inputCbPort() {
    robot = "icub";        
}

repeaterThread::repeaterThread(string _robot, string _configFile):inputCbPort(){
    robot = _robot;
    configFile = _configFile;
}

repeaterThread::~repeaterThread() {
    // do nothing
}

bool repeaterThread::threadInit() {
    
    /* open ports */ 
    //inputCbPort.hasNewImage = false;
    //inputCbPort.useCallback();          // to enable the port listening to events via callback

    if (!inputCbPort.open(getName("/img:i").c_str())) {
        cout <<": unable to open port for reading events  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputPort.open(getName("/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
}

void repeaterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string repeaterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void repeaterThread::setInputPortName(string InpPort) {
    this->inputPortName = InpPort;
}

void repeaterThread::run() {    
    while (isStopping() != true) {
        if (inputCbPort.getInputCount()) {
            inputImage = inputCbPort.read(true);
            inputHeight = inputImage->height();
            inputWidth  = inputImage->width();
            widthRatio  = floor(inputWidth / outputWidth);
            heightRatio = floor(inputHeight / outputHeight);

            printf("\n ratio %d %d \n", widthRatio, heightRatio);
            
            //outputPort.prepare() = *inputImage;
            outputImage = &outputPort.prepare();
            processing();
            outputPort.write();  
        }
    }               
}


void repeaterThread::processing() {
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
    
    printf("padding matrix: %d-%d=%d \n",tempIpl.widthStep, tempIpl.width,  matrixPadding);
    printf("padding outputImage : %d \n", outPadding);
    
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
    //instead of 
    //outputImage-> wrapIplImage(&tempIpl);

    /**
    unsigned char* pIn  = inputImage->getRawImage();
    unsigned char* pOut = outputImage->getRawImage();
    int inPadding  = inputImage->getPadding();
    int outPadding = outputImage->getPadding();
    int inRowSize  = inputImage->getRowSize();

    for (int r = 0; r < outputHeight; r ++) {
        for(int c = 0 ; c < outputWidth; c++) {
            
            *pOut++ = *pIn++;
            *pOut++ = *pIn++;
            *pOut++ = *pIn++;
            
            for (int rcCount = 0; rcCount < widthRatio - 1; rcCount++) {
                pIn+=3;
            }
            
        }
        pOut += outPadding;
        pIn  += inPadding;

        for (int rrCount = 0; rrCount < heightRatio - 1 ; rrCount++) {
            pIn += inRowSize;
        }
        
    }
    */
    
}

void repeaterThread::threadRelease() {
    // nothing     
}

void repeaterThread::onStop() {
    inputCallbackPort.interrupt();
    outputPort.interrupt();

    inputCallbackPort.close();
    outputPort.close();
}

