// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file opticalFlowThread.cpp
 * @brief Implementation of the eventDriven thread (see opticalFlowThread.h).
 */

#include "opticalFlowThread.h"

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

opticalFlowThread::opticalFlowThread() {
    robot = "icub";        
    firstInstance = true;
}

opticalFlowThread::opticalFlowThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
    firstInstance = true;
}

opticalFlowThread::~opticalFlowThread() {
    // do nothing
}

bool opticalFlowThread::threadInit() {
    // initialization of the images
    inputImagePrev = new ImageOf<PixelRgb>;


    if (!inputPort.open(getName("/img:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
   
    if (!outputPort.open(getName("/img:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
    

}

void opticalFlowThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string opticalFlowThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void opticalFlowThread::setInputPortName(string InpPort) {
    
}

void opticalFlowThread::run() {    
    while (isStopping() != true) {

        if (inputPort.getInputCount()){

            inputImage = inputPort.read(true);
            
            if(firstInstance){                
                firstInstance = false;
                width         = inputImage->width();
                height        = inputImage->height();
                rowsize       = inputImage->getRowSize();
                inputImagePrev->resize(width, height);

                CvSize sz;
                sz.height = height;
                sz.width  = width;
                // processing of the previous and current image
                imgU1 = cvCreateImage(sz,IPL_DEPTH_64F,1);
                imgV1 = cvCreateImage(sz,IPL_DEPTH_64F,1);
                imgO1 = cvCreateImage(sz,IPL_DEPTH_64F,1);
                imgU2 = cvCreateImage(sz,IPL_DEPTH_64F,1);
                imgV2 = cvCreateImage(sz,IPL_DEPTH_64F,1);
                imgO2 = cvCreateImage(sz,IPL_DEPTH_64F,1);
            }
            else{
                /*
                IplImage *imgCurrent  = (IplImage*) inputImage->getIplImage();
                IplImage *imgPrevious = (IplImage*) inputImagePrev->getIplImage();
                //wrap all the input and output images in OpenCVImageAdapter, so that they can be
                //accessed by OpenVis3D
                OpenCVImageAdapter*ovaImg1  = new OpenCVImageAdapter(imgCurrent);
                OpenCVImageAdapter*ovaImg2  = new OpenCVImageAdapter(imgPrevious);
                OpenCVImageAdapter*ovaImgU1 = new OpenCVImageAdapter(imgU1);
                OpenCVImageAdapter*ovaImgV1 = new OpenCVImageAdapter(imgV1);
                OpenCVImageAdapter*ovaImgO1 = new OpenCVImageAdapter(imgO1);
                OpenCVImageAdapter*ovaImgU2 = new OpenCVImageAdapter(imgU2);
                OpenCVImageAdapter*ovaImgV2 = new OpenCVImageAdapter(imgV2);
                OpenCVImageAdapter*ovaImgO2 = new OpenCVImageAdapter(imgO2);
                //create Birchfield-Tomasi local matcher and set its default parameter alpha to 20.0
                BTLocalMatcherT<double> btmatcher;
                double alpha[] = {20.0};
                btmatcher.setParams(1,alpha);
                
                //create global diffusion-based optical flow algorithm instance
                OvFlowDiffuseMatcherT<double> flowDiffuseMatcher;
                
                //create general optical flow algorithm execution manager instance
                OvFlowT<double> flowManager;
                flowManager.setLocalImageMatcher(btmatcher);
                flowManager.setGlobalMatcher(flowDiffuseMatcher);
                
                printf("\nRunning optical flow ...\n");
                
                //EXECUTE optical flow estimation
                flowManager.doOpticalFlow(*ovaImg1, *ovaImg2, minshiftX, maxshiftX, minshiftY, maxshiftY, *ovaImgU1, *ovaImgV1, *ovaImgO1, *ovaImgU2, *ovaImgV2, *ovaImgO2);
                */
                
            }
            // copying content into previous image
            startTime = Time::now();
            memcpy(inputImagePrev->getRawImage(), inputImage->getRawImage(), sizeof(unsigned char) * rowsize * height);
            endTime   = Time::now();
            intervalTime = endTime - startTime;
            std::cout.precision(8);
            std::cout<<"1 measured interval time "<<std::fixed<<intervalTime<<std::endl;
            
            

            if (outputPort.getOutputCount()) {
                outputImage = &outputPort.prepare();
                outputImage->resize(width, height);
              
                startTime = Time::now();
                outputImage->copy((const ImageOf<PixelRgb>)*inputImagePrev);
                endTime   = Time::now();
                intervalTime = endTime - startTime;
                std::cout<<"2 measured interval time "<<std::fixed<<intervalTime<<std::endl;
                
                outputPort.write();  
            }
            
        }
    }               
}

void opticalFlowThread::threadRelease() {
    delete inputImagePrev;
     
}

void opticalFlowThread::onStop() {    
    outputPort.interrupt();
    outputPort.close();
}

