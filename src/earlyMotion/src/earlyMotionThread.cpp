// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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
 * @file earlyMotionThread.cpp
 * @brief Implementation of the visual filter thread (see earlyMotionThread.h).
 */

#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/earlyMotionThread.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

const int maxKernelSize = 5;

template<class T>
inline T max(T a, T b, T c) {    
    if(b > a) a = b;
    if(c > a) a = c;
    return a;
}

earlyMotionThread::earlyMotionThread() {
    
    inputExtImage = new ImageOf<PixelRgb>;
    inputImageFiltered = new ImageOf<PixelRgb>;
    motion = new ImageOf<PixelMono>;

    lambda = 0.05f;

    resized = false;
}

earlyMotionThread::~earlyMotionThread() {
    
    delete inputExtImage;
    delete inputImageFiltered;
    delete inputImage;

    delete motion;    
}

bool earlyMotionThread::threadInit() {
    /* open ports */ 
    if (!imagePortIn.open(getName("/image:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!motionPort.open(getName("/motion:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    /*
    if (!imagePortExt.open(getName("/imageExt:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
        }*/

    return true;
}

void earlyMotionThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string earlyMotionThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void earlyMotionThread::run() {
    while (isStopping() != true) {
        inputImage = imagePortIn.read(true);

        if (inputImage != NULL) {
            if (!resized) {
                resize(inputImage->width(), inputImage->height());
                resized = true;
            }
            else {
                //filterInputImage();
            }
          
            // sending the edge image on the outport                 
            // the copy to the port object can be avoided...
            if((motionPort.getOutputCount())) {
                ImageOf<PixelMono>& out = motionPort.prepare();
                out.resize(width, height);
                
                // extend logpolar input image
                //extender(inputImage, maxKernelSize);
                
                //extractPlanes();
                
                temporalSubtraction(&out);
                motionPort.write();
            }

            
        }
   }
}

void earlyMotionThread::resize(int width_orig,int height_orig) {
    this->width_orig = width_orig;
    this->height_orig = height_orig;
    this->width = width_orig+2*maxKernelSize;
    this->height = height_orig+maxKernelSize;

    // resizing the ROI
    //originalSrcsize.height = height_orig;
    //originalSrcsize.width = width_orig;
    //srcsize.width = width;
    //srcsize.height = height;

    // resizing plane images
    motion->resize(width_orig, height_orig);
    inputImageFiltered->resize(width_orig, height_orig);
    inputImageFiltered->zero();
 
    inputExtImage->resize(width,height);
    
}

void earlyMotionThread::filterInputImage() {
    int i;
    const int sz = inputImage->getRawImageSize();
    unsigned char * pFiltered = inputImageFiltered->getRawImage();
    unsigned char * pCurr = inputImageFiltered->getRawImage();
    const float ul = 1.0f - lambda;
    for (i = 0; i < sz; i++) {
        *pFiltered = (unsigned char)(lambda * *pCurr++ + ul * *pFiltered + .5f);
        pFiltered ++;
    }
}

ImageOf<PixelRgb>* earlyMotionThread::extender(ImageOf<PixelRgb>* inputOrigImage, int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*inputExtImage, *inputOrigImage, maxSize);
    return inputExtImage;
}

void earlyMotionThread::extractPlanes() {
    

    /* use getPadding!!!! */
    //int paddingMono = redPlane->getPadding(); 
    int padding3C = inputExtImage->getPadding(); 

    const int h = inputExtImage->height();
    const int w = inputExtImage->width();

    
}

void earlyMotionThread::temporalStore() {
    int padding = inputImage->getPadding();
    unsigned char* pin = inputImage->getRawImage();
    for(int row = 0; row < height; row++) {
        for(int col = 0; col < width ; col++) {
            *imageT3 = *imageT2;
            *imageT2 = *imageT1;
            *imageT1 = *inputImage;    
            imageT1++;
            imageT2++;
            imageT3++;
            inputImage++;
        }
        inputImage  += padding;
        imageT1  += padding;
        imageT2  += padding;
        imageT3  += padding;
    }
}

void earlyMotionThread::temporalSubtraction(ImageOf<PixelMono>* outputImage) {
    int padding = inputImage->getPadding();
    unsigned char* pin = inputImage->getRawImage();
    unsigned char* pout = outputImage->getRawImage();
    for(int row = 0; row < height; row++) {
        for(int col = 0; col < width ; col++) {
            *inputImage++ = *outputImage++;
        }
        inputImage  += padding;
        outputImage += padding;
    }
}


void earlyMotionThread::threadRelease() {
    resized = false;
}

void earlyMotionThread::onStop() {
    imagePortIn.interrupt();
    //imagePortOut.interrupt();
    //imagePortExt.interrupt();

    
    motionPort.close();
    //imagePortExt.close();

    imagePortIn.close();
}

