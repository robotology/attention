// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file plotterThread.cpp
 * @brief Implementation of the thread that represent image (see header plotterThread.h)
 */

#include <iCub/plotterThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 33

plotterThread::plotterThread() : RateThread(THRATE) {
    synchronised = false;
    count  = 0;
    width  = 320;   //default dimension width 
    height = 240;   //default dimension height
}

plotterThread::~plotterThread() {

}

bool plotterThread::threadInit() {
    printf("\n starting the thread.... \n");
    // opening ports 
    leftPort.open      (getName("/image:o").c_str());
    rightPort.open     (getName("/right:o").c_str());

    // initialising images
    imageLeft      = new ImageOf<PixelMono>;
    imageLeft->resize(width,height);
    
    
    printf("initialization in plotter thread correctly ended \n");
    return true;
}

void plotterThread::interrupt() {
    leftPort.interrupt();    
    rightPort.interrupt();
}

void plotterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string plotterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void plotterThread::resize(int widthp, int heightp) {
}

void plotterThread::copyLeft(ImageOf<PixelMono>* image) {
    //printf("retinalSize in plotterThread %d \n",retinalSize);
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft  = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < height; r++) {
            for(int c = 0; c < width; c++) {                 
                *pleft++ = *pimage++;              
            }
            pleft  += padding;
            pimage += padding;
        }
    }
}

void plotterThread::copyLeft(ImageOf<PixelRgb>* image) {
    int padding= imageLeft->getPadding();
    //printf("retinalSize in plotterThread %d %d %d \n",padding, width, height);
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft  = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < height; r++) {
            for(int c = 0; c < width; c++) {                
                if(r%2 == 0) {
                    *pleft++ = 0;
                    *pleft++ = 0;
                    *pleft++ = 0;
                }
                else{
                    *pleft++ = 255;
                    *pleft++ = 255;
                    *pleft++ = 255;
                }
            }
            pleft  += padding;
            pimage += padding;
        }
    }
}

bool plotterThread::test(){
    bool ret = true;
    ret &= Network::exists(getName("/image:i").c_str());
    return ret;
}

void plotterThread::run() {
    count++;
    imageLeft  = &leftPort.prepare();
    imageLeft->resize(width, height);
    imageRight = &rightPort.prepare();
    imageRight->resize(width, height);
    synchronised = true;

    
    if(leftPort.getOutputCount()) {
        leftPort.write();
    }
    if(rightPort.getOutputCount()) {
        rightPort.write();
    }

    
}




void plotterThread::threadRelease() {
    printf("plotterThread: portClosing \n");  
    leftPort.close();
    rightPort.close();

    printf("freeing memory \n");
    // free allocated memory here please

    printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
