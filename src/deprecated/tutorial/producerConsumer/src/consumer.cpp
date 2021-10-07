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
 * @file consumer.cpp
 * @brief Implementation of the thread that represent image (see header consumer.h)
 */

#include <iCub/consumer.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10

consumer::consumer() : RateThread(THRATE) {
    synchronised = false;
    count  = 0;
    width  = 320;   //default dimension width 
    height = 240;   //default dimension height
}

consumer::~consumer() {

}

bool consumer::threadInit() {
	timepred_consumer  = Time::now();
    printf("\n starting the thread.... \n");
    // opening ports 
    leftPort.open      (getName("/vis:o").c_str());
    rightPort.open     (getName("/right:o").c_str());

    // initialising images
    imageLeft      = new ImageOf<PixelRgb>;
    imageLeft->resize(width,height);
    
    
    printf("initialization in plotter thread correctly ended \n");
    return true;
}

void consumer::interrupt() {
    leftPort.interrupt();    
    rightPort.interrupt();
}

void consumer::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string consumer::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void consumer::resize(int widthp, int heightp) {
}

void consumer::copyLeft(ImageOf<PixelMono>* image) {
    //printf("retinalSize in consumer %d \n",retinalSize);
    int padding= image->getPadding();
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft  = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < height; r++) {
            for(int c = 0; c < width; c++) {                
                if(r%2 == 0){
                    *pleft++ = 0;
                }
                else{
                    *pleft++ = 255;
                }
            }
            pleft  += padding;
            pimage += padding;
        }
    }
}

void consumer::copyLeft(ImageOf<PixelRgb>* image) {
    int padding= imageLeft->getPadding();
    //printf("retinalSize in consumer %d %d %d \n",padding, width, height);
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

bool consumer::test(){
    bool ret = true;
    ret &= Network::exists(getName("/image:i").c_str());
    return ret;
}

void consumer::run() {
	timenow_consumer = Time::now();
	timeDiff_consumer = timenow_consumer-timepred_consumer;
	pSem->wait();
	(*pSharedResource)--; //count is shared,we need a semaphore to synchronize
	printf ("Consumer: %d %f \n", *pSharedResource, timeDiff_consumer);
	pSem->post();
	//Time::delay(3.0);
	timepred_consumer  = Time::now();
}

void consumer::setSharedResource(int* ppSharedResource) {

	pSharedResource=ppSharedResource;
}

void consumer::setSharedSemaphore(Semaphore* ppSem) {

	pSem=ppSem;
}


void consumer::threadRelease() {
    printf("consumer: portClosing \n");  
    leftPort.close();
    rightPort.close();

    printf("freeing memory \n");
    // free allocated memory here please

    printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
