// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
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
 * @file mosaicThread.cpp
 * @brief Implementation of the mosaic thread (see mosaicThread.h).
 */

#include <iCub/mosaicThread.h>
#include <cstring>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;



mosaicThread::mosaicThread() {

    inputImage = new ImageOf<PixelRgb>;
    outputImageMosaic = new ImageOf<PixelRgb>;
    
    resized = false;

}

mosaicThread::~mosaicThread() {
    // do nothing   
}

bool mosaicThread::threadInit() {
    /* open ports */ 
    if (!imagePortIn.open(getName("/image:i").c_str())) {
        cout <<": unable to open port for camera  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    
    if (!imagePortOut.open(getName("/image:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    return true;
}


void mosaicThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string mosaicThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void mosaicThread::setMosaicDim(int w, int h) {
    //mosaic image default size
    width = w;
    height = h;
    //default position of input image's center
    xcoord = width/2;
    ycoord = height/2;
}

void mosaicThread::setInputDim(int w, int h) {
    //input image default size
    width_orig = w;
    height_orig = h;
}

void mosaicThread::run() {
    
    outputImageMosaic->resize(this->width,this->height);
    outputImageMosaic->zero();
    while (isStopping() != true)  {                
        
        inputImage = imagePortIn.read(false); // do not wait                          
        if (inputImage != NULL ) {            
            if(!resized) {
                resize(inputImage->width(),inputImage->height());
                resized = true;
            }
            makeMosaic(inputImage); 
            imagePortOut.prepare() = *outputImageMosaic;                   
            imagePortOut.write();
        }                      
    }
}

void mosaicThread::resize(int width_orig,int height_orig) {        
    inputImage->resize(width_orig,height_orig);
    this->width_orig = width_orig;
    this->height_orig = height_orig;
    
}

bool mosaicThread::placeInpImage(int X, int Y) {
    if(X > this->width || X < 0 || Y > this->height || Y < 0) return false;
    this->xcoord = X;
    this->ycoord = Y;
    return true;
}

bool mosaicThread::setMosaicSize(int width=DEFAULT_WIDTH, int height=DEFAULT_HEIGHT) {
    if(width > MAX_WIDTH || width < width_orig || width <= 0 
       || height > MAX_HEIGHT || height < height_orig || height <= 0 ) return false;
    this->width = width;
    this->height = height;
    outputImageMosaic->resize(this->width,this->height);
    outputImageMosaic->zero();
    return true;    
}

void mosaicThread::makeMosaic(ImageOf<PixelRgb>* inputImage) {
    int i,j;
    unsigned char* inpTemp = inputImage->getRawImage();
    unsigned char* outTemp = outputImageMosaic->getRawImage();
    int iW = inputImage->width();
    int iH = inputImage->height();
    int mPad = outputImageMosaic->getPadding();
    
    for(i = 0 ; i < iH ; ++i) {
        for(j = 0 ; j < iW ; ++j) {
            int mosaicX = i; mosaicX -= iH / 2 ; mosaicX += ycoord; 
            int mosaicY = j; mosaicY -= iW / 2 ; mosaicY += xcoord;
            if(mosaicX < height && mosaicY < width) {
                int index = mosaicX; index *= width * 3 + mPad; index += 3 * mosaicY;
                 
                *(outTemp + index) += *inpTemp;
                inpTemp++;
                
                *(outTemp + index + 1) += *inpTemp;
                inpTemp++;
                
                *(outTemp + index + 2) += *inpTemp;
                inpTemp++;
            }
            else inpTemp +=3;
        }
    }
}


void mosaicThread::threadRelease() {
    resized = false;     
}

void mosaicThread::onStop() {
    imagePortIn.interrupt();
    imagePortOut.interrupt();
        
    imagePortOut.close();
    imagePortIn.close();
}

