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
 * @file sacPlannerThread.cpp
 * @brief Implementation of the thread of prioritiser Collector(see header sacPlannerThread.h)
 */

#include <iCub/sacPlannerThread.h>
#include <cstring>

using namespace iCub::logpolar;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10

inline void copy_8u_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}


sacPlannerThread::sacPlannerThread() {
    
}

sacPlannerThread::sacPlannerThread(string moduleName) {
    printf("setting the name in the saccPLanner Thread \n");
    setName(moduleName);
}

sacPlannerThread::~sacPlannerThread() {
    
}

bool sacPlannerThread::threadInit() {
    idle = false;
    printf("starting the thread.... \n");
    /* open ports */
    string rootName("");
    //rootName.append(getName("/cmd:i"));
    //printf("opening ports with rootname %s .... \n", rootName.c_str());
    //inCommandPort.open(rootName.c_str());
    
    rootName.append(getName("/corr:o"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    corrPort.open(rootName.c_str());

    //initializing logpolar mapping
    cout << "||| initializing the logpolar mapping" << endl;
    int numberOfRings = 152;
    int numberOfAngles = 252;
    int xSizeValue = 320;
    int ySizeValue = 240;
    double overlap = 1.0;
    if (!trsfL2C.allocLookupTables(BOTH, numberOfRings, numberOfAngles, xSizeValue, ySizeValue, overlap)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "||| lookup table L2C allocation done" << endl;

    if (!trsfC2L.allocLookupTables(C2L, numberOfRings, numberOfAngles,xSizeValue, ySizeValue,  overlap)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "||| lookup table C2L allocation done" << endl;

    
    return true;
}

void sacPlannerThread::interrupt() {
    //inCommandPort.interrupt();
}

void sacPlannerThread::setName(string str) {
    this->name = str;
    printf("name: %s", name.c_str());
}

std::string sacPlannerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void sacPlannerThread::run() {
    while(isStopping() != true){        
        //Bottle* b=inCommandPort.read(true);        
        if(!idle) {
            if((corrPort.getOutputCount())&&(inputImage!=NULL)) {
                ImageOf<PixelRgb>& outputImage =  corrPort.prepare();
                ImageOf<PixelRgb>* intermImage = new ImageOf<PixelRgb>;
                ImageOf<PixelRgb>* intermImage2 = new ImageOf<PixelRgb>;
                intermImage->resize(320,240);
                intermImage2->resize(320,240);
                outputImage.resize(320,240);
                outputImage.zero();
                trsfL2C.logpolarToCart(*intermImage,*inputImage);
                shiftROI(intermImage,&outputImage,120,140);
                //printf("copying the image %d %d \n", inputImage->width(), inputImage->height());
                //copy_8u_C1R(inputImage,&outputImage);
                //trsfL2C.cartToLogpolar(outputImage, *intermImage2);
                //outputImage.copy(*inputImage); 
                //inputImage->copy(outputImage);
                corrPort.write();
                delete intermImage;
                delete intermImage2;
            }
        }
        Time::delay(0.05);
    }
}

void sacPlannerThread::referenceRetina(ImageOf<PixelRgb>* ref) {
    printf("referenceRetina \n");
    inputImage = ref;
}

void sacPlannerThread::shiftROI(ImageOf<PixelRgb>* inImg,ImageOf<PixelRgb>* outImg,int x, int y) {
    int dx = x - 160;
    int dy = y - 120;
    printf("dx %d dy %d \n", dx,dy);
    unsigned char* pinput  = inImg->getRawImage();
    unsigned char* poutput = outImg->getRawImage();
    int padding = inImg->getPadding();
    printf("padding  %d \n", padding);
    int rowsize = inImg->getRowSize();
    printf("rowsize %d \n", rowsize);
    if(dx > 0) {
        printf("jumping in x \n");
        pinput += 3 * dx;
    }
    if(dy > 0) {
        printf("jumping in y \n");
        pinput += dy * rowsize;
    }
    for(int row = 0; row < inImg->height(); row++) {        
        if((row + dy <= 0)||(row + dy > 240)){            
            for(int col = 0; col< inImg->width(); col++) {
                //zero
                *poutput++ = (unsigned char) 0;
                *poutput++ = (unsigned char) 0;
                *poutput++ = (unsigned char) 0;
            }
            poutput += padding;
        }
        else {
            for(int col = 0; col< inImg->width(); col++) {
                if((col + dx <= 0)||(col + dx > 320)){
                    //zero
                    *poutput++ = (unsigned char) 0;
                    *poutput++ = (unsigned char) 0;
                    *poutput++ = (unsigned char) 0;
                }
                else {
                    //copying
                    *poutput++ = *pinput++;
                    *poutput++ = *pinput++;
                    *poutput++ = *pinput++;
                }
            }
            poutput += padding;
            pinput  += ( rowsize -  280  * 3);
        }
    }
    
}


void sacPlannerThread::onStop() {
    //inCommandPort.interrupt();
    //inCommandPort.close();
    corrPort.interrupt();
    corrPort.close();
}

void sacPlannerThread::threadRelease() {
    
}

void sacPlannerThread::setSaccadicTarget(int rho, int theta) {
    printf("setting Saccadic Planner \n");
    printf("rho %d theta %d \n", rho, theta);
}
