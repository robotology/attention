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
    redPlane = new ImageOf<PixelMono>;
    redPlane2 = new ImageOf<PixelMono>;
    redPlane3 = new ImageOf<PixelMono>;
    greenPlane = new ImageOf<PixelMono>;
    greenPlane2 = new ImageOf<PixelMono>;
    greenPlane3 = new ImageOf<PixelMono>;
    bluePlane = new ImageOf<PixelMono>;
    bluePlane2 = new ImageOf<PixelMono>;
    bluePlane3 = new ImageOf<PixelMono>;
    yellowPlane = new ImageOf<PixelMono>;
    yellowPlane2 = new ImageOf<PixelMono>;
    inputExtImage = new ImageOf<PixelRgb>;
    inputImageFiltered = new ImageOf<PixelRgb>;

    redPlus = new ImageOf<PixelMono>;
    redMinus = new ImageOf<PixelMono>;
    greenPlus = new ImageOf<PixelMono>;
    greenMinus = new ImageOf<PixelMono>;
    bluePlus = new ImageOf<PixelMono>;
    yellowMinus = new ImageOf<PixelMono>;

    redGreen = new ImageOf<PixelMono>;
    greenRed = new ImageOf<PixelMono>;
    blueYellow = new ImageOf<PixelMono>;
    edges = new ImageOf<PixelMono>;

    lambda = 0.05f;

    resized = false;
}

earlyMotionThread::~earlyMotionThread() {
    delete redPlane;
    delete redPlane2;
    delete redPlane3;
    delete greenPlane;
    delete greenPlane2;
    delete greenPlane3;
    delete bluePlane;
    delete bluePlane2;
    delete bluePlane3;
    delete yellowPlane;
    delete yellowPlane2;
    delete inputExtImage;
    delete inputImageFiltered;
    delete inputImage;

    delete redPlus;
    delete redMinus;
    delete greenPlus;
    delete greenMinus;
    delete bluePlus;
    delete yellowMinus;

    delete redGreen;
    delete greenRed;
    delete blueYellow;
    delete edges;

    
}

bool earlyMotionThread::threadInit() {
    /* open ports */ 
    if (!imagePortIn.open(getName("/image:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imagePortOut.open(getName("/image:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imagePortExt.open(getName("/imageExt:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!rgPort.open(getName("/rg:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!grPort.open(getName("/gr:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!byPort.open(getName("/by:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
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
                filterInputImage();
            }
          
            // extend logpolar input image
            extender(inputImage, maxKernelSize);
            // extract RGB and Y planes
            extractPlanes();
            // gaussian filtering of the of RGB and Y
            filtering();
            // colourOpponency map construction
            colourOpponency();
            // apply sobel operators on the colourOpponency maps and combine via maximisation of the 3 edges
            edgesExtract();
            // sending the edge image on the outport
                 
            // the copy to the port object can be avoided...
            if((edges!=0)&&(imagePortOut.getOutputCount())) {
                imagePortOut.prepare() = *(edges);
                imagePortOut.write();
            }
            if((redGreen!=0)&&(rgPort.getOutputCount())) {
                rgPort.prepare() = *(redGreen);
                rgPort.write();
            }
            if((greenRed!=0)&&(grPort.getOutputCount())) {
                grPort.prepare() = *(greenRed);
                grPort.write();
            }
            if((blueYellow!=0)&&(byPort.getOutputCount())) {
                byPort.prepare() = *(blueYellow);
                byPort.write();
            }
            if((inputExtImage!=0)&&(imagePortExt.getOutputCount())) {
                imagePortExt.prepare() = *(inputExtImage);
                imagePortExt.write();
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
    edges->resize(width_orig, height_orig);
    inputImageFiltered->resize(width_orig, height_orig);
    inputImageFiltered->zero();
 
    inputExtImage->resize(width,height);
    redPlane->resize(width,height);
    redPlane2->resize(width,height);
    redPlane3->resize(width,height);
    greenPlane->resize(width,height);
    greenPlane2->resize(width,height);
    greenPlane3->resize(width,height);
    bluePlane->resize(width,height);
    bluePlane2->resize(width,height);
    bluePlane3->resize(width,height);
    yellowPlane->resize(width,height);
    yellowPlane2->resize(width,height);

    redPlus->resize(width,height);
    redMinus->resize(width,height);
    greenPlus->resize(width,height);
    greenMinus->resize(width,height);
    bluePlus->resize(width,height);
    yellowMinus->resize(width,height);

    redGreen->resize(width, height);
    greenRed->resize(width, height);
    blueYellow->resize(width, height);

    
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
    /* check ipp for the existence of functions with output by plane (rather than by pixel) */
    //Ipp8u* shift[3];
    //Ipp8u* yellowP;
    
    //shift[0] = redPlane->getRawImage();
    //shift[1] = greenPlane->getRawImage();
    //shift[2] = bluePlane->getRawImage();
    //yellowP = yellowPlane->getRawImage();
    //Ipp8u* inputPointer = inputExtImage->getRawImage();

    /* use getPadding!!!! */
    int paddingMono = redPlane->getPadding(); //redPlane->getRowSize()-redPlane->width();
    int padding3C = inputExtImage->getPadding(); //inputExtImage->getRowSize()-inputExtImage->width()*3;

    const int h = inputExtImage->height();
    const int w = inputExtImage->width();

    
}

void earlyMotionThread::filtering() {

}

void earlyMotionThread::colourOpponency() {

}

void earlyMotionThread::threadRelease() {
    resized = false;
}

void earlyMotionThread::onStop() {
    imagePortIn.interrupt();
    imagePortOut.interrupt();
    imagePortExt.interrupt();
    rgPort.interrupt();
    grPort.interrupt();
    byPort.interrupt();
    
    imagePortOut.close();
    imagePortExt.close();
    rgPort.close();
    grPort.close();
    byPort.close();
    imagePortIn.close();
}

