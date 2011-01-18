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
 * @file stereoAttThread.cpp
 * @brief Implementation of the stereo attentive thread (see visualFilterThread.h).
 */

//#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/stereoAttThread.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;

const int maxKernelSize = 5;
#define THRATE 100

stereoAttThread::stereoAttThread():RateThread(THRATE){
    resized=false;
    inputLeft=0;
    inputRight=0;
    shiftvalue=0;
}

stereoAttThread::stereoAttThread(int delay):RateThread(delay){
    resized=false;
    inputLeft=0;
    inputRight=0;
    shiftvalue=0;
}

stereoAttThread::~stereoAttThread() {
    delete inputLeft;
    delete inputRight;
}

bool stereoAttThread::threadInit() {
    /* open ports */ 
    inLeftPort.open(getName("/left:i").c_str());
    inRightPort.open(getName("/right:i").c_str());
    shiftPort.open(getName("/shift:i").c_str());
    vergenceInPort.open(getName("/vergence:i").c_str());
    outPort.open(getName("/image:o").c_str());

    //initializing gazecontrollerclient
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    option.put("local","/clientStereoVision/gaze");

    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }

    return true;
}

void stereoAttThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string stereoAttThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void stereoAttThread::resize(int widthp, int heightp) {
    height=heightp;
    width=widthp;
    srcsize.width=widthp;
    srcsize.height=heightp;

    inputLeft=new ImageOf<PixelRgb>;
    inputLeft->resize(width,height);
    inputRight=new ImageOf<PixelRgb>;
    inputRight->resize(width,height);
}

void stereoAttThread::run() {
    //read port in sequence
    tmp=inLeftPort.read(false);
    if(!resized) {
        if(tmp!=0) {
            resize(tmp->width(), tmp->height());
            resized=true;
        }
        else {
            return;
        }
    }
    if(tmp!=0)
        ippiCopy_8u_C3R(tmp->getRawImage(),tmp->getRowSize(),inputLeft->getRawImage(), inputLeft->getRowSize(),srcsize);
    tmp=inRightPort.read(false);
    if(tmp!=0) {
        ippiCopy_8u_C3R(tmp->getRawImage(),tmp->getRowSize(),inputRight->getRawImage(), inputRight->getRowSize(),srcsize);
    }
    Bottle* b=shiftPort.read(false);
    ImageOf<PixelRgb>& outImage=outPort.prepare();
    outImage.resize(width,height);
    if(b!=0) {
        shiftvalue=b->get(0).asInt();
        b->clear();
    }
    //shift images and fuse them
    fuse(outImage);

    if(vergenceInPort.getInputCount()) {
        int joint=0,angle=0;
        int first,second;
        Bottle* b=vergenceInPort.read(false);
        if(b!=0) {
            first=b->get(0).asVocab();
            second=b->get(1).asVocab();
            joint=b->get(2).asInt();
            angle=b->get(3).asInt();
        }
    }
    outPort.write();
}

/**
* cartesian shift of the right image over the left for a certain ammount of pixels
*/
void stereoAttThread::shift(int shift, ImageOf<PixelRgb>& outImage) {
    unsigned char* pRight=inputRight->getRawImage();
    unsigned char* pLeft=inputLeft->getRawImage();
    int padding=inputLeft->getPadding();
    unsigned char* pOutput=outImage.getRawImage();
    if(shift>=0) {
        for (int row=0;row<height;row++) {
            pRight+=shift*3;
            for (int col=0;col<width-shift;col++) {
                *pOutput=(unsigned char) floor(0.5 * *pLeft + 0.5 * *pRight);
                pLeft++;pRight++;pOutput++;
                *pOutput=(unsigned char) floor(0.5 * *pLeft + 0.5 * *pRight);
                pLeft++;pRight++;pOutput++;
                *pOutput=(unsigned char) floor(0.5 * *pLeft + 0.5 * *pRight);
                pLeft++;pRight++;pOutput++;
            }
            for(int col=width-shift;col<width;col++){
                *pOutput++=*pLeft++;
                *pOutput++=*pLeft++;
                *pOutput++=*pLeft++;
            }
            //padding
            pLeft+=padding;
            pRight+=padding;
            pOutput+=padding;
        }
    }
    else {
        shift=0-shift;
        for (int row=0;row<height;row++) {
            pLeft+=shift*3;
            for (int col=0;col<width-shift;col++) {
                *pOutput=(unsigned char) floor(0.5 * *pLeft + 0.5 * *pRight);
                pLeft++;pRight++;pOutput++;
                *pOutput=(unsigned char) floor(0.5 * *pLeft + 0.5 * *pRight);
                pLeft++;pRight++;pOutput++;
                *pOutput=(unsigned char) floor(0.5 * *pLeft + 0.5 * *pRight);
                pLeft++;pRight++;pOutput++;
            }
            for(int col=width-shift;col<width;col++){
                *pOutput++=*pRight++;
                *pOutput++=*pRight++;
                *pOutput++=*pRight++;
            }
            //padding
            pLeft+=padding;
            pRight+=padding;
            pOutput+=padding;
        }
    }
}

/**
* cartesian shift of the right image over the left for a certain ammount of pixels
*/
void stereoAttThread::fuse(ImageOf<PixelRgb>& outImage) {
    unsigned char* pRight=inputRight->getRawImage();
    unsigned char* pLeft=inputLeft->getRawImage();
    int padding=inputLeft->getPadding();
    unsigned char* pOutput=outImage.getRawImage();
    for (int row=0;row<height;row++) {
        for (int col=0;col<width;col++) {
            *pOutput=(unsigned char) floor(0.5 * *pLeft + 0.5 * *pRight);
            pLeft++;pRight++;pOutput++;
            *pOutput=(unsigned char) floor(0.5 * *pLeft + 0.5 * *pRight);
            pLeft++;pRight++;pOutput++;
            *pOutput=(unsigned char) floor(0.5 * *pLeft + 0.5 * *pRight);
            pLeft++;pRight++;pOutput++;
        }
        //padding
        pLeft+=padding;
        pRight+=padding;
        pOutput+=padding;
    }
}


void stereoAttThread::threadRelease() {
    //close ports
    inLeftPort.close();
    inRightPort.close();
    outPort.close();
    shiftPort.close();
    vergenceInPort.close();

    delete clientGazeCtrl;
}


