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
 * Public License fo
 r more details
 */

/**
 * @file colourSaliecyThread.cpp
 * @brief Implementation of the thread (see header colourSaliencyThread.h)
 */

#include <iCub/colourSaliencyThread.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10
#define BUCKETLEAKCONST 100

colourSaliencyThread::colourSaliencyThread() : RateThread(THRATE) {
    trained=false;
    resized=false;
    count=0;
    targetRed=0;
    targetGreen=0;
    targetBlue=0;
    inImage=0;
    lambda=0.25;
    trainingVector=new int[4096];
    memset(trainingVector,0,4096 * sizeof(int));
}

colourSaliencyThread::~colourSaliencyThread() {
    delete trainingVector;
}

bool colourSaliencyThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    printf("opening ports.... \n");
    inPort.open(getName("/image:i").c_str());
    outPort.open(getName("/image:o").c_str());
    trainingPort.open(getName("/training:i").c_str());
    return true;
}

void colourSaliencyThread::interrupt() {
    outPort.interrupt();
    inPort.interrupt();
    trainingPort.interrupt();
}

void colourSaliencyThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string colourSaliencyThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void colourSaliencyThread::resize(int widthp, int heightp) {
    width=widthp;
    height=heightp;
}

void colourSaliencyThread::run() {
    count++;
    if(inPort.getInputCount()) {
        inImage=inPort.read(false);
        if((!resized)&&(inImage!=0)) {
            resized=true;
            resize(inImage->width(), inImage->height());
        }
    }
    if((inImage!=0)&&(trainingPort.getInputCount())) {
        trainingImage=trainingPort.read(false);
        if (trainingImage!=0){
            trainingX=trainingImage->width();
            trainingY=trainingImage->height();
            training(trainingImage);
            trained=true;
        }
        if(outPort.getOutputCount() && (trained)) {
            ImageOf<yarp::sig::PixelMono>& outputImage=outPort.prepare();
            outputImage.resize(inImage->width(), inImage->height());
            makeProbImage(*inImage,&outputImage);
            outPort.write();
        }
        if(count%BUCKETLEAKCONST) {
            bucketLeak();
            count=0;
        }
    }
}

void colourSaliencyThread::threadRelease() {
    //closing ports
    inPort.close();
    outPort.close();
    trainingPort.close();
}

void colourSaliencyThread::training(ImageOf<PixelRgb>* trainingImage) {
    unsigned char* pixel=trainingImage->getRawImage();
    int padding=trainingImage->getPadding();
    int rowsize=trainingImage->getRowSize();
    unsigned char trainRed, trainGreen, trainBlue;

    const float ul = 1.0f - lambda;
    //mean values
    /*for(int r=0;r<trainingImage.height();r++) {
        for(int c=0;c<trainingImage.width();c++) {
            trainRed=*pixel;
            pixel++;
            trainGreen=*pixel;
            pixel++;
            trainBlue=*pixel;
            pixel++;
            if((trainRed!=0)||(trainGreen!=0)||(trainBlue!=0)) {
                targetRed = (unsigned char)(lambda * trainRed + ul * targetRed + .5f);
                targetGreen = (unsigned char)(lambda * trainGreen + ul * targetGreen + .5f);
                targetBlue = (unsigned char)(lambda * trainBlue + ul * targetBlue + .5f);
            }
        }
        pixel+=padding;
    }*/
    for(int r=0;r<trainingImage->height();r++) {
        for(int c=0;c<trainingImage->width();c++) {
            trainRed=*pixel;
            pixel++;
            trainGreen=*pixel;
            pixel++;
            trainBlue=*pixel;
            pixel++;
            int posRed=(int)floor((double)trainRed/16.0);
            int posGreen=(int)floor((double)trainGreen/16.0);
            int posBlue=(int)floor((double)trainBlue/16.0);
            if (trainingVector[posRed * 256 + posGreen * 16 + posBlue]<100) {
                trainingVector[posRed * 256 + posGreen * 16 + posBlue]+=1;
            }
        }
        pixel+=padding;
    }
}

void colourSaliencyThread::bucketLeak(){
    for(int j=0;j<4096;j++) {
        if(trainingVector[j]>0) {
            trainingVector[j]--;
        }
    }
}

void colourSaliencyThread::makeProbImage(ImageOf<PixelRgb> imageIn, ImageOf<PixelMono>* imageOut) {
    unsigned char* pixel;
    unsigned char* pixelOut;
    int padding = imageIn.getPadding();
    int paddingOut = imageOut->getPadding();
    int rowsize = imageIn.getRowSize();
    int rowsizeOut = imageOut->getRowSize();
    unsigned char trainRed, trainGreen, trainBlue;

    double sumProb = 0;
    const int particleX = 4;
    const int particleY = 4;
    unsigned char* baseline = imageIn.getRawImage();
    unsigned char* baselineOut = imageOut->getRawImage();
    int maxjr=(int) floor((double)imageIn.height()/particleY);
    int maxjc=(int) floor((double)imageIn.width()/particleX);
    for (int jr=0;jr<maxjr;jr++) {
        for (int jc = 0 ; jc < maxjc ; jc++) {
            pixel = baseline + jc * particleX * 3;
            for(int r = 0 ; r < particleX ; r++) {
                for(int c = 0;c < particleY ; c++) {
                    trainRed = *pixel;
                    pixel++;
                    trainGreen = *pixel;
                    pixel++;
                    trainBlue = *pixel;
                    pixel++;
                    int posRed = (int) floor((double)trainRed / 16.0);
                    int posGreen = (int) floor((double)trainGreen / 16.0);
                    int posBlue = (int) floor((double)trainBlue / 16.0);
                    double den = (trainingX * trainingY);
                    sumProb += (double)(trainingVector[posRed * 256 + posGreen * 16 + posBlue] / den);
                }
                pixel += (rowsize - particleX * 3);
            }
            //printf("%f ", sumProb);
            pixelOut = baselineOut + jc * particleX;
            for(int r = 0 ; r < particleY ; r++) {
                for(int c = 0 ; c < particleX ; c++) {
                    *pixelOut = (int) floor(sumProb * 255);
                    pixelOut++;
                }
                pixelOut += (rowsizeOut - particleX);
            }
            sumProb = 0;
        }
        baseline += rowsize * particleY;
        baselineOut += rowsizeOut * particleY;
    }
}


void colourSaliencyThread::makeDistanceImage(ImageOf<PixelRgb> imageIn, ImageOf<PixelMono>& imageOut) {
    unsigned char* pixelIn = imageIn.getRawImage();
    unsigned char* pixelOut = imageOut.getRawImage();
    int padding = imageIn.getPadding();
    //mean values
    for(int r=0;r<height;r++) {
        for(int c=0;c<width;c++) {
            double distanceRed = *pixelIn++ - targetRed;
            double distanceGreen = *pixelIn++ - targetGreen;
            double distanceBlue = *pixelIn++ - targetBlue;
            double distance = sqrt((double)
                            distanceRed * distanceRed +
                            distanceGreen * distanceGreen +
                            distanceBlue * distanceBlue
                            );
            distance=256 - distance/sqrt(3.0);
            *pixelOut++ = (unsigned char)floor(distance);
        }
        pixelOut += padding;
        pixelIn += padding;
    }
}

void colourSaliencyThread::makeCorrImage(ImageOf<PixelRgb> imageIn, ImageOf<PixelMono> imageOut) {
    unsigned char* pixel=imageIn.getRawImage();
    unsigned char *pixelNext, *pixelPrev, *pixelPRow, *pixelNRow;
    int rowSize=imageIn.getRowSize();
    int padding=imageIn.getPadding();
    Vector num(3);
    Vector den_P(3);
    Vector den_T(3);
    //mean values
    /*
    for(int r=0;r<height;r++) {
        for(int c=0;c<width;c++) {
            
        }
    }
    */
    //computation of the correlation
    for(int r=0;r<height;r++) {
        for(int c=0;c<width;c++) {
            pixelPRow=pixel-rowSize;
            pixelNRow=pixel+rowSize;
            //current Row
            pixelCorr(pixel, num, den_P, den_T);
            pixelNext=pixel+3;
            pixelCorr(pixelNext, num, den_P, den_T);
            pixelPrev=pixel-3;
            pixelCorr(pixelPrev, num, den_P, den_T);
            //previous Row
            if(r>0) {
                pixelCorr(pixelPRow, num, den_P, den_T);
                pixelNext=pixelPRow+3;
                pixelCorr(pixelNext, num, den_P, den_T);
                pixelPrev=pixelPRow-3;
                pixelCorr(pixelPrev, num, den_P, den_T);
            }
            //next Row
            if(r<height-1) {
                pixelCorr(pixelNRow, num, den_P, den_T);
                pixelNext=pixelNRow+3;
                pixelCorr(pixelNext, num, den_P, den_T);
                pixelPrev=pixelNRow-3;
                pixelCorr(pixelPrev, num, den_P, den_T);
            }
            double R_corr = num[0] / sqrt(den_P[0] * den_T[0] + 0.00001);
            double G_corr = num[1] / sqrt(den_P[1] * den_T[1] + 0.00001);
            double B_corr = num[2] / sqrt(den_P[2] * den_T[2] + 0.00001);
            double _corrFunct = (R_corr + G_corr + B_corr) / 3.0;
            //_corrFunct[k] *=;
            //_corrFunct[k] /=;
        }
    }
}


void colourSaliencyThread::pixelCorr(unsigned char* pixel,Vector& num, Vector& den_P, Vector& den_T) {
    //Red
    num[0]   += (*pixel * targetRed);
    den_P[0] += (*pixel * *pixel);
    den_T[0] += (targetRed * targetRed);
    //Green
    pixel++;
    num[1]   += (*pixel * targetGreen);
    den_P[1] += (*pixel * *pixel);
    den_T[1] += (targetGreen * targetGreen);
    //Blue
    pixel++;
    num[2]   += (*pixel * targetBlue);
    den_P[2] += (*pixel * *pixel);
    den_T[2] += (targetBlue * targetBlue);
}

