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
#define _shiftLevels 1

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
                outputImage.resize(252,152);
                outputImage.zero();
                trsfL2C.logpolarToCart(*intermImage,*inputImage);
                shiftROI(intermImage,intermImage2,100,180);
                //printf("copying the image %d %d \n", inputImage->width(), inputImage->height());
                //copy_8u_C1R(inputImage,&outputImage);
                trsfL2C.cartToLogpolar(outputImage, *intermImage2);
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
    int width   = inImg->width();
    int height  = inImg->height(); 
    int centerX = width>>1;
    int centerY = height>>1;
    int dx = x - centerX;
    int dy = y - centerY;
    int channel = inImg->getPixelSize();
    printf("dx %d dy %d \n", dx,dy);
    unsigned char* pinput  = inImg->getRawImage();
    unsigned char* poutput = outImg->getRawImage();
    int padding = inImg->getPadding();
    int rowsize = inImg->getRowSize();
 
    int direction;
    if(dx > 0) {
        direction = -1;
        printf("jumping in x \n");
        pinput += channel * dx;
    }
    else {
        direction = 1;
    }
    
    if(dy > 0) {
        pinput += dy * rowsize;
    }
    for(int row = 0; row < height; row++) {        
        if((row + dy <= 0)||(row + dy >= height)){                        
            for(int col = 0; col < inImg->width(); col++) {
                //zero
                *poutput = (unsigned char) 0;
                poutput++;
                *poutput = (unsigned char) 0;
                poutput++;
                *poutput = (unsigned char) 0;
                poutput++;
            }
            poutput += padding;
        }        
        else {
            for(int col = 0; col < width; col++) {
                if((col + dx <= 0)||(col + dx > width)){
                    //zero
                    *poutput++ = (unsigned char) 0;
                    *poutput++ = (unsigned char) 0;
                    *poutput++ = (unsigned char) 0;
                }
                else {
                    //copying
                    *poutput = *pinput; //red
                    poutput++; pinput++;
                    *poutput = *pinput; //green
                    poutput++; pinput++;
                    *poutput = *pinput; //blue
                    poutput++; pinput++;
                }
            }

            poutput += padding;            
            pinput  += ( rowsize -  (width + dx * direction)  * channel + channel * direction);
        }        
    }        
}


void sacPlannerThread::logCorrRgbSum(ImageOf<PixelRgb>* imgA, ImageOf<PixelRgb>* imgB,double* pCorr, int step) {
    int _actRings    = imgA->height();
    int _sizeTheta  = imgA->width();
    if((_actRings!=imgB->height())||(_sizeTheta!=imgB->width())) {
        printf("ERROR : the dimension of the two images do not match! \n");
        return;
    }
   

    unsigned char* aPtr = imgA->getRawImage();
    unsigned char* bPtr = imgB->getRawImage();
    int padding = imgA->getPadding();
    int _count[_shiftLevels];
    double _corrFunct[_shiftLevels];
    pCorr = &_corrFunct[0]; 
    

    //Correlation Function Computation
    for (int k = 0; k < _shiftLevels; k++) {
        
        int iA, iB;

        double average_Ar = 0;
        double average_Ag = 0;
        double average_Ab = 0;
        double average_Br = 0;
        double average_Bg = 0;
        double average_Bb = 0;
        
        double R_corr = 0;
        double G_corr = 0;
        double B_corr = 0;
        
        double pixelA = 0;
        double pixelB = 0;
        
        // k1 = k *_img.Size_LP;  ???????
        
        /* for (j = 0; j < _actRings; j++) {
           
           for (i = 0; i < _img.Size_Theta; i++) {

                iR = _shiftMap[k1 + j *_img.Size_Theta + i];
                iL = 3 * (j*_img.Size_Theta + i);
                
                if (iR > 0) {
                 average_Lr += lPtr[iL];
                 average_Rr += rPtr[iR];
                 //average_Lg += lPtr[iL+1];
                 //average_Rg += rPtr[iR+1];
                 //average_Lb += lPtr[iL+2];
                 //average_Rb += rPtr[iR+2];
                }
           }
        }*/

        if (_count[k] != 0) {
            average_Ar /= _count[k];
            average_Br /= _count[k];
            average_Ag /= _count[k];
            average_Bg /= _count[k];
            average_Ab /= _count[k];
            average_Bb /= _count[k];
        }

        double numr   = 0;
        double den_Ar = 0;
        double den_Br = 0;
        double numg   = 0;
        double den_Ag = 0;
        double den_Bg = 0;
        double numb   = 0;
        double den_Ab = 0;
        double den_Bb = 0;

        for (int j = _actRings - 1; j >= 0; j-=step) {
            for (int i = _sizeTheta - 1; i >= 0; i-=step) {

                //iR = _shiftMap[k1 + j*_img.Size_Theta + i];
                iA = 3 * (j * _sizeTheta + i); 
                iB = 3 * (j * _sizeTheta + i);
                aPtr += iA;
                bPtr += iB;

                if (iA > 0) {
                    //Red
                    pixelA = aPtr[iA] - average_Ar;
                    pixelB = bPtr[iB] - average_Br;
                    numr   += (pixelA * pixelB);
                    den_Ar += (pixelA * pixelA);
                    den_Br += (pixelB * pixelB);
                    //Green
                    pixelA = aPtr[iA+1] - average_Ag;
                    pixelB = bPtr[iB+1] - average_Bg;
                    numg   += (pixelA * pixelB);
                    den_Ag += (pixelA * pixelA);
                    den_Bg += (pixelB * pixelB);
                    //Blue
                    pixelA = aPtr[iA+2] - average_Ab;
                    pixelB = bPtr[iB+2] - average_Bb;
                    numb   += (pixelA * pixelB);
                    den_Ab += (pixelA * pixelA);
                    den_Bb += (pixelB * pixelB);
                }
            }
        }
        R_corr = numr / sqrt(den_Ar * den_Br + 0.00001);
        G_corr = numg / sqrt(den_Ag * den_Bg + 0.00001);
        B_corr = numb / sqrt(den_Ab * den_Bb + 0.00001);
        _corrFunct[k] = (R_corr + G_corr + B_corr) / 3.0;
        _corrFunct[k] *= _count[k];
        double _maxCount = 255.0;             // normalisation hard coded
        _corrFunct[k] /= _maxCount;  // normalisation
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
