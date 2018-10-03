// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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

#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/earlyMotionThread.h>
#include <cstring>
#include <cv.h>
#include <opencv2/imgproc/imgproc.hpp>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

const int maxKernelSize = 5;
const int increment     = 50;

template<class T>
inline T max(T a, T b, T c) {    
    if(b > a) a = b;
    if(c > a) a = c;
    return a;
}

earlyMotionThread::earlyMotionThread() {
    count = 1;
    inputExtImage      = new ImageOf<PixelRgb>;
    inputImageFiltered = new ImageOf<PixelMono>;
    motion             = new ImageOf<PixelMono>;
    lambda = 0.8f;     // weight of the current image
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

void earlyMotionThread::resize(int width_orig,int height_orig) {
    this->width_orig  = width_orig;
    this->height_orig = height_orig;
    this->width  = width_orig  + 2 * maxKernelSize;
    this->height = height_orig + maxKernelSize;

    // resizing the ROI
    //originalSrcsize.height = height_orig;
    //originalSrcsize.width = width_orig;
    //srcsize.width = width;
    //srcsize.height = height;

    // resizing plane images
    motion->resize(width_orig, height_orig);


    imageT1 = new ImageOf<PixelMono>;
    imageT1->resize(width_orig, height_orig);
    imageT1->zero();
    imageT2 = new ImageOf<PixelMono>;
    imageT2->resize(width_orig, height_orig);
    imageT2->zero();
    imageT3 = new ImageOf<PixelMono>;
    imageT3->resize(width_orig, height_orig);
    imageT3->zero();
    imageT4 = new ImageOf<PixelMono>;
    imageT4->resize(width_orig, height_orig);
    imageT4->zero();
        
    inputImageFiltered->resize(width_orig, height_orig);
    inputImageFiltered->zero();
 
    inputExtImage->resize(width,height);   
}

void earlyMotionThread::run() {
    while (isStopping() != true && !suspended) {
        count++;
        inputImage = imagePortIn.read(true);
        
        if (inputImage != NULL) {
            if (!resized) {
                resize(inputImage->width(), inputImage->height());
                resized = true;
            }
            else {
                filterInputImage();
            }
            
            
            
            // sending the edge image on the outport                 
            // the copy to the port object can be avoided...
            if((motionPort.getOutputCount())) {
                ImageOf<PixelMono>& out = motionPort.prepare();
                out.resize(width_orig, height_orig);
                out.zero();

                
                // extend logpolar input image
                //extender(inputImage, maxKernelSize);
                
                //extractPlanes();
                
                temporalSubtraction(&out);

                motionPort.write();
            }            
            if(count % 1 == 0) {
                temporalStore();
                count=1;
            }
        }
    }
}


void earlyMotionThread::filterInputImage() {
    int i;
    const int sz = inputImage->getRawImageSize();
    unsigned char * pFiltered = inputImageFiltered->getRawImage();
    unsigned char * pCurr     = inputImage->getRawImage();
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
    int padding3C = inputExtImage->getPadding(); 

    const int h = inputExtImage->height();
    const int w = inputExtImage->width();
    
}

void earlyMotionThread::temporalStore() {
    int padding = inputImage->getPadding();
    unsigned char* pin      = inputImage->getRawImage();
    unsigned char* pimageT1 = imageT1->getRawImage();
    unsigned char* pimageT2 = imageT2->getRawImage();
    unsigned char* pimageT3 = imageT3->getRawImage();
    unsigned char* pimageT4 = imageT4->getRawImage();
 
    for(int row = 0; row < height_orig; row++) {
        for(int col = 0; col < width_orig ; col++) {
            *pimageT4 = *pimageT3;
            *pimageT3 = *pimageT2;
            *pimageT2 = *pimageT1;
            *pimageT1 = *pin;    
            pimageT1++;
            pimageT2++;
            pimageT3++;
            pimageT4++;
            pin++;
        }
        pin  += padding;
        pimageT1  += padding;
        pimageT2  += padding;
        pimageT3  += padding;
        pimageT4  += padding;
    }
}

void  earlyMotionThread::_medianfilter(const element* signal, element* result, int N)
{
    //   Move window through all elements of the signal
    for (int i = 2; i < N - 2; ++i)
    {
        //   Pick up window elements
        element window[5];
        for (int j = 0; j < 5; ++j)
            window[j] = signal[i - 2 + j];
        //   Order elements (only half of them)
        for (int j = 0; j < 3; ++j)
        {
            //   Find position of minimum element
            int min = j;
            for (int k = j + 1; k < 5; ++k)
                if (window[k] < window[min])
                    min = k;
            //   Put found minimum element in its place
            const element temp = window[j];
            window[j] = window[min];
            window[min] = temp;
        }
        //   Get result - the middle element
        result[i - 2] = window[2];
    }
}

void  earlyMotionThread::medianfilter(element* signal, element* result, int N)
{
    //   Check arguments
    if (!signal || N < 1)
        return;
    //   Treat special case N = 1
    if (N == 1)
    {
        if (result)
            result[0] = signal[0];
        return;
    }
    //   Allocate memory for signal extension
    element* extension = new element[N + 4];
    //   Check memory allocation
    if (!extension)
        return;
    //   Create signal extension
    memcpy(extension + 2, signal, N * sizeof(element));
    for (int i = 0; i < 2; ++i)
    {
        extension[i] = signal[1 - i];
        extension[N + 2 + i] = signal[N - 1 - i];
    }
    //   Call median filter implementation
    _medianfilter(extension, result ? result : signal, N + 4);
    //   Free memory
    delete[] extension;
}


void earlyMotionThread::temporalSubtraction(ImageOf<PixelMono>* outputImage) {
    int padding = inputImage->getPadding();
    int rowsize = inputImage->getRowSize();
    
    unsigned char* pin  = inputImageFiltered->getRawImage();
    unsigned char* pout = outputImage->getRawImage();
    unsigned char* pimageT1 = imageT1->getRawImage();
    unsigned char* pimageT2 = imageT2->getRawImage();
    unsigned char* pimageT3 = imageT3->getRawImage();
    unsigned char* pimageT4 = imageT4->getRawImage();    
    
    unsigned char diff10, diff21, diff32, diff43, diff20, diff30, diff40;
    unsigned char max = 0;
    for(int row = 0; row < height_orig; row++) {
        for(int col = 0; col < width_orig ; col++) {
            diff10 = (*pin      - *pimageT1) * (*pin      - *pimageT1);
            diff21 = (*pimageT2 - *pimageT1) * (*pimageT2 - *pimageT1);
            diff32 = (*pimageT3 - *pimageT2) * (*pimageT3 - *pimageT2);
            diff43 = (*pimageT4 - *pimageT3) * (*pimageT4 - *pimageT3);
            diff20 = (*pin      - *pimageT2) * (*pin      - *pimageT2);
            diff30 = (*pin      - *pimageT3) * (*pin      - *pimageT3);
            diff40 = (*pin      - *pimageT4) * (*pin      - *pimageT4);

            
            //*pout += (unsigned char) floor(sqrt(diff10 + diff20 + diff30 + diff40 + diff21 + diff32 ) * (exp( (2.3 * row)   / (double)height_orig) - 1));
            *pout += (unsigned char) floor(
                                           (  sqrt((double)diff10) + sqrt((double)diff32) )
                                           *
                                           (exp((2.3 * 150)   / (double)height_orig) - 1));

            if(*pout > max) {
                max = *pout;
            }
            
            if(*pout >= threshold){
//               printf("255 \n");
               *pout = 255;
               *(pout + 1) = 255;
               *(pout - 1) = 255;
               *(pout - rowsize) = 255;
               *(pout + rowsize) = 255;
            }                     
            pout++;
            pin++;
            pimageT1++;
            pimageT2++;
            pimageT3++;
            pimageT4++;
        }
        pin      += padding;
        pout     += padding;
        pimageT1 += padding;
        pimageT2 += padding;
        pimageT3 += padding;
        pimageT4 += padding;
    }

    // Rea & Jonas 10/08/2018 + Add median filter to filter out noises
    medianfilter(pout,pout, 3);

    //printf("max %d \n", max);
    
    //ImageOf<PixelFloat>* imageTmp = new ImageOf<PixelFloat>;
    //imageTmp->resize(width_orig,height_orig);
    //convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar> convHoriz(9,&G9[0],0,0.8,-50);  
    //convHoriz.convolve1D(outputImage,imageTmp);
    //convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar> convVert(9,&G9[0],1,0.8,-50);  
    //convHoriz.convolve1D(imageTmp,outputImage);

    //convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelFloat>,float> kirschConvolution0(3,3,K1,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    //kirschConvolution0.convolve2D(outputImage,imageTmp);
    //pout = outputImage->getRawImage();
    //float* ptmp = (float*)imageTmp->getRawImage();
    //int paddingTmp = imageTmp->getPadding();
    //for(int row = 0; row < height_orig; row++) {
    //    for(int col = 0; col < width_orig ; col++) {
    //        *pout++ = floor(*ptmp++ * 255.0);
    //   }
    //    pout += padding;
    //       ptmp += paddingTmp;
    //}
    //delete imageTmp;
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
void earlyMotionThread::setThreshold(unsigned char threshold) {
    earlyMotionThread::threshold = threshold;
}
unsigned char earlyMotionThread::getThreshold() const {
    return threshold;
}

