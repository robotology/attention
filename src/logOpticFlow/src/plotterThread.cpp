// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Rea Francesco
  * email:francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
  */
/**
 * @file plotterThread.cpp
 * @brief Implementation of the thread for visualization of the flow (see logOFThread.h).
 */

#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/plotterThread.h>

#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

template<class T>
inline T max(T a, T b, T c) {    
    if(b > a) a = b;
    if(c > a) a = c;
    return a;
}

inline void depthConvert(ImageOf<PixelFloat>* src, ImageOf<PixelMono>* dest) {
    float* psrc = (float*) src->getRawImage();
    printf("got the pointer src \n");
    unsigned char* pdest = dest->getRawImage();
    printf("got the pointer dest \n");
    int height = src->height();
    int width  = src->width();
    int destPadding = dest->getPadding();
    int srcPadding  = src->getPadding();
    printf("got dimensions \n");
    float max = 1.17; float min = 3.40; 
    printf("before the loop \n");
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            if(*psrc < min) min = *psrc;
            if(*psrc > max) max = *psrc;
            psrc++;
        }
        psrc += srcPadding;
    }

    float m = (max - min) / 255;
    float q = -min;

    printf("setting the value of the dest image \n");

    psrc = (float*) src->getRawImage();
    for (int r = 0; r < height; r++) {
        for (int c = 0; c < width; c++) {
            *pdest++ = (unsigned char) floor((*psrc + q) / m);
        }
        pdest += destPadding;
    }        

    printf("after setting the dest image \n");
    
}

inline void copyImage(ImageOf<PixelRgb>* src,ImageOf<PixelRgb>* dest) {
    unsigned char* srcp  = src->getRawImage();
    unsigned char* destp = dest->getRawImage();
    int height  = src->height();
    int rowSize = src->getRowSize();
    memcpy(destp,srcp,height * rowSize * sizeof(char));
}

inline void  copyImage(ImageOf<PixelMono>* src,ImageOf<PixelMono>* dest) {
    unsigned char* srcp = src->getRawImage();
    unsigned char* destp = dest->getRawImage();
    
    int height  = src->height();
    int rowSize = src->getRowSize();

    /*int width   = src->width();
    int padding = src->getPadding();
    for (int row = 0; row < height; row++) {
        for(int col = 0; col < width; col++) {
            *destp++ = *srcp++;
            *destp++ = *srcp++;
            *destp++ = *srcp++;
        }
        destp += padding;
        srcp  += padding;
        }*/

    memcpy(destp,srcp,height * rowSize * sizeof(char));
}



plotterThread::plotterThread():RateThread(RATE_OF_PLOTTER_THREAD) {
    printf("plotterThread::RateThread() \n");
    count = 0;

    //calcXSem = new Semaphore *[COUNTCOMPUTERSX * COUNTCOMPUTERSY]; 
    //calcYSem = new Semaphore *[COUNTCOMPUTERSX * COUNTCOMPUTERSY]; 
    calcSem  = new Semaphore *[COUNTCOMPUTERSX * COUNTCOMPUTERSY];
    reprSem  = new Semaphore *[COUNTCOMPUTERSX * COUNTCOMPUTERSY];
    tempSem  = new Semaphore *[COUNTCOMPUTERSX * COUNTCOMPUTERSY];

    for(int i = 0; i < COUNTCOMPUTERSX * COUNTCOMPUTERSY; i++) {
        //calcXSem[i] = new Semaphore();
        //calcYSem[i] = new Semaphore();
        calcSem[i]  = new Semaphore();
        reprSem[i]  = new Semaphore();
        tempSem[i]  = new Semaphore();
    }

    img = 0;
    
    inputImage          = new ImageOf<PixelRgb>;
    outputImage         = new ImageOf<PixelRgb>;
    flowImage           = new ImageOf<PixelRgb>;
    finalOutputImage    = new ImageOf<PixelRgb>;
    filteredInputImage  = new ImageOf<PixelRgb>;
    extendedInputImage  = new ImageOf<PixelRgb>;    
    Rplus               = new ImageOf<PixelMono>;
    Rminus              = new ImageOf<PixelMono>;
    Gplus               = new ImageOf<PixelMono>;
    Gminus              = new ImageOf<PixelMono>;
    Bplus               = new ImageOf<PixelMono>;
    Bminus              = new ImageOf<PixelMono>;
    Yminus              = new ImageOf<PixelMono>;    
  
    YofYUV              = new ImageOf<PixelMono>;    
    intensImg           = new ImageOf<PixelMono>;
    intensXGrad         = new ImageOf<PixelMono>;
    intensYGrad         = new ImageOf<PixelFloat>;
    intXgrad8u          = new ImageOf<PixelMono>;
    intYgrad8u          = new ImageOf<PixelMono>;
    //gradientImgXCopy    = new ImageOf<PixelMono>;
    //gradientImgYCopy    = new ImageOf<PixelMono>;
    intensImgCopy       = new ImageOf<PixelMono>;
    prevIntensImg       = new ImageOf<PixelMono>;
    unXtnIntensImg      = new ImageOf<PixelMono>;   
    
    redPlane            = new ImageOf<PixelMono>;
    greenPlane          = new ImageOf<PixelMono>;
    bluePlane           = new ImageOf<PixelMono>;
    yellowPlane         = new ImageOf<PixelMono>;

    Yplane              = new ImageOf<PixelMono>;
    Uplane              = new ImageOf<PixelMono>;
    Vplane              = new ImageOf<PixelMono>;
    
    unXtnYplane         = new ImageOf<PixelMono>;
    unXtnUplane         = new ImageOf<PixelMono>;
    unXtnVplane         = new ImageOf<PixelMono>;
    
    YofYUVpy            = new ImageOf<PixelMono>;
    UofYUVpy            = new ImageOf<PixelMono>;
    VofYUVpy            = new ImageOf<PixelMono>;
    RplusUnex           = new ImageOf<PixelMono>;
    GplusUnex           = new ImageOf<PixelMono>;
    BplusUnex           = new ImageOf<PixelMono>;

    tmpMonoLPImage      = new ImageOf<PixelMono>;

    gaborPosHorConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,0,.5,0);
    gaborPosVerConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,1,.5,0);
    gaborNegHorConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,0,.5,0);
    gaborNegVerConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,1,.5,0);  
    gradientHorConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(3,3,Sobel2DXgrad_small,0.6,-50,0);
    gradientVerConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(3,3,Sobel2DYgrad_small,0.6,-50,0);

    for (int i = 0; i < COUNTCOMPUTERSX * COUNTCOMPUTERSY; i++ ) {
        computersValueU[i] = (short*) malloc(dimComput * dimComput * sizeof(short));
        computersValueV[i] = (short*) malloc(dimComput * dimComput * sizeof(short));
        int row = floor( (double) i  / COUNTCOMPUTERSX);
        int col = i  - row * COUNTCOMPUTERSX;
        posXi[i] = 12 * row + (6 + 1) ;
        posGamma[i] = 12 * col + (6 + 5);
    }    
    
    lambda  = 0.3f;
    resized = false;
    isYUV   = true;
}

plotterThread::~plotterThread() {
    printf("plotterThread::~plotterThread() \n");      
}

bool plotterThread::threadInit() {
    printf(" \n opening ports by main thread\n");
    
    /* open ports */        
    if (!flowPort.open(getName("/flow2:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  

    return true;
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

void plotterThread::waitSemaphores(Semaphore** pointer) {
    Semaphore* p;
    for (int i = 0; i < COUNTCOMPUTERSX * COUNTCOMPUTERSY; i++) {
        p = pointer[i];
        p->wait();
    }
}

void plotterThread::postSemaphores(Semaphore** pointer) {
    Semaphore* p;
    for (int i = 0; i < COUNTCOMPUTERSX * COUNTCOMPUTERSY; i++) {
        p = pointer[i];
        p->post();
    }
}

void plotterThread::run() {
    //printf("plotterThread::run \n");
    flowImage->zero();
    representOF();
    
    if((flowImage!=0)&&(flowPort.getOutputCount())) {               
        flowPort.prepare() = *(flowImage);                         
        flowPort.write();
    }
}

int plotterThread::countPositiveValue(short* valuePointer){
    short* tmp = valuePointer;
    int count = 0;
    for(int i = 0; i < dimComput * dimComput; i++) {
        if(abs(*tmp) > 1.0) {
            count++;
        }
        tmp++;
    }
    return count;
}

void plotterThread::representOF(){
    
    unsigned char* tempPointer;    
    tempPointer = represPointer;
    //int rowSizeC = (((252 + 5)/ 8)+ 1)  * 8;
    //printf("representing OF %d %d  \n", rowSizeC, rowSize);

    if(represPointer!=0) {
        //representing the limits
        //printf("image pointer %x \n", represPointer);
        /*
        tempPointer  = represPointer + (posXi * rowSize + posGamma) * 3;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 0;   tempPointer++;
        */
        
        /*
        tempPointer  = represPointer + ((posXi - width) * rowSize + (posGamma - height)) * 3;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        
        tempPointer  = represPointer + ((posXi - width) * rowSize + (posGamma + height)) * 3;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        
        tempPointer  = represPointer + ((posXi + width ) * rowSize + (posGamma - height)) * 3;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        
        tempPointer  = represPointer + ((posXi + width ) * rowSize + (posGamma + height)) * 3;
        *tempPointer = 0;   tempPointer++;
        *tempPointer = 255; tempPointer++;
        *tempPointer = 0;   tempPointer++;
        */

        int sumGamma, sumXi;
        int valueGamma, valueXi;
        int meanGamma,  meanXi;
        
        double uSingle, vSingle;
        //int rowSize = flowImage->getRowSize();
        
        for (int visitComputer = 67; visitComputer < COUNTCOMPUTERSX * COUNTCOMPUTERSY; visitComputer++) {

            tempPointer = represPointer + (posXi[visitComputer] - calcHalf) * rowSize 
                                           + (posGamma[visitComputer] - calcHalf) * 3 ;

            short* valuePointerU = computersValueU[visitComputer];
            int countU = countPositiveValue(valuePointerU);
            short* valuePointerV = computersValueV[visitComputer];
            int countV = countPositiveValue(valuePointerV);
            int magnitude;
            
            
            if((countU < 5) && (countV < 5)) {
                continue;
            }
            

            int countGamma = 0, countXi = 0;
            sumGamma = 0; sumXi = 0;

            for(int i = 0; i < dimComput; i++) {
                for(int j =0; j< dimComput; j++) {
                    uSingle = *valuePointerU;
                    vSingle = *valuePointerV;
                    valueGamma = *valuePointerU;
                    valueXi    = *valuePointerV;
                    valuePointerU++;
                    valuePointerV++;
                    
                    //printf("uSingle vSingle %d %d \n", valueGamma,valueXi);

                    magnitude = valueXi * valueXi + valueGamma * valueGamma;
                    //tempPointer  = represPointer + (((posXi + j - calcHalf )* rowSize) + posGamma + i - calcHalf) * 3;
                    if(magnitude > 1000) {
                        //cvLine(represenIpl,
                        //               cvPoint(posGamma + i - calcHalf, posXi + j - calcHalf), 
                        //               cvPoint(posGamma + i - calcHalf + valueGamma, posXi + j - calcHalf + valueXi),                        
                        //               cvScalar(0,0,255,0)); 
                        
                        
                        
                        //red channel
                        //*tempPointer++ = 255;
                        if(valueXi > 0)
                            *tempPointer = 255 * magnitude;
                        else
                            *tempPointer = 127 * magnitude;
                        tempPointer++;
                        //*tempPointer++ = 255;
                        // green channel
                        if(valueGamma > 0)
                            *tempPointer = 255 * magnitude;
                        else
                            *tempPointer = 127 * magnitude;
                        tempPointer++;
                        // blue channel
                        *tempPointer = 0  ;   tempPointer++;
                        /*
                        if(magnitude > 2000) {
                            cvLine(represIpl,
                                   cvPoint(posGamma[visitComputer] + i - calcHalf, posXi[visitComputer] + j - calcHalf), 
                                   cvPoint(posGamma[visitComputer] + i - calcHalf + valueGamma / 10, posXi[visitComputer] + j - calcHalf + valueXi /10),
                                   cvScalar(0,0,255,0));  
                        }
                        */
                        
                    }                    
                    else {
                        /*
                        *tempPointer = 0;   tempPointer++;
                        if(*tempPointer > 0 ){
                            *tempPointer = *tempPointer - 1; 
                        }
                        else{
                            *tempPointer = *tempPointer;
                        }
                        tempPointer++;
                        *tempPointer = 0;   tempPointer++;
                        */
                        
                        tempPointer += 3;                    
                    }
                    
                    
                    //printf("Single: %f %f \n", uSingle, vSingle);
                    
                    if(valueGamma!=0) {
                        sumGamma += valueGamma;
                        countGamma++;
                    }   
                    if(valueXi!=0){
                        sumXi    += valueXi;
                        countXi++;
                    }
                    
                    
                    /*
                    if(
                       (posGamma[visitComputer] + valueGamma < 0)       || (posXi[visitComputer] + valueXi < 0) ||
                       (posGamma[visitComputer] + valueGamma > rowSize) || (posXi[visitComputer] + valueXi > 152)
                       ){
                        //printf("line out of image boundaries \n");
                    }
                    else {
                        if((valueGamma != 0)&&(valueXi != 0)) {
                            
                            //printf("endPoint %f %f %d %d \n",uSingle, vSingle, endPointX, endPointY);
                            
                            //  cvLine(represenIpl,
                            //  cvPoint(posGamma + i - calcHalf, posXi + j - calcHalf), 
                            //  cvPoint(posGamma + i - calcHalf + valueGamma, posXi + j - calcHalf + valueXi),
                            //  cvScalar(0,0,255,0));   
                            
                            
                            //tempPointer  = represPointer + (posXi  * rowSize + posGamma)  * 3;
                            
                            //tempPointer  = represPointer + (((posXi + j - calcHalf )* rowSize) + posGamma + i - calcHalf) * 3;
                            //*tempPointer = 255; tempPointer++;
                            //*tempPointer = 0  ; tempPointer++;
                            //*tempPointer = 0  ; tempPointer++;
                            
                        }
                    }             
                    */

                } //end for j
                tempPointer += rowSize - dimComput * 3;
            } // end for i

             
            //printf("sumGamma %f sumXi %f  countGamma %d  countXi %d \n", sumGamma, sumXi, countGamma, countXi);
            if(sumGamma!=0) {
                meanGamma  =  (int) (sumGamma / countGamma);
            }
            else{
                meanGamma = 0;
            }
            if (sumXi != 0) {
                meanXi     =  (int) (sumXi    / countXi   );
            }
            else {
                meanXi = 0;
            }
            //printf("------------------ value: %d/%d %d/%d \n", sumGamma,countGamma, sumXi, countXi);
            //printf("------------------ mean ( %d %d) \n", meanGamma, meanXi);
            
            if((abs(meanGamma) > 100) || (abs(meanXi) > 100)) {
                cvLine(represIpl,
                       cvPoint(posGamma[visitComputer] , posXi[visitComputer]), 
                       cvPoint(posGamma[visitComputer] + meanGamma, posXi[visitComputer] + meanXi),
                       cvScalar(255,0,0,0));
            }        
            
        } // end visitComputer
        
    }
    else {
        printf("null pointer \n");
    }
}




void plotterThread::resize(int width_orig,int height_orig) {

    this->width_orig  = inputImage->width(); //width_orig;
    this->height_orig = inputImage->height();//height_orig;
    
    width  = this->width_orig   + maxKernelSize * 2;
    height = this->height_orig  + maxKernelSize;
    printf("expressing width and height %d %d \n", width, height);
    
    //resizing yarp image 
    
    filteredInputImage->resize(width_orig,height_orig);
    extendedInputImage->resize(width, height);
    outputImage->resize(width, height);
    flowImage->resize(width, height);
    finalOutputImage->resize(width, height);
    Rplus->resize(width, height);
    Rminus->resize(width, height);
    Gplus->resize(width, height);
    Gminus->resize(width, height);
    Bplus->resize(width, height);
    Bminus->resize(width, height);
    Yminus->resize(width, height);
    
    tmpMonoLPImage->resize(width, height);
    intensImg->resize(width, height);
    intensXGrad->resize(width, height);
    intensYGrad->resize(width, height);
    intXgrad8u->resize(width, height);
    intYgrad8u->resize(width, height);
    //gradientImgXCopy->resize(width, height);
    //gradientImgYCopy->resize(width, height);
    intensImg->resize(width, height);
    intensImgCopy->resize(width, height);
    prevIntensImg->resize(width, height);
    unXtnIntensImg->resize(this->width_orig,this->height_orig);    

    redPlane->resize(width, height);
    greenPlane->resize(width, height);
    bluePlane->resize(width, height);
    yellowPlane->resize(width, height);
    Yplane->resize(width, height);
    Uplane->resize(width, height);
    Vplane->resize(width, height);

    unXtnYplane->resize(width_orig, height_orig);
    unXtnUplane->resize(width_orig, height_orig);
    unXtnVplane->resize(width_orig, height_orig);

    YofYUVpy->resize(width_orig, height_orig);
    UofYUVpy->resize(width_orig, height_orig);
    VofYUVpy->resize(width_orig, height_orig);
    
    // Note, resizing 
    RplusUnex->resize(width_orig, height_orig);
    GplusUnex->resize(width_orig, height_orig);
    BplusUnex->resize(width_orig, height_orig);
    
    
    // allocating for CS ncsscale = 4;   

    cs_tot_32f     = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_32F, 1  );
    int_gradx_32f  = cvCreateImage( cvSize(width, height),IPL_DEPTH_32F, 1  );
    colcs_out      = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    ycs_out        = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    scs_out        = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    vcs_out        = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    
    /* zeroing the images*/
    outputImage->zero();
    flowImage->zero();
    
    /* initialising the optic flow computers */
    printf("initialising the opticflow computers \n");
    for(int row = 0; row < COUNTCOMPUTERSY; row ++) {
        for(int col = 0; col < COUNTCOMPUTERSX; col ++) {
            /*
            printf("\n initialising %d computer \n ",row * COUNTCOMPUTERSX + col );
            ofComputer[row * COUNTCOMPUTERSX + col] =
                new opticFlowComputer(row * COUNTCOMPUTERSX + col,
                                      12 * row + 1 + 6,
                                      12 * col + 6 + 5,
                                      5);
            ofComputer[row * COUNTCOMPUTERSX + col]->setName(getName("").c_str());
            ofComputer[row * COUNTCOMPUTERSX + col]->setHasStarted(false);
            if(row > 2) {
                ofComputer[row * COUNTCOMPUTERSX + col]->start();
            }
            */
        }
    }    
}

void plotterThread::setPortionU(int id, short* value) {
    /*
      int row = floor(id / COUNTCOMPUTERSX);
    int col = id  - row * COUNTCOMPUTERSX;
    int posY = 12 * row + (6 + 1) - halfCalc ;
    int posX = 12 * col + (6 + 5) - halfCalc ;
    represPointer = imageFlow ->getRawImage + posY * rowSize + posX * 3;
    for ( int x = 0; x < dimComput; x++) {
        *represenPointer = value; representPointer++;
        *representPointer = 
    }
    represenPointer += rowSize - dimComput * 3;
    */
    
    short* iter = computersValueU[id] ;
    /*
    for (int i = 0; i < dimComput * dimComput; i++) {
        //printf("copying the portion %d \n", *value);
        *iter++ = *value++;
    }
    */
    memcpy(iter, value, dimComput * dimComput * sizeof(short));


}

void plotterThread::setPortionV(int id, short* value) {
    short* iter = computersValueV[id] ;
    /*
    for (int i = 0; i < dimComput * dimComput; i++) {
        *iter++ = *value++;
    }
    */
    memcpy(iter, value, dimComput * dimComput * sizeof(short));
}

void plotterThread::initFlowComputer(int index) {
    printf("setting calculus %x pointer \n", intensImg->getRawImage());
    //ofComputer[index]->setCalculusPointer(gradientImgXCopy->getRawImage());
    //ofComputer[index]->setCalculusPointerY(gradientImgYCopy->getRawImage());
    /*
    ofComputer[index]->setCalculusPointer(intensImgCopy->getRawImage());
    ofComputer[index]->setCalculusRowSize(intensImgCopy->getRowSize());
    printf("setting representation pointer %x %d \n", outputImage->getRawImage(), intensImg->getRowSize());
    ofComputer[index]->setRepresenPointer(flowImage->getRawImage());
    ofComputer[index]->setRepresenImage(flowImage);
    printf("setting the image for temporal gradient \n");
    ofComputer[index]->setTemporalPointer(prevIntensImg->getRawImage());
    printf("setting semaphores \n");
    */
    //ofComputer[index]->setCalculusXSem(calcXSem[index]);
    //ofComputer[index]->setCalculusYSem(calcYSem[index]);
    /*
    ofComputer[index]->setCalculusSem(calcSem[index]);
    ofComputer[index]->setRepresentSem(reprSem[index]);
    ofComputer[index]->setTemporalSem(tempSem[index]);
    */
}


void plotterThread::filterInputImage() {    
    int i;
    const int szInImg = inputImage->getRawImageSize();
    unsigned char * pFilteredInpImg = filteredInputImage->getRawImage();
    unsigned char * pCurr = inputImage->getRawImage();
    int pad = inputImage->getPadding();
    float lambda = .5f;
    const float ul = 1.0f - lambda;
    for (i = 0; i < szInImg; i++) { // assuming same size
        *pFilteredInpImg = (unsigned char)(lambda * *pCurr++ + ul * *pFilteredInpImg++ + .5f);
    }
}


void plotterThread::extender(int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*extendedInputImage, *inputImage, maxSize);    
}

void plotterThread::extractPlanes() {

    //chromeThread->setFlagForDataReady(false);           
    // We extract color planes from the RGB image. Planes are red,blue, green and yellow (AM of red & green)
    uchar* shift[4];
    uchar* YUV[3];
    uchar* unXtnYUV[3];
    int padInput;
    int padUnX;
    int padUnXtnYUV;
    int padMono;
    uchar* tmpIntensityImage;
    uchar* ptrIntensityImg;
    uchar* ptrUnXtnIntensImg;
    uchar* inputPointer;    
    
    // Pointers to raw plane image
    shift[0] = (uchar*) redPlane->getRawImage(); 
    shift[1] = (uchar*) greenPlane->getRawImage(); 
    shift[2] = (uchar*) bluePlane->getRawImage(); 
    shift[3] = (uchar*) yellowPlane->getRawImage();

    YUV[0] = (uchar*) Yplane->getRawImage(); 
    YUV[1] = (uchar*) Uplane->getRawImage(); 
    YUV[2] = (uchar*) Vplane->getRawImage();

    unXtnYUV[0] = (uchar*) unXtnYplane->getRawImage(); 
    unXtnYUV[1] = (uchar*) unXtnUplane->getRawImage(); 
    unXtnYUV[2] = (uchar*) unXtnVplane->getRawImage(); 

 
    ptrIntensityImg   = (uchar*) intensImg->getRawImage();
    ptrUnXtnIntensImg = (uchar*) unXtnIntensImg->getRawImage();
    inputPointer      = (uchar*) extendedInputImage->getRawImage();
    padInput          = extendedInputImage->getPadding();
    padMono           = redPlane->getPadding();
    padUnX            = unXtnIntensImg->getPadding();
    padUnXtnYUV       = unXtnYplane->getPadding();
    

    const int h = extendedInputImage->height();
    const int w = extendedInputImage->width();

    
    for(int r = 0; r < h; r++) {       
        
        for(int c = 0; c < w; c++) {
            // assuming order of color channels is R,G,B. For some format it could be B,G,R.
            *shift[0] = *inputPointer++;
            *shift[1] = *inputPointer++;
            *shift[2] = *inputPointer++;

            *shift[3]++ = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));
            *ptrIntensityImg = ONE_BY_ROOT_THREE * sqrt((double) (*shift[0] * *shift[0] +*shift[1] * *shift[1] +*shift[2] * *shift[2]));
            

            // RGB to Y'UV conversion
            float red = (float)*shift[0];
            float green = (float)*shift[1];
            float blue = (float)*shift[2];

            int Y, U, V;
            Y = 0.299*red + 0.587*green + 0.114*blue;
            U = (blue-*YUV[0])*0.564 +128.0;
            V = (red-*YUV[0])*0.713 +128.0;
            
            *YUV[0] = max(16,min(235,Y));
            *YUV[1] = max(16,min(235,U));
            *YUV[2] = max(16,min(235,V));
            

            if(r>=maxKernelSize && c >=maxKernelSize && c< w-maxKernelSize){
                *ptrUnXtnIntensImg++ = *ptrIntensityImg;
                *unXtnYUV[0]++ = *YUV[0];
                *unXtnYUV[1]++ = *YUV[1];
                *unXtnYUV[2]++ = *YUV[2];                
            }

            ptrIntensityImg++;
            YUV[0]++;
            YUV[1]++;
            YUV[2]++;
            shift[0]++;
            shift[1]++;
            shift[2]++;
        }
        // paddings
        inputPointer += padInput;
        ptrIntensityImg += padMono;
        if(r>=maxKernelSize){            
            ptrUnXtnIntensImg += padUnX;
            unXtnYUV[0] += padUnXtnYUV;
            unXtnYUV[1] += padUnXtnYUV;
            unXtnYUV[2] += padUnXtnYUV;
        }
        shift[0] += padMono;
        shift[1] += padMono;
        shift[2] += padMono;
        shift[3] += padMono;
        YUV[0] += padMono;
        YUV[1] += padMono;
        YUV[2] += padMono;       
                
    } 

}

void plotterThread::filtering() {
    // We gaussian blur the image planes extracted before, one with positive Gaussian and then negative
    
    //Positive
    // This is calculated via first scale of YUV planes
    tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(redPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Rplus);
    

    tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(redPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Bplus);

    tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(greenPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Gplus);      
       
    
    //Negative
    tmpMonoLPImage->zero();
    gaborNegHorConvolution->convolve1D(redPlane,tmpMonoLPImage);
    gaborNegVerConvolution->convolve1D(tmpMonoLPImage,Rminus);

    tmpMonoLPImage->zero();
    gaborNegHorConvolution->convolve1D(bluePlane,tmpMonoLPImage);
    gaborNegVerConvolution->convolve1D(tmpMonoLPImage,Bminus);
    
    tmpMonoLPImage->zero();
    gaborNegHorConvolution->convolve1D(greenPlane,tmpMonoLPImage);
    gaborNegVerConvolution->convolve1D(tmpMonoLPImage,Gminus);

    tmpMonoLPImage->zero();
    gaborNegHorConvolution->convolve1D(yellowPlane,tmpMonoLPImage);
    gaborNegVerConvolution->convolve1D(tmpMonoLPImage,Yminus);    
    
}



void plotterThread::colorOpponency(){

    // get opponent colors for eg R+G- from R+ and G- channels
    // Finding color opponency now

    ImageOf<PixelMono>& coRG = colorOpp1Port.prepare();
    ImageOf<PixelMono>& coGR = colorOpp2Port.prepare();
    ImageOf<PixelMono>& coBY = colorOpp3Port.prepare();

    coRG.resize(width,height);
    coGR.resize(width,height);
    coBY.resize(width,height);
    
    
    uchar* pRG = coRG.getRawImage();
    uchar* pGR = coGR.getRawImage();
    uchar* pBY = coBY.getRawImage();

    uchar* rPlus  = Rplus->getRawImage();
    uchar* rMinus = Rminus->getRawImage();
    uchar* gPlus  = Gplus->getRawImage();
    uchar* gMinus = Gminus->getRawImage();
    uchar* bPlus  = Bplus->getRawImage();
    uchar* yMinus = Yminus->getRawImage();

    int padChannel   = Rplus->getPadding();
    int padOpponents = coRG.getPadding();

    for(int r = 0; r < height; r++) {
        for(int c = 0; c < width; c++) {
            
            *pRG++ = ((*rPlus >> 1) + 128 - (*gMinus >> 1) );
            *pGR++ = ((*gPlus >> 1) + 128 - (*rMinus >> 1) );
            *pBY++ = ((*bPlus >> 1) + 128 - (*yMinus >> 1) );

            rMinus++;
            rPlus++;
            gMinus++;
            gPlus++;
            yMinus++;
            bPlus++;
        }

        rMinus += padChannel;
        rPlus  += padChannel;
        gMinus += padChannel;
        gPlus  += padChannel;
        yMinus += padChannel;
        bPlus  += padChannel;
        pRG += padOpponents;
        pGR += padOpponents;
        pBY += padOpponents;

    }

    if(colorOpp1Port.getOutputCount()) {
        colorOpp1Port.write();
    }
    if(colorOpp2Port.getOutputCount()) {
        colorOpp2Port.write();
    }
    if(colorOpp3Port.getOutputCount()) {
        colorOpp3Port.write();
    }
    
#ifdef DEBUG_OPENCV
    cvNamedWindow("ColorOppRG");
    cvShowImage("ColorOppRG", (IplImage*)coRG.getIplImage());
    cvNamedWindow("ColorOppGR");
    cvShowImage("ColorOppGR", (IplImage*)coGR.getIplImage());
    cvNamedWindow("ColorOppBY");
    cvShowImage("ColorOppBY", (IplImage*)coBY.getIplImage());
    cvWaitKey(2);
#endif

}

void plotterThread::centerSurrounding(){        
        
        // Allocate temporarily
        ImageOf<PixelMono>& _Y = intensityCSPort.prepare();
        _Y.resize(this->width_orig,this->height_orig);

        ImageOf<PixelMono>& _UV = chromPort.prepare();
        _UV.resize(this->width_orig,this->height_orig);
        
        ImageOf<PixelMono>& _V = VofHSVPort.prepare();
        _V.resize(this->width_orig,this->height_orig);
        
        YofYUVpy->zero();
        VofYUVpy->zero();
        UofYUVpy->zero();
        Rplus->zero();
        Bplus->zero();
        Gplus->zero();
        
        
        //float YUV2RGBCoeff[9]={1, 0, 1.403,
                                //1, -.344, -.714,
                                //1, 1.77, 0
                                //};
                                //{-2.488,  3.489,  0.000,
                                 //3.596, -1.983, -0.498,
                                //-3.219,  1.061,  2.563};
                                     
}

void plotterThread::addFloatImage(IplImage* sourceImage, CvMat* cvMatAdded, double multFactor, double shiftFactor){

    IplImage stub, *toBeAddedImage;
    toBeAddedImage = cvGetImage(cvMatAdded, &stub);
    assert( sourceImage->width == toBeAddedImage->width && sourceImage->height == toBeAddedImage->height );
    float *ptrSrc, *ptrToBeAdded;
    ptrSrc = (float*)sourceImage->imageData;
    ptrToBeAdded = (float*)toBeAddedImage->imageData;
    int padImage = sourceImage->widthStep/sizeof(float) - sourceImage->width; //assuming monochromatic uchar image
    for(int i=0 ; i< sourceImage->height; ++i){
        for(int j=0; j< sourceImage->width; ++j){
            *ptrSrc = *ptrSrc + (multFactor* *ptrToBeAdded + shiftFactor); // in-place
            ptrSrc++;
            ptrToBeAdded++;
        }
        ptrSrc += padImage;
        ptrToBeAdded += padImage;
   }   
}


void plotterThread::threadRelease() {    
    printf("plotterThread: thread releasing \n");

    flowPort.interrupt();
    flowPort.close();

    resized = false;    

    // deallocating resources

    delete flowImage;
    /*
    delete inputImage;
    delete outputImage;
    delete finalOutputImage;
    delete filteredInputImage;
    delete extendedInputImage;
    delete Rplus;
    delete Rminus;
    delete Gplus;
    delete Gminus;
    delete Bplus;
    delete Bminus;
    delete Yminus;
    */

    printf("correctly deleting the images \n");
    /*
    delete gaborPosHorConvolution;    
    delete gaborPosVerConvolution;    
    delete gaborNegHorConvolution;    
    delete gaborNegVerConvolution;
    delete YofYUV;
    delete intensImg;
    //delete intensXGrad;
    //delete intensYGrad;
    delete intYgrad8u; 
    delete intYgrad8u;
    //delete gradientImgXCopy;
    //delete gradientImgXCopy;
    delete intensImgCopy;
    delete prevIntensImg;
    delete unXtnIntensImg;
    delete redPlane;
    delete greenPlane;
    delete bluePlane;
    delete yellowPlane;
    delete Yplane;
    delete Uplane;
    delete Vplane;
    delete unXtnYplane;
    delete unXtnUplane;
    delete unXtnVplane;
    delete YofYUVpy;
    delete UofYUVpy;
    delete VofYUVpy;
    delete RplusUnex;
    delete GplusUnex;
    delete BplusUnex;
    delete tmpMonoLPImage;

    delete[] reprSem;
    delete[] tempSem;
    delete[] calcSem;
    delete[] computersValueU;
    delete[] computersValueV;
    */

    //delete[] calcXSem;
    //delete[] calcYSem;

    printf("correctly freed memory of images \n");
    /*
    for(int j = 0 ; j < COUNTCOMPUTERSX * COUNTCOMPUTERSY; j++) {
        printf("stopping %d computer \n", j);        
        //ofComputer[j]->stop();
    } 
    */
 
    printf("Done with releasing earlyVision thread.\n");
    
}




