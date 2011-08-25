// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Shashank Pathak
 * email:   francesco.rea@iit.it, shashank.pathak@iit.it
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
 * @file edgesThread.cpp
 * @brief Implementation of the early stage of vision thread (see edgesThread.h).
 */

#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/edgesThread.h>

#include <cstring>

#define ONE_BY_ROOT_TWO 0.707106781
#define ONE_BY_ROOT_THREE 0.577350269
#define USE_PROPORTIONAL_KIRSCH
#define NO_DEBUG_OPENCV //DEBUG_OPENCV //

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

edgesThread::edgesThread():RateThread(RATE_OF_EDGES_THREAD) {   
    
    edgesThreadIsProcessing = false;
    dataReadyForEdgesThread = false;
    resized                 = false;    

    intensityImage      = new ImageOf<PixelMono>;
    tmpMonoSobelImage1  = new SobelOutputImage;
    tmpMonoSobelImage2  = new SobelOutputImage;
    //intensityImage      = NULL;

    sobel2DXConvolution = new convolve<ImageOf<PixelMono>,uchar,SobelOutputImage,SobelOutputImagePtr>(5,5,Sobel2DXgrad,SOBEL_FACTOR,SOBEL_SHIFT);
    sobel2DYConvolution = new convolve<ImageOf<PixelMono>,uchar,SobelOutputImage,SobelOutputImagePtr>(5,5,Sobel2DYgrad,SOBEL_FACTOR,SOBEL_SHIFT);

    sobelIsNormalized = 0;
    sobelLimits[0] = 0;
    sobelLimits[1] = 2.0;    
   
}

edgesThread::~edgesThread() {
    
    
    
    
}

bool edgesThread::threadInit() {
    printf("opening ports by edges thread \n");
    /* open ports */    
    
    if (!edges.open(getName("/edges:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    return true;
}

void edgesThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string edgesThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void edgesThread::run() {
    
    //printf("running edges thread \n");
    

        // skip if data is not ready yet
        //while(!getFlagForDataReady()) {};  
        if(getFlagForDataReady() && resized) {              

            // extract edges
            edgesExtract();  
            setFlagForDataReady(false);
            
        }       
        
    
}



void edgesThread::resize(int width_orig,int height_orig) { 
    
    this->widthLP = width_orig;
    this->heightLP = height_orig;

    intensityImage->resize(width_orig,height_orig);
    tmpMonoSobelImage1->resize(width_orig,height_orig);
    tmpMonoSobelImage2->resize(width_orig,height_orig);
    resized = true;   
}



void edgesThread::copyRelevantPlanes(ImageOf<PixelMono> *I){
    
    if(!getFlagForThreadProcessing() && I->getRawImage() != NULL ){         
        //printf("going to copy relevant planes in edges thread\n");
       setFlagForDataReady(false);
        // allocate
        if(!resized){
            resize(I->width(), I->height());
        }        

        // deep-copy
        memcpy( (uchar*)intensityImage->getRawImage(),(uchar*)I->getRawImage(), I->getRawImageSize());
        
    
        /*

        // CAUTION:shallow copy

        intensityImage = I;
        */      
        
        setFlagForDataReady(true);    
        
    }
         

}



void edgesThread::edgesExtract() {
    //printf("going to edge extract planes in edges thread\n");
    ImageOf<PixelMono>& edgesPortImage = edges.prepare();
    //ImageOf<PixelMono> edgesPortImage;
    edgesPortImage.resize(intensityImage->width(),intensityImage->height());

    setFlagForThreadProcessing(true);
    // X derivative 
    tmpMonoSobelImage1->zero();
    sobel2DXConvolution->convolve2D(intensityImage,tmpMonoSobelImage1);

    // Y derivative
    tmpMonoSobelImage2->zero();     // This can be removed 
    sobel2DYConvolution->convolve2D(intensityImage,tmpMonoSobelImage2);    

    setFlagForThreadProcessing(false);    

    //clearing up the previous value
    //edges->zero();

    uchar* pedges= (uchar*)edgesPortImage.getRawImage();
    SobelOutputImagePtr* ptrHorz = (SobelOutputImagePtr*)tmpMonoSobelImage1->getRawImage();
    SobelOutputImagePtr* ptrVert = (SobelOutputImagePtr*)tmpMonoSobelImage2->getRawImage();
     
    const int pad_edges = edgesPortImage.getPadding()/sizeof(uchar);
    int padHorz = tmpMonoSobelImage1->getPadding()/sizeof(SobelOutputImagePtr);
    int padVert = tmpMonoSobelImage2->getPadding()/sizeof(SobelOutputImagePtr);

    // Does not consider extended portion
    float normalizingRatio = 255.0/(sobelLimits[0]-sobelLimits[1]);
    for (int row = 0; row < edgesPortImage.height(); row++) {
        for (int col = 0; col < edgesPortImage.width(); col++) {

            
            double rg = sqrt((*ptrHorz ) * (*ptrHorz ) + (*ptrVert ) * (*ptrVert ))*0.707106781;
            
            if(sobelIsNormalized < SOBEL_FLICKER){
                sobelLimits[0] = sobelLimits[0]<rg?rg:sobelLimits[0];   // max
                sobelLimits[1] = sobelLimits[1]>rg?rg:sobelLimits[1];   //min
                *pedges = (uchar)(255*rg);
                
            }
            else {
                *pedges = (uchar)(normalizingRatio*(rg-sobelLimits[1]));
            }
            
            
            pedges++;
            ptrHorz++; ptrVert++;
            
        }
        // padding
        pedges += pad_edges;
        ptrHorz += padHorz;
        ptrVert += padVert;        
    } 

    sobelIsNormalized++;

#ifdef DEBUG_OPENCV
    cvNamedWindow("Edges");
    cvShowImage("Edges", (IplImage*)edgesPortImage.getIplImage());
    cvWaitKey(1);
    //cvDestroyWindow("Edges");
#endif
    //printf("Done with edges \n");
    edges.write();
    
}



void edgesThread::threadRelease() {
    
    printf("Releasing\n");

    //deallocating resources
    delete intensityImage;
    delete tmpMonoSobelImage1;
    delete tmpMonoSobelImage2;
    
    edges.interrupt();
    edges.close();

    printf("done with release\n");
    
}


