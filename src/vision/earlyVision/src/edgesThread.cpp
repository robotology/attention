// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Rea Francesco, Shashank Pathak
  * email:francesco.rea@iit.it, shashank.pathak@iit.it
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
 * @file edgesThread.cpp
 * @brief Implementation of the early stage of edges thread (see edgesThread.h).
 */

#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/edgesThread.h>

#ifdef WITH_CUDA
#include <iCub/cudaVision/cudaVision.h>
#endif

#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


edgesThread::edgesThread():PeriodicThread(RATE_OF_EDGES_THREAD) {
    
    edgesThreadIsProcessing = false;
    dataReadyForEdgesThread = false;
    resized                 = false;    

    intensityImage      = new ImageOf<PixelMono>;
    tmpMonoSobelImage1  = new SobelOutputImage;
    tmpMonoSobelImage2  = new SobelOutputImage;    

    sobel2DXConvolution = new convolve<ImageOf<PixelMono>,uchar,SobelOutputImage,SobelOutputImagePtr>(5,5,Sobel2DXgrad,SOBEL_FACTOR,SOBEL_SHIFT);
    sobel2DYConvolution = new convolve<ImageOf<PixelMono>,uchar,SobelOutputImage,SobelOutputImagePtr>(5,5,Sobel2DYgrad,SOBEL_FACTOR,SOBEL_SHIFT);

    sobelIsNormalized = 0;
    sobelLimits[0] = 0;
    sobelLimits[1] = 2.0;

    dImgIn   = 0;
    dImgOut  = 0;
    dImgBuff = 0;
   
}

edgesThread::~edgesThread() {
    
    printf("Edges thread object destroyed.\n");   
    
    
}

bool edgesThread::threadInit() {
    printf("opening ports by edges thread \n");
    /* open ports */    
    
    if (!edges.open(getName("/edges:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    
    
    if (!debugOutPort.open(getName("/debug:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }  

#ifdef WITH_CUDA
    setKernelF32Sep(SOBLE_ROW_H, 0, SOBEL_KERNSIZE);
    setKernelF32Sep(SOBLE_COL_H, 1, SOBEL_KERNSIZE);
#endif
    
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
    
    if(getFlagForDataReady() && resized) {              

        // extract edges
        edgesExtract();  
        setFlagForDataReady(false);
            
    }       
    
}

void edgesThread::resize(int w,int h) { 
    
    this->widthLP  = w;
    this->heightLP = h;
    this->widthUnXnt  = w - 2*maxKernelSize;
    this->heightUnXnt = h - maxKernelSize;

    intensityImage->resize(w,h);
    tmpMonoSobelImage1->resize(w,h);
    tmpMonoSobelImage2->resize(w,h);
    resized = true;   



}



void edgesThread::copyRelevantPlanes(ImageOf<PixelMono> *I){
    
    if(!getFlagForThreadProcessing() && I->getRawImage() != NULL ){         
        //printf("going to copy relevant planes in edges thread\n");
        setFlagForDataReady(false);
        // allocate
        if(!resized){
            resize(I->width(), I->height());        // I is the extended intensity image
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
    
    ImageOf<PixelMono>& edgesPortImage = edges.prepare();
    edgesPortImage.resize(widthUnXnt,heightUnXnt);
    setFlagForThreadProcessing(true);

#ifdef WITH_CUDA   
        // static float SOBLE_ROW[] =  {-1.62658,  -3.25315,  -0.00000,   3.25315,   1.62658 };
        // static float SOBLE_COL[] =  {-0.61479,  -2.45915,  -3.68873,  -2.45915,  -0.61479 };

        CvSize size = cvSize(intensityImage->width(), intensityImage->height());     
        IplImage* hImg = cvCreateImage(size, IPL_DEPTH_32F, 1);
        cvCvtScale((IplImage*)intensityImage->getIplImage(),hImg, 1.0/255.0); 
        /*
        
        //alocating memory on GPU if it's required
        // we allocate a big image for left and right together 
        
        if(!dImgIn) { 
            HANDLE_ERROR( cudaMalloc((void **) &dImgIn,   size.width * size.height * sizeof(float)) );
        }
        if(!dImgBuff) {
            HANDLE_ERROR( cudaMalloc((void **) &dImgBuff, size.width * size.height * sizeof(float)) );
        }
        if(!dImgOut) {
            HANDLE_ERROR( cudaMalloc((void **) &dImgOut,  size.width * size.height * sizeof(float)) );  
        }

        HANDLE_ERROR( cudaMemcpy(dImgIn, hImg->imageData, 
                                 size.width * size.height * sizeof(float), cudaMemcpyHostToDevice) );
        
        // 1D ROW and COl                          
        
        convF32Sep(dImgOut, dImgIn, 
                   size.width, size.height,
                   0, 1, SOBEL_KERNSIZE, dImgBuff);        
               
        HANDLE_ERROR( cudaMemcpy(hImg->imageData, dImgOut, 
                      size.width*size.height*sizeof(float), cudaMemcpyDeviceToHost) ); 
      
        */
        //sending image to a debug port
        if(debugOutPort.getOutputCount()) {
            ImageOf<PixelMono> &debugOutImage = debugOutPort.prepare(); 
            debugOutImage.resize(intensityImage->width(),intensityImage->height());
            //IplImage *cvDebugOutImage = (IplImage*)debugOutImage.getIplImage();
            cvCvtScale(hImg, debugOutImage.getIplImage(), 255);
            //cvCopy(hImg, debugOutImage.getIplImage());
            debugOutPort.write();
        }
             
     
        cvReleaseImage(&hImg);
#else
    // X derivative 
    tmpMonoSobelImage1->zero();

    sobel2DXConvolution->convolve2D(intensityImage,tmpMonoSobelImage1);
    
    // Y derivative
    tmpMonoSobelImage2->zero();     // This can be removed
    sobel2DYConvolution->convolve2D(intensityImage,tmpMonoSobelImage2);


#endif
    setFlagForThreadProcessing(false);    

    //clearing up the previous value
    edgesPortImage.zero();

    uchar* pedges= (uchar*)edgesPortImage.getRawImage();
    SobelOutputImagePtr* ptrHorz = (SobelOutputImagePtr*)tmpMonoSobelImage1->getRawImage();
    SobelOutputImagePtr* ptrVert = (SobelOutputImagePtr*)tmpMonoSobelImage2->getRawImage();
     
    const int pad_edges = edgesPortImage.getPadding()/sizeof(uchar);
    int padHorz = tmpMonoSobelImage1->getPadding()/sizeof(SobelOutputImagePtr) + 2*maxKernelSize;
    int padVert = tmpMonoSobelImage2->getPadding()/sizeof(SobelOutputImagePtr) + 2*maxKernelSize;

    // Does not consider extended portion
    ptrHorz += maxKernelSize*(tmpMonoSobelImage1->getRowSize()/sizeof(SobelOutputImagePtr)) + maxKernelSize;
    ptrVert += maxKernelSize*(tmpMonoSobelImage2->getRowSize()/sizeof(SobelOutputImagePtr)) + maxKernelSize;

    float normalizingRatio = 255.0/(sobelLimits[0]-sobelLimits[1]);
    for (int row = 0; row < this->heightUnXnt; row++) {
        for (int col = 0; col < this->widthUnXnt; col++) {



            double rg = sqrt((*ptrHorz ) * (*ptrHorz ) + (*ptrVert ) * (*ptrVert ))*0.707106781;
            
            if(sobelIsNormalized < SOBEL_FLICKER){
                sobelLimits[0] = sobelLimits[0]<rg?rg:sobelLimits[0];   // max
                sobelLimits[1] = sobelLimits[1]>rg?rg:sobelLimits[1];   //min
                *pedges = (uchar)(255*rg);
                
            }
            else {
                *pedges = (uchar)(normalizingRatio*(rg-sobelLimits[1]));
            }

            if(row < 5 || col < 5 || row > this->heightUnXnt - 5  || col > this->widthUnXnt - 5  ){
                *pedges = 0;
            }
            
            pedges++;
            ptrHorz++; ptrVert++;
            
        }
        // padding
        pedges += pad_edges;
        ptrHorz += padHorz ;
        ptrVert += padVert ;        
    }

    sobelIsNormalized++;

#ifdef DEBUG_OPENCV
    cvNamedWindow("Edges");
    cvShowImage("Edges", (IplImage*)edgesPortImage.getIplImage());
    cvWaitKey(2);
    //cvDestroyWindow("Edges");
#endif
    
    edges.write();
    
}

void edgesThread::threadRelease() {
    
    printf("Releasing edges thread ...\n");
    edges.interrupt();
    edges.close();
    debugOutPort.interrupt();
    debugOutPort.close();

    //deallocating resources
    delete intensityImage;
    delete tmpMonoSobelImage1;
    delete tmpMonoSobelImage2;  

#ifdef WITH_CUDA
    if(dImgIn) {
        HANDLE_ERROR( cudaFree(dImgIn) );
    }
    if(dImgBuff) {
        HANDLE_ERROR( cudaFree(dImgBuff) );
    }
    if(dImgOut) {
        HANDLE_ERROR( cudaFree(dImgOut) );
    }
#endif

    printf("Done with releasing edges thread\n");
    
}


