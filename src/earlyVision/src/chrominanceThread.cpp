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
 * @file chrominanceThread.cpp
 * @brief Implementation of the early stage of vision thread (see chrominanceThread.h).
 */

#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/chrominanceThread.h>

#include <cstring>
#include <sys/time.h>

#include <time.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

int powTailRecurse( int base, int power, int result){    
    if(power ==1)
        return result;
    else 
        return powTailRecurse(base,power-1,result*base);
}

int pow(int base, int power){
    assert( base!= 0 || power != 0); // sorry for 0 to power 0 !
    if(base == 1 || base == 0 )
        return base;            
    else if(power == 0)
        return 1;
    else
        return powTailRecurse(base,power,base);
}


chrominanceThread::chrominanceThread():RateThread(RATE_OF_CHROME_THREAD) {    
    
    chromeThreadProcessing      = false;
    dataReadyForChromeThread    = false;
    resized                     = false;
            
    chromUnXtnIntensImg = new ImageOf<PixelMono>;

    
    cartIntensImg       = new ImageOf<PixelMono>; 
    
    
    
    //Logpolar to cartesian and vice versa
    xSizeValue = CART_ROW_SIZE ;         
    ySizeValue = CART_COL_SIZE;          // y dimension of the remapped cartesian image
    overlap = 1.0;         // overlap in the remapping
    numberOfRings = COL_SIZE;      // 152, number of rings in the remapping
    numberOfAngles = ROW_SIZE;     // number of angles in the remapping   
    
    
    //gaborFiveByFive[0] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,Gab0,GABOR_SCALE_FACTOR,GABOR_SHIFT,GABOR_FLICKER);
    //gaborFiveByFive[1] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,Gab45,GABOR_SCALE_FACTOR,GABOR_SHIFT,GABOR_FLICKER);
    //gaborFiveByFive[2] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,Gab90,GABOR_SCALE_FACTOR,GABOR_SHIFT,GABOR_FLICKER);
    //gaborFiveByFive[3] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,GabM45,GABOR_SCALE_FACTOR,GABOR_SHIFT,GABOR_FLICKER);
    
    
    double* gabsDouble[4] = {Gab0D,Gab45D,Gab90D,GabM45D};
    float* gabsfloat[4] = {Gab0,Gab45,Gab90,GabM45};
    for(int i=0 ; i<GABOR_ORIS; ++i){
        gaborKernels[i] = cvCreateMat( 5,5, CV_64FC1 );
        *gaborKernels[i] = cvMat( 5, 5, CV_64FC1, gabsDouble[i] );
    }
            
    anchor = cv::Point(2,2);
    
    for(int i=0; i<GABOR_SCALES; ++i){
        imageForAScale[i] = new ImageOf<PixelFloat>;
        imageAtScale[i] = new ImageOf<PixelFloat>;
        gaussUpScaled[i] = new ImageOf<PixelFloat>;
        weightGaborAtScale[i] = 750;
        weightIntensityAtScale[i] = 510;
    }
    tempCSScaleOne = new ImageOf<PixelFloat>;
    imageInCartMono = new ImageOf<PixelMono>;
                
    
 
}

chrominanceThread::~chrominanceThread() {    
  printf("chrominance thread object destroyed. \n"); 
}

bool chrominanceThread::threadInit() {
    printf("opening ports by chrominance thread \n");
    /* open ports */    

    if (!orientPort0.open(getName("/orient0:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!orientPort45.open(getName("/orient45:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!orientPort90.open(getName("/orient90:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!orientPortM45.open(getName("/orientM45:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!totalOrientImagePort.open(getName("/sumOrientations:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!totalOrientCartImgPort.open(getName("/sumOrientationsInCart:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    //initializing logpolar mapping
    cout << "||| initializing the logpolar mapping" << endl;

    if (!lpMono.allocLookupTables(BOTH, numberOfRings, numberOfAngles, xSizeValue, ySizeValue, overlap)) {
        cerr << "can't allocate lookup tables for mono" << endl;
        return false;
    }
    cout << "|-| lookup table allocation for mono done" << endl;
    
    cvNamedWindow("Adjusting weights");
    cvResizeWindow("Adjusting weights",600,400);
    
    for(int i=0 ; i<GABOR_ORIS; ++i){
        char nameWtInten[20], nameWtGabor[20];
        sprintf(nameWtInten,"wtInten orient#:%d",i);
        sprintf(nameWtGabor,"wtGabor orient#:%d",i);
        cvCreateTrackbar(nameWtInten,"Adjusting weights", &weightIntensityAtScale[i], 1000, NULL); // no callback
        cvCreateTrackbar(nameWtGabor,"Adjusting weights", &weightGaborAtScale[i], 1000, NULL); // no callback
    }
   
    return true;
}

void chrominanceThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string chrominanceThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void chrominanceThread::run() {
    //if(totalOrientCartImgPort.getOutputCount()< 1 && totalOrientImagePort.getOutputCount()<1){
        // we are not interested in orientations, so do nothing
    //}
    //else{
        if(!resized){
            this->resize(ROW_SIZE,COL_SIZE);
        }
        else if(getFlagForDataReady()){                                   
            setFlagForThreadProcessing(true);
            //lpMono.logpolarToCart(*cartIntensImg,*chromUnXtnIntensImg);        
            orientation();        
            setFlagForThreadProcessing(false);
            setFlagForDataReady(false);
        }
    //}    
}

void chrominanceThread::resize(int width_orig,int height_orig) {  

    
    this->widthLP = width_orig;
    this->heightLP = height_orig;
    
    this->widthCr = CART_ROW_SIZE;
    this->heightCr = CART_COL_SIZE;

    chromUnXtnIntensImg->resize(width_orig,height_orig);
    cartIntensImg->resize(CART_ROW_SIZE, CART_COL_SIZE);
   
    
    for(int j=0; j<GABOR_SCALES; ++j){
        
        imageForAScale[j]->resize(ROW_SIZE,COL_SIZE);
        gaussUpScaled[j]->resize(ROW_SIZE,COL_SIZE);
        
        int factor = pow(2,j);
        int widthTemp = ROW_SIZE/factor;
        int heightTemp = COL_SIZE/factor;
        int cartWidthTemp = CART_ROW_SIZE/factor;
        int cartHeightTemp = CART_COL_SIZE/factor;
        
        //patch
        if(j==3){
            widthTemp += 1;
            heightTemp += 1;
        }
        
        imageAtScale[j]->resize(cartWidthTemp,cartHeightTemp);
       
    }
    
    resized = true;
    
}



void chrominanceThread::copyRelevantPlanes(ImageOf<PixelMono> *I){
    
    if(!getFlagForThreadProcessing() && I->getRawImage() != NULL  ){ 
        
        setFlagForDataReady(false);
        if(!resized){
            resize(I->width(), I->height()); 
        }
        int widthI = I->width();
        int heightI = I->height();
        
        
        // allocate
        chromUnXtnIntensImg->resize(I->width(), I->height());
        
        // copy
        memcpy( (uchar*)chromUnXtnIntensImg->getRawImage(),(uchar*)I->getRawImage(), I->getRawImageSize());        
        
        // set the flag to convey to other thread(s)        
        setFlagForDataReady(true);       
    }   

}

void chrominanceThread::copyScalesOfImages(ImageOf<PixelMono> *I, CvMat **toBeCopiedGauss){
    if(!getFlagForThreadProcessing() && resized){ 
        setFlagForDataReady(false);
        lpMono.logpolarToCart(*cartIntensImg,*I);
        IplImage* floatImage = cvCreateImage(cvSize(cartIntensImg->width(),cartIntensImg->height()),32,1);
        cvConvertScale((IplImage*)cartIntensImg->getIplImage(),floatImage,1.0/255.0,0.0);
        for(int i=0; i<GABOR_SCALES; ++i){
            cvCopy(toBeCopiedGauss[i],(IplImage*)gaussUpScaled[i]->getIplImage());
            cvResize(floatImage,(IplImage*)imageAtScale[i]->getIplImage());
            
        }
        cvReleaseImage(&floatImage);
        setFlagForDataReady(true); 
   } 

}

void chrominanceThread::orientation() {
 
        
    //----------------------------------------------------------------------------------------------------------//
    if(getFlagForDataReady()){
     //Checking!
                ImageOf<PixelFloat>  *temp2 = new ImageOf<PixelFloat>;                
                ImageOf<PixelFloat>* imageInCart = new ImageOf<PixelFloat>;
                                
                ImageOf<PixelMono>* imageInCartMonoLogP = new ImageOf<PixelMono>;
                
                imageInCart->resize(CART_ROW_SIZE,CART_COL_SIZE);
                imageInCartMono->resize(CART_ROW_SIZE,CART_COL_SIZE);
                imageInCartMonoLogP->resize(ROW_SIZE,COL_SIZE);
                imageInCart->zero();
                        
                //By now we have all scales of Y-image in pyramid for  0,45,90 and -45, scales are 4
                for(int eachOrient=0; eachOrient<GABOR_ORIS; ++eachOrient){                    
                               
                    tempCSScaleOne->resize(ROW_SIZE,COL_SIZE);
                    for(int eachScale=0; eachScale<GABOR_SCALES; ++eachScale){
                        int factor = pow(2,eachScale);
                        int widthTemp = CART_ROW_SIZE/factor;
                        int heightTemp = CART_COL_SIZE/factor;
                        /*
                        //patch
                        if(eachScale==3){
                            widthTemp += 1;
                            heightTemp += 1;
                        }*/
                        temp2->resize(widthTemp,heightTemp);
                        temp2->zero();
                            
                        cvFilter2D((IplImage*)imageAtScale[eachScale]->getIplImage(),(IplImage*)temp2->getIplImage(),gaborKernels[eachOrient],anchor);
                        imageInCart->zero();
                        imageInCartMono->zero();
                        cvResize((IplImage*)temp2->getIplImage(),(IplImage*)imageInCart->getIplImage(),CV_INTER_CUBIC);
                        cvConvertScale((IplImage*)imageInCart->getIplImage(),(IplImage*)imageInCartMono->getIplImage(),255*255,0);
                        lpMono.cartToLogpolar(*imageInCartMonoLogP,*imageInCartMono);
                        cvConvertScale((IplImage*)imageInCartMonoLogP->getIplImage(),(IplImage*)imageForAScale[eachScale]->getIplImage(),1.0/255.0,0.0);                            
                        
                    }
                        
                    // Combine the images for each orient
                    float* imgHdrTot = (float*)tempCSScaleOne->getRawImage();
                    float* imgPtrTot = (float*)tempCSScaleOne->getRawImage();
                    int rowSize = tempCSScaleOne->getRowSize()/sizeof(float);
                    tempCSScaleOne->zero();                      
                       
                    float *scaleImgPtr, *scaleImgHdr, *intensityImgPtr, *intensityImgHdr, *intensityImgPtrSec, *intensityImgHdrSec;
                    int scaleImgRowSize, intensityImgRowSize;
                     
                   for(int scaleNow = 0; scaleNow <GABOR_SCALES-1; ++scaleNow){
                        scaleImgPtr = (float*)imageForAScale[scaleNow]->getRawImage();
                        scaleImgHdr = (float*)imageForAScale[scaleNow]->getRawImage();
                        scaleImgRowSize = imageForAScale[scaleNow]->getRowSize()/sizeof(float);
                        intensityImgPtr = (float*)gaussUpScaled[scaleNow+1]->getRawImage();
                        intensityImgHdr = (float*)gaussUpScaled[scaleNow+1]->getRawImage();
                        intensityImgPtrSec = (float*)gaussUpScaled[scaleNow+2]->getRawImage();
                        intensityImgHdrSec = (float*)gaussUpScaled[scaleNow+2]->getRawImage();
                        intensityImgRowSize = gaussUpScaled[scaleNow+1]->getRowSize()/sizeof(float);
                        
                        for(int i=0; i<tempCSScaleOne->height(); ++i){
                            imgPtrTot = imgHdrTot+rowSize*i;
                            scaleImgPtr = scaleImgHdr + scaleImgRowSize*i;
                            intensityImgPtr = intensityImgHdr + intensityImgRowSize*i;                                
                            intensityImgPtrSec = intensityImgHdrSec + intensityImgRowSize*i;                                
                            for(int j=0; j<tempCSScaleOne->width(); ++j){
                                //weightGaborAtScale[scaleNow] = 1000.0;
                                //weightIntensityAtScale[scaleNow+1] = weightIntensityAtScale[scaleNow+2] =500.0;
                                *imgPtrTot += max(0.0,*scaleImgPtr*((float)weightGaborAtScale[scaleNow]-500.0)/1000.0 
                                             - (*intensityImgPtr*((float)weightIntensityAtScale[scaleNow+1]-500.0)/1000.0 
                                              + *intensityImgPtrSec*((float)weightIntensityAtScale[scaleNow+2]-500.0)/500.0 )/2.0);
                                imgPtrTot++;
                                scaleImgPtr++;
                                intensityImgPtr++;
                                intensityImgPtrSec++;
                            }
                        }
                   }
                   
                    scaleImgPtr = (float*)imageForAScale[GABOR_SCALES-2]->getRawImage();
                    scaleImgHdr = (float*)imageForAScale[GABOR_SCALES-2]->getRawImage();
                    scaleImgRowSize = imageForAScale[GABOR_SCALES-2]->getRowSize()/sizeof(float);
                    intensityImgPtr = (float*)gaussUpScaled[GABOR_SCALES-1]->getRawImage();
                    intensityImgHdr = (float*)gaussUpScaled[GABOR_SCALES-1]->getRawImage();                        
                    intensityImgRowSize = gaussUpScaled[GABOR_SCALES-1]->getRowSize()/sizeof(float);
                    
                    for(int i=0; i<tempCSScaleOne->height(); ++i){
                        imgPtrTot = imgHdrTot+rowSize*i;
                        scaleImgPtr = scaleImgHdr + scaleImgRowSize*i;
                        intensityImgPtr = intensityImgHdr + intensityImgRowSize*i;                                
                        for(int j=0; j<tempCSScaleOne->width(); ++j){
                            //weightGaborAtScale[GABOR_SCALES-2] = 1000.0;
                            //weightIntensityAtScale[GABOR_SCALES-1]  =500.0;
                            *imgPtrTot += max(0.0,*scaleImgPtr *((float)weightGaborAtScale[GABOR_SCALES-2]-500.0)/500.0 - *intensityImgPtr *((float)weightIntensityAtScale[GABOR_SCALES-1]-500.0)/500.0);
                            imgPtrTot++;
                            scaleImgPtr++;
                            intensityImgPtr++;                                
                        }
                    }
                       
                       
                   // we have now result for this orientation
                   imageInCartMono->zero();
                   imageInCartMonoLogP->zero();
                                      
                   cvConvertScale((IplImage*)tempCSScaleOne->getIplImage(),(IplImage*)imageInCartMonoLogP->getIplImage(),255,0);
                   lpMono.logpolarToCart(*imageInCartMono,*imageInCartMonoLogP);
                   cvWaitKey(2);
                   
                   if(eachOrient == 0){
                        orientPort0.prepare() = *imageInCartMono;
                        orientPort0.write();
                        }
                    if(eachOrient == 1){
                        orientPort45.prepare() = *imageInCartMono;
                        orientPort45.write();
                        }
                    if(eachOrient == 2){
                        orientPort90.prepare() = *imageInCartMono;
                        orientPort90.write();
                        }
                    if(eachOrient == 3){
                        orientPortM45.prepare() = *imageInCartMono;
                        orientPortM45.write();
                        }
                         
                       
                                
                }
                //cvWaitKey(0);
                delete temp2;
                delete imageInCart;
                delete imageInCartMonoLogP;
                
    }
    
    /*totalOrientImagePort.write();
    
    if(totalOrientCartImgPort.getOutputCount()){
        totalOrientCartImgPort.prepare() = *oriAll;
        totalOrientCartImgPort.write();
    } */   
  
}


void chrominanceThread::threadRelease() {    

    printf("Releasing chrominance thread .....\n");

    lpMono.freeLookupTables();
    
    orientPort0.interrupt();
    orientPort45.interrupt();
    orientPort90.interrupt();
    orientPortM45.interrupt();        
    
    orientPort0.close();
    orientPort45.close();
    orientPort90.close();
    orientPortM45.close();
    
    totalOrientCartImgPort.interrupt();
    totalOrientCartImgPort.close();

    //deallocating resources
    delete chromUnXtnIntensImg;
    delete cartIntensImg;   
    for(int i=0; i<GABOR_SCALES; ++i){
        delete imageForAScale[i];
        delete imageAtScale[i];
        delete gaussUpScaled[i];        
    }
    
    printf("Done with releasing chrominance thread.\n");
    delete tempCSScaleOne;
    delete imageInCartMono;  
    
    
    
}


