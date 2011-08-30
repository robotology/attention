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




chrominanceThread::chrominanceThread():RateThread(RATE_OF_CHROME_THREAD) {    
    
    chromeThreadProcessing      = false;
    dataReadyForChromeThread    = false;
    resized                     = false;
        
    chromUnXtnIntensImg = new ImageOf<PixelMono>;

    o0      = new KirschOutputImage;
    o45     = new KirschOutputImage;
    o90     = new KirschOutputImage;
    oM45    = new KirschOutputImage;
    
    tmpKirschCartImage1  = new KirschOutputImage;
    tmpKirschCartImage2  = new KirschOutputImage;
    tmpKirschCartImage3  = new KirschOutputImage;
    tmpKirschCartImage4  = new KirschOutputImage;
    totalKirsch          = new KirschOutputImage; 
    cartIntensImg       = new ImageOf<PixelMono>; 
    logPolarOrientImg   = new ImageOf<PixelMono>;
    
    ori0                = new ImageOf<PixelMono>;
    ori45               = new ImageOf<PixelMono>;
    ori90               = new ImageOf<PixelMono>;
    oriM45              = new ImageOf<PixelMono>;
    oriAll              = new ImageOf<PixelMono>;

    kirschSalPos0 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_POS_KERNEL,KIRSCH_POS_KERNEL,r1,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalPos45 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_POS_KERNEL,KIRSCH_POS_KERNEL,r2,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalPos90 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_POS_KERNEL,KIRSCH_POS_KERNEL,r3,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalPosM45 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_POS_KERNEL,KIRSCH_POS_KERNEL,r4,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);

    kirschSalNeg0 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_NEG_KERNEL,KIRSCH_NEG_KERNEL,rn1,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalNeg45 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_NEG_KERNEL,KIRSCH_NEG_KERNEL,rn2,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalNeg90 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_NEG_KERNEL,KIRSCH_NEG_KERNEL,rn3,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalNegM45 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_NEG_KERNEL,KIRSCH_NEG_KERNEL,rn4,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    //kirschListOfNegKernels = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_NEG_KERNEL,KIRSCH_NEG_KERNEL,listOfNeg,NBR_KIRSCH_NEG_KERNELS,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    

    kirschIsNormalized = 0;
    kirschLimits[0][0] = 0;
    kirschLimits[0][1] = 2.0;
    kirschLimits[1][0] = 0;
    kirschLimits[1][1] = 2.0;
    kirschLimits[2][0] = 0;
    kirschLimits[2][1] = 2.0;
    kirschLimits[3][0] = 0;
    kirschLimits[3][1] = 2.0;

    for(int i=0; i<4; ++i) {
        wtForEachOrientation[i]= 1/4.0; // equal weights by default
    }    

    //Logpolar to cartesian and vice versa
    xSizeValue = CART_ROW_SIZE ;         
    ySizeValue = CART_COL_SIZE;          // y dimension of the remapped cartesian image
    overlap = 1.0;         // overlap in the remapping
    numberOfRings = COL_SIZE;      // 152, number of rings in the remapping
    numberOfAngles = ROW_SIZE;     // number of angles in the remapping   
    
 /*   //Logpolar to cartesian and vice versa
    xSizeValue = 320 ;         
    ySizeValue = 240;          // y dimension of the remapped cartesian image
    overlap = 1.0;         // overlap in the remapping
    numberOfRings = 152;      // number of rings in the remapping
    numberOfAngles = 252;     // number of angles in the remapping
*/
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

    //initializing logpolar mapping
    cout << "||| initializing the logpolar mapping" << endl;

    if (!lpMono.allocLookupTables(BOTH, numberOfRings, numberOfAngles, xSizeValue, ySizeValue, overlap)) {
        cerr << "can't allocate lookup tables for mono" << endl;
        return false;
    }
    cout << "|-| lookup table allocation for mono done" << endl;
   
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
    if(getFlagForDataReady() && resized){
                               
        setFlagForThreadProcessing(true);
        lpMono.logpolarToCart(*cartIntensImg,*chromUnXtnIntensImg);        
        orientation();        
        setFlagForThreadProcessing(false);
        setFlagForDataReady(false);

    }    
}

void chrominanceThread::resize(int width_orig,int height_orig) {  

    
    this->widthLP = width_orig;
    this->heightLP = height_orig;
    
    this->widthCr = CART_ROW_SIZE;
    this->heightCr = CART_COL_SIZE;

    chromUnXtnIntensImg->resize(width_orig,height_orig);
    cartIntensImg->resize(CART_ROW_SIZE, CART_COL_SIZE);

    // for Kirsch
    // float images
    o0->resize(CART_ROW_SIZE, CART_COL_SIZE);
    o45->resize(CART_ROW_SIZE, CART_COL_SIZE);
    o90->resize(CART_ROW_SIZE, CART_COL_SIZE);
    oM45->resize(CART_ROW_SIZE, CART_COL_SIZE);
    // uchar images
    ori0->resize(CART_ROW_SIZE, CART_COL_SIZE);
    ori45->resize(CART_ROW_SIZE, CART_COL_SIZE);
    ori90->resize(CART_ROW_SIZE, CART_COL_SIZE);
    oriM45->resize(CART_ROW_SIZE, CART_COL_SIZE);
    oriAll->resize(CART_ROW_SIZE, CART_COL_SIZE);    
    logPolarOrientImg->resize(ROW_SIZE,COL_SIZE);

    tmpKirschCartImage1->resize(CART_ROW_SIZE, CART_COL_SIZE);
    tmpKirschCartImage2->resize(CART_ROW_SIZE, CART_COL_SIZE);
    tmpKirschCartImage3->resize(CART_ROW_SIZE, CART_COL_SIZE);
    tmpKirschCartImage4->resize(CART_ROW_SIZE, CART_COL_SIZE);
    totalKirsch->resize(CART_ROW_SIZE, CART_COL_SIZE);
    /*listOfNegKir[0]->resize(CART_ROW_SIZE, CART_COL_SIZE);
    listOfNegKir[1]->resize(CART_ROW_SIZE, CART_COL_SIZE);
    listOfNegKir[2]->resize(CART_ROW_SIZE, CART_COL_SIZE);
    listOfNegKir[3]->resize(CART_ROW_SIZE, CART_COL_SIZE); */
    
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
        
        /*

        // CAUTION:shallow copy

        chromUnXtnIntensImg = I;
        chromYplane = Y;
        chromUplane = U;
        chromVplane = V; 
        */         
        setFlagForDataReady(true);       
    }   

}

void chrominanceThread::orientation() {
    //printf("start orientation \n");
    int cartesWidth  = cartIntensImg->width();
    int cartesHeight = cartIntensImg->height();

    // orientation port
    ImageOf<PixelMono>& oPort0   = orientPort0.prepare();    
    ImageOf<PixelMono>& oPort45  = orientPort45.prepare();    
    ImageOf<PixelMono>& oPort90  = orientPort90.prepare();    
    ImageOf<PixelMono>& oPortM45 = orientPortM45.prepare();
    ImageOf<PixelMono>& totImg = totalOrientImagePort.prepare();
    
    // resize images to logpolar dimension
    oPort0.resize(ROW_SIZE,COL_SIZE);
    oPort45.resize(ROW_SIZE,COL_SIZE);
    oPort90.resize(ROW_SIZE,COL_SIZE);
    oPortM45.resize(ROW_SIZE,COL_SIZE);
    totImg.resize(ROW_SIZE,COL_SIZE);
    

    // using Kirsch cum positive Gaussian matrix
    /*
    kirschSalPos0->convolve2D(cartIntensImg,o0);
    kirschSalPos45->convolve2D(cartIntensImg,o45);
    kirschSalPos90->convolve2D(cartIntensImg,o90);
    kirschSalPos45->convolve2D(cartIntensImg,oM45);   
    

    // using Kirsch cum Negative gaussian matrix
    kirschSalNeg0->convolve2D(cartIntensImg,tmpKirschCartImage1); 
    kirschSalNeg45->convolve2D(cartIntensImg,tmpKirschCartImage2); 
    kirschSalNeg90->convolve2D(cartIntensImg,tmpKirschCartImage3); 
    kirschSalNegM45->convolve2D(cartIntensImg,tmpKirschCartImage4);
    */

    kirschSalPos0->convolve2DRegion(cartIntensImg,o0,0,cartIntensImg->height()/2,
                                    cartIntensImg->width()/2,cartIntensImg->width()/2,
                                    cartIntensImg->height()/2);
    kirschSalPos45->convolve2DRegion(cartIntensImg,o45,0,cartIntensImg->height()/2,
                                    cartIntensImg->width()/2,cartIntensImg->width()/2,
                                    cartIntensImg->height()/2);
    kirschSalPos90->convolve2DRegion(cartIntensImg,o90,0,cartIntensImg->height()/2,
                                    cartIntensImg->width()/2,cartIntensImg->width()/2,
                                    cartIntensImg->height()/2);
    kirschSalPosM45->convolve2DRegion(cartIntensImg,oM45,0,cartIntensImg->height()/2,
                                    cartIntensImg->width()/2,cartIntensImg->width()/2,
                                    cartIntensImg->height()/2);

    kirschSalNeg0->convolve2DRegion(cartIntensImg,tmpKirschCartImage1,0,cartIntensImg->height()/2,
                                    cartIntensImg->width()/2,cartIntensImg->width()/2,
                                    cartIntensImg->height()/2);
    kirschSalNeg45->convolve2DRegion(cartIntensImg,tmpKirschCartImage2,0,cartIntensImg->height()/2,
                                    cartIntensImg->width()/2,cartIntensImg->width()/2,
                                    cartIntensImg->height()/2); 
    kirschSalNeg90->convolve2DRegion(cartIntensImg,tmpKirschCartImage3,0,cartIntensImg->height()/2,
                                    cartIntensImg->width()/2,cartIntensImg->width()/2,
                                    cartIntensImg->height()/2); 
    kirschSalNegM45->convolve2DRegion(cartIntensImg,tmpKirschCartImage4,0,cartIntensImg->height()/2,
                                    cartIntensImg->width()/2,cartIntensImg->width()/2,
                                    cartIntensImg->height()/2);
    

/*
    kirschListOfNegKernels->convolve2Dlist(cartIntensImg,listOfNegKir);
*/  

    uchar* ori[4]= {(uchar*)ori0->getRawImage(),
                    (uchar*)ori45->getRawImage(),
                    (uchar*)ori90->getRawImage(),
                    (uchar*)oriM45->getRawImage()};

    float *ptrTotKir = (float*)totalKirsch->getRawImage();

    KirschOutputImagePtr* p[4] = { (KirschOutputImagePtr*)o0->getRawImage(),
                                    (KirschOutputImagePtr*)o45->getRawImage(),
                                    (KirschOutputImagePtr*)o90->getRawImage(),
                                    (KirschOutputImagePtr*)oM45->getRawImage()};

    KirschOutputImagePtr* n[4] = { (KirschOutputImagePtr*)tmpKirschCartImage1->getRawImage(),
                                    (KirschOutputImagePtr*)tmpKirschCartImage2->getRawImage(),
                                    (KirschOutputImagePtr*)tmpKirschCartImage3->getRawImage(),
                                    (KirschOutputImagePtr*)tmpKirschCartImage4->getRawImage()};

     
    const int pad_output = ori0->getPadding() / sizeof(uchar);
    int padK             = o0->getPadding()  / sizeof(KirschOutputImagePtr);     

    
#ifdef USE_PROPORTIONAL_KIRSCH
    float normalizingRatio[4];
    normalizingRatio[0] = 255.0 / (kirschLimits[0][0] - kirschLimits[0][1]);
    normalizingRatio[1] = 255.0 / (kirschLimits[1][0] - kirschLimits[1][1]);
    normalizingRatio[2] = 255.0 / (kirschLimits[2][0] - kirschLimits[2][1]);
    normalizingRatio[3] = 255.0 / (kirschLimits[3][0] - kirschLimits[3][1]);
#endif

    float multiplier = 1;
    float multiplierForNeg = .33;
    float normalizingFactor = 1.0/(wtForEachOrientation[0]+wtForEachOrientation[1]
                                + wtForEachOrientation[2]+wtForEachOrientation[3]);
    normalizingFactor = min((float)1000.0,max((float).001,normalizingFactor));      //against nan
    // normalized weights
    float w0 = wtForEachOrientation[0]*normalizingFactor;
    float w1 = wtForEachOrientation[1]*normalizingFactor;
    float w2 = wtForEachOrientation[2]*normalizingFactor;
    float w3 = wtForEachOrientation[3]*normalizingFactor;
    for (int row = 0; row < ori0->height(); row++) {
        for (int col = 0; col < ori0->width(); col++) {

#ifdef USE_PROPORTIONAL_KIRSCH //-------------------------------------
            //if() {
                //printf("second proportional kirsch \n");
                //if (row < CART_ROW_SIZE) {
                if(kirschIsNormalized < KIRSCH_FLICKER){

                        float tmpV0 = (abs(*p[0])-.33*(abs(*n[1])+abs(*n[2])+abs(*n[3]))) *255 ;
                        float tmpV1 = (abs(*p[1])-.33*(abs(*n[0])+abs(*n[2])+abs(*n[3]))) *255 ;
                        float tmpV2 = (abs(*p[2])-.33*(abs(*n[1])+abs(*n[0])+abs(*n[3]))) *255 ;
                        float tmpV3 = (abs(*p[3])-.33*(abs(*n[1])+abs(*n[2])+abs(*n[0]))) *255 ;
                    
                        kirschLimits[0][0] = kirschLimits[0][0] < tmpV0 ? tmpV0 : kirschLimits[0][0]; // max
                        kirschLimits[0][1] = kirschLimits[0][1] > tmpV0 ? tmpV0 : kirschLimits[0][1]; // min
                        kirschLimits[1][0] = kirschLimits[1][0] < tmpV1 ? tmpV1 : kirschLimits[1][0]; // max
                        kirschLimits[1][1] = kirschLimits[1][1] > tmpV1 ? tmpV1 : kirschLimits[1][1]; // min

                        kirschLimits[2][0] = kirschLimits[2][0] < tmpV2 ? tmpV2 : kirschLimits[2][0]; // max
                        kirschLimits[2][1] = kirschLimits[2][1] > tmpV2 ? tmpV2 : kirschLimits[2][1]; // min
                        kirschLimits[3][0] = kirschLimits[3][0] < tmpV3 ? tmpV3 : kirschLimits[3][0]; // max
                        kirschLimits[3][1] = kirschLimits[3][1] > tmpV3 ? tmpV3 : kirschLimits[3][1]; // min

                        *ori[0] = tmpV0;
                        *ori[1] = tmpV1 ;
                        *ori[2] = tmpV2 ;
                        *ori[3] = tmpV3 ;
                        //*ptrTotKir = max(max(tmpV0,tmpV1),max(tmpV2,tmpV3)); 
                        *ptrTotKir =    w0*tmpV0 +
                                        w1*tmpV1 +
                                        w2*tmpV2 +
                                        w3*tmpV3 ;                     
                    
                }
                else {
                   
                        //float tmpV = 255.0 *abs(*p[i]);
                        float tmpV0 = normalizingRatio[0]*(*p[0] - kirschLimits[0][1]);
                        float tmpV1 = normalizingRatio[1]*(*p[1] - kirschLimits[1][1]);
                        float tmpV2 = normalizingRatio[2]*(*p[2] - kirschLimits[2][1]);
                        float tmpV3 = normalizingRatio[3]*(*p[3] - kirschLimits[3][1]);
                        *ori[0] = tmpV0>255?255:tmpV0<0?0:(unsigned char)tmpV0;
                        *ori[1] = tmpV1>255?255:tmpV1<0?0:(unsigned char)tmpV1;
                        *ori[2] = tmpV2>255?255:tmpV2<0?0:(unsigned char)tmpV2;
                        *ori[3] = tmpV3>255?255:tmpV3<0?0:(unsigned char)tmpV3;
                        //*ptrTotKir = max(max(tmpV0,tmpV1),max(tmpV2,tmpV3));
                        *ptrTotKir =    w0*tmpV0 +
                                        w1*tmpV1 +
                                        w2*tmpV2 +
                                        w3*tmpV3 ;  
                        
                }
                
            
#else //--------------------------------------------------------------

                float tmpV0 = (abs(*p[0])-multiplierForNeg*(abs(*n[1])+abs(*n[2])+abs(*n[3]))) *multiplier ;
                float tmpV1 = (abs(*p[1])-multiplierForNeg*(abs(*n[0])+abs(*n[2])+abs(*n[3]))) *multiplier ;
                float tmpV2 = (abs(*p[2])-multiplierForNeg*(abs(*n[0])+abs(*n[1])+abs(*n[3]))) *multiplier ;
                float tmpV3 = (abs(*p[3])-multiplierForNeg*(abs(*n[1])+abs(*n[2])+abs(*n[0]))) *multiplier ;
                float l = 0;
                float u = 255;
                *ori[0] = max(l,min(u,255*tmpV0));
                *ori[1] = max(l,min(u,255*tmpV1));
                *ori[2] = max(l,min(u,255*tmpV2));
                *ori[3] = max(l,min(u,255*tmpV3));
                //*ptrTotKir = max(max(tmpV0,tmpV1),max(tmpV2,tmpV3));
                *ptrTotKir =    w0*tmpV0 +
                                        w1*tmpV1 +
                                        w2*tmpV2 +
                                        w3*tmpV3 ; 
             
#endif //---------------------------------------------------------------
            ori[0]++;
            p[0]++;
            n[0]++;
            ori[1]++;
            p[1]++;
            n[1]++;
            ori[2]++;
            p[2]++;
            n[2]++;
            ori[3]++;
            p[3]++;
            n[3]++;
            ptrTotKir++;             
            
        }
        // padding
        ori[0] += pad_output;
        p[0] += padK; 
        n[0] += padK; 
        ori[1] += pad_output;
        p[1] += padK; 
        n[1] += padK; 
        ori[2] += pad_output;
        p[1] += padK; 
        n[2] += padK; 
        ori[3] += pad_output;
        p[3] += padK; 
        n[3] += padK;
        ptrTotKir += padK; 
               
    } 

    kirschIsNormalized++;

    cvConvertScale((IplImage*)totalKirsch->getIplImage(),(IplImage*)oriAll->getIplImage(),255,0);

    // Converting cartesian images back to logpolar
    lpMono.cartToLogpolar(totImg,*oriAll);
    lpMono.cartToLogpolar(oPort0,*ori0);
    lpMono.cartToLogpolar(oPort45,*ori45);
    lpMono.cartToLogpolar(oPort90,*ori90);
    lpMono.cartToLogpolar(oPortM45,*oriM45);    
    
#ifdef DEBUG_OPENCV
    cvNamedWindow("Original");
    cvShowImage("Original",(IplImage*)totImg.getIplImage());
    cvNamedWindow("Sum");
    cvShowImage("Sum",  (IplImage*)oPort0.getIplImage());    
    cvWaitKey(2);
#endif
     
    orientPort0.write();
    orientPort45.write();
    orientPort90.write();
    orientPortM45.write();
    totalOrientImagePort.write();    
  
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

    //deallocating resources
    delete chromUnXtnIntensImg;

    delete o0;
    delete o45;
    delete o90;
    delete oM45;

    delete tmpKirschCartImage1;    
    delete tmpKirschCartImage2;    
    delete tmpKirschCartImage3;    
    delete tmpKirschCartImage4;
    delete totalKirsch;

    delete ori0;
    delete ori45;
    delete ori90;
    delete oriM45;
    delete oriAll;

    delete cartIntensImg;
    delete logPolarOrientImg;

    delete kirschSalPos0;
    delete kirschSalPos45;
    delete kirschSalPos90;
    delete kirschSalPosM45;
    delete kirschSalNeg0;
    delete kirschSalNeg45;
    delete kirschSalNeg90;
    delete kirschSalNegM45;    
    
    printf("Done with releasing chrominance thread.\n");
    
}


