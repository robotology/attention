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

#define ONE_BY_ROOT_TWO 0.707106781
#define ONE_BY_ROOT_THREE 0.577350269
#define USE_PROPORTIONAL_KIRSCH
#define NO_DEBUG_OPENCV //DEBUG_OPENCV //

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;




chrominanceThread::chrominanceThread():RateThread(RATE_OF_CHROME_THREAD) {
    
    
    
    chromeThreadProcessing      = false;
    dataReadyForChromeThread    = false;
    resized                     = false;
    //*(this->dataReadyForChromeThread) = false;

    chromYplane = new ImageOf<PixelMono>;
    chromUplane = new ImageOf<PixelMono>;
    chromVplane = new ImageOf<PixelMono>;
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
    
    

    kirschSalPos0 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_POS_KERNEL,KIRSCH_POS_KERNEL,r1,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalPos45 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_POS_KERNEL,KIRSCH_POS_KERNEL,r2,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalPos90 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_POS_KERNEL,KIRSCH_POS_KERNEL,r3,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalPosM45 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_POS_KERNEL,KIRSCH_POS_KERNEL,r4,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);

    kirschSalNeg0 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_NEG_KERNEL,KIRSCH_NEG_KERNEL,rn1,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalNeg45 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_NEG_KERNEL,KIRSCH_NEG_KERNEL,rn2,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalNeg90 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_NEG_KERNEL,KIRSCH_NEG_KERNEL,rn3,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschSalNegM45 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(KIRSCH_NEG_KERNEL,KIRSCH_NEG_KERNEL,rn4,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);

    
    

    kirschIsNormalized = 0;
    kirschLimits[0][0] = 0;
    kirschLimits[0][1] = 2.0;
    kirschLimits[1][0] = 0;
    kirschLimits[1][1] = 2.0;
    kirschLimits[2][0] = 0;
    kirschLimits[2][1] = 2.0;
    kirschLimits[3][0] = 0;
    kirschLimits[3][1] = 2.0;
    

    img_Y = new ImageOf<PixelMono>;
	img_UV = new ImageOf<PixelMono>;
	img_V = new ImageOf<PixelMono>;
    isYUV = true;
	
	
    
    
    
    
    

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
    
    

    
    
}

bool chrominanceThread::threadInit() {
    printf("opening ports by chrominance thread \n");
    /* open ports */ 
   
   
    if(isYUV){
        
        
        if (!chromPort.open(getName("/chrominance:o").c_str())) {
            cout << ": unable to open port "  << endl;
            return false;  // unable to open; let RFModule know so that it won't run
        }
    }

    else{
        
        if (!chromPort.open(getName("/S:o").c_str())) {
            cout << ": unable to open port "  << endl;
            return false;  // unable to open; let RFModule know so that it won't run
        }
    
    }
    
    if (!intensityCSPort.open(getName("/centSurrIntensity:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

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
    
    //printf("running chrome thread \n");
    

        // wait if data is not ready yet  
        //while(!getFlagForDataReady()) {};
        
          if(getFlagForDataReady() && resized){  
                               
                setFlagForThreadProcessing(true);
                lpMono.logpolarToCart(*cartIntensImg,*chromUnXtnIntensImg);

                // Center-surround
                centerSurrounding();

                //printf("before colour opponency \n");
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
    o0->resize(CART_ROW_SIZE, CART_COL_SIZE);
    o45->resize(CART_ROW_SIZE, CART_COL_SIZE);
    o90->resize(CART_ROW_SIZE, CART_COL_SIZE);
    oM45->resize(CART_ROW_SIZE, CART_COL_SIZE);
    tmpKirschCartImage1->resize(CART_ROW_SIZE, CART_COL_SIZE);
    tmpKirschCartImage2->resize(CART_ROW_SIZE, CART_COL_SIZE);
    tmpKirschCartImage3->resize(CART_ROW_SIZE, CART_COL_SIZE);
    tmpKirschCartImage4->resize(CART_ROW_SIZE, CART_COL_SIZE);
    totalKirsch->resize(CART_ROW_SIZE, CART_COL_SIZE);
    

    // allocating for CS ncsscale = 4;   

    cs_tot_32f  = cvCreateImage( cvSize(widthLP, heightLP),IPL_DEPTH_32F, 1  );
    colcs_out   = cvCreateImage( cvSize(widthLP, heightLP),IPL_DEPTH_8U, 1  );
    ycs_out     = cvCreateImage( cvSize(widthLP, heightLP),IPL_DEPTH_8U, 1  );
    scs_out     = cvCreateImage( cvSize(widthLP, heightLP),IPL_DEPTH_8U, 1  );
    vcs_out     = cvCreateImage( cvSize(widthLP, heightLP),IPL_DEPTH_8U, 1  );
    
    
    centerSurr  = new CenterSurround( widthLP,heightLP,1.0 );

    
    isYUV = true;
	img_Y->resize( this->widthLP, this->heightLP );
    img_UV->resize( this->widthLP, this->heightLP );
    img_V->resize( this->widthLP, this->heightLP );
    resized = true;
   
    
}



void chrominanceThread::copyRelevantPlanes(ImageOf<PixelMono> *I,ImageOf<PixelMono> *Y, ImageOf<PixelMono> *U,ImageOf<PixelMono> *V){
    
    if(!getFlagForThreadProcessing() && I->getRawImage() != NULL && U->getRawImage() != NULL  && V->getRawImage() != NULL  && Y->getRawImage() != NULL ){ 
        //printf("Going to copy relevant planes in chrome thread\n");
        setFlagForDataReady(false);
        if(!resized){
            resize(I->width(), I->height()); 
        }
        int widthI = I->width();
        int widthY = Y->width();
        int widthU = U->width();
        int widthV = V->width();
        int heightI = I->height();
        int heightY = Y->height();
        int heightU = U->height();
        int heightV = V->height();

        
        
        // allocate
        chromUnXtnIntensImg->resize(I->width(), I->height());
        chromYplane->resize(Y->width(), Y->height());
        chromUplane->resize(U->width(), U->height());
        chromVplane->resize(V->width(), V->height()); 

        // copy
        memcpy( (uchar*)chromUnXtnIntensImg->getRawImage(),(uchar*)I->getRawImage(), I->getRawImageSize());
        memcpy( (uchar*)chromYplane->getRawImage(),(uchar*)Y->getRawImage(),Y->getRawImageSize());
        memcpy( (uchar*)chromUplane->getRawImage(),(uchar*)U->getRawImage(),U->getRawImageSize());
        memcpy( (uchar*)chromVplane->getRawImage(),(uchar*)V->getRawImage(),V->getRawImageSize());
    
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



void chrominanceThread::centerSurrounding(){

        //printf("Going to centerSurrounding planes in chrome thread\n");
        
        // Allocate temporarily
        //ImageOf<PixelMono> _Y,_UV,_V;
        ImageOf<PixelMono>& _Y = intensityCSPort.prepare();
        _Y.resize(this->widthLP,this->heightLP);
        ImageOf<PixelMono>& _UV = chromPort.prepare();
        _UV.resize(this->widthLP,this->heightLP);
        
        ImageOf<PixelMono>& _V = VofHSVPort.prepare();
        _V.resize(this->widthLP,this->heightLP);
        //cvNamedWindow("test");
        //cvShowImage("test",(IplImage*)(cartIntensImg->getIplImage()));
        //cvWaitKey(0);
        
        if(true){
                //performs centre-surround uniqueness analysis on first plane
                
                centerSurr->proc_im_8u( (IplImage*)chromUnXtnIntensImg->getIplImage(),(IplImage*)_Y.getIplImage());
                //cvNamedWindow("_Y");
                //cvShowImage("_Y",(IplImage*)chromUnXtnIntensImg->getIplImage());
                //cvWaitKey(2);
                cvSet(cs_tot_32f,cvScalar(0));
                //cvNamedWindow("test");
                //cvShowImage("test",(IplImage*)(chromUplane->getIplImage()));
                //cvWaitKey(0);
                
                if (isYUV){

                    
                    
                
                    //performs centre-surround uniqueness analysis on second plane:
                    centerSurr->proc_im_8u( (IplImage*)(chromUplane->getIplImage()),scs_out);
                    cvAdd(centerSurr->get_centsur_32f(),cs_tot_32f,cs_tot_32f); // in place?

                      
                    
                    //Colour process V:performs centre-surround uniqueness analysis:
                    centerSurr->proc_im_8u( (IplImage*)(this->chromVplane->getIplImage()), vcs_out);
                    cvAdd(centerSurr->get_centsur_32f(),cs_tot_32f,cs_tot_32f);
                    
                    
                    //get min max   
                    double valueMin = 1000;
                    double valueMax = -1000;
                    img_UV->zero();     // this is not strictly required
                  	cvMinMaxLoc(cs_tot_32f,&valueMin,&valueMax);            
                    if ( valueMax == valueMin || valueMin < -1000 || valueMax > 1000){ 
                        valueMax = 255.0f; valueMin = 0.0f;
                    }
                    cvConvertScale(cs_tot_32f,(IplImage*)_UV.getIplImage(),255/(valueMax - valueMin),-255*valueMin/(valueMax-valueMin)); //LATER
                    //cvConvertScale(cs_tot_32f,(IplImage*)img_UV->getIplImage(),255,0);
                    
                    
                    
                    
                }
                else{
                    //performs centre-surround uniqueness analysis on second plane:
                    centerSurr->proc_im_8u( (IplImage*)(this->chromUplane->getIplImage()),(IplImage*)_UV.getIplImage() );
                    //Colour process V:performs centre-surround uniqueness analysis:
                    centerSurr->proc_im_8u( (IplImage*)(this->chromVplane->getIplImage()), (IplImage*)_V.getIplImage());           

                }


                
                    
                
 /*              

                //this is nasty, resizes the images...
                unsigned char* imgY = img_Y->getPixelAddress( maxKernelSize, maxKernelSize );
                unsigned char* imgUV = img_UV->getPixelAddress( maxKernelSize, maxKernelSize );
                unsigned char* imgV;
                unsigned char* imgVo;

                if (!isYUV){
                   imgV = img_V->getPixelAddress( maxKernelSize, maxKernelSize );
                   imgVo = _V.getRawImage();
                }
                
                unsigned char* imgYo = _Y.getRawImage();
                unsigned char* imgUVo = _UV.getRawImage();
                int rowsize= _Y.getRowSize();
                int rowsize2= img_Y->getRowSize();

                for(int row=0; row<heightLP; row++) {
                    for(int col=0; col<widthLP; col++) {
                        *imgYo  = *imgY;
                        *imgUVo = *imgUV;
                        if (!isYUV) {
                            *imgVo = *imgV;
                            imgVo++;  imgV++;          
                        }
                        imgYo++;  imgUVo++;
                        imgY++;   imgUV++;
                    }    
                    imgYo+=rowsize - widthLP;
                    imgUVo+=rowsize - widthLP;
                    imgY+=rowsize2 - widthLP;
                    imgUV+=rowsize2 - widthLP;
                    if (!isYUV) {
                        imgVo+=rowsize - widthLP;
                        imgV+=rowsize2 - widthLP;       
                    }
                }

  */              
                //output Y centre-surround results to ports
                if (intensityCSPort.getOutputCount() ){
                    intensityCSPort.write();
                }

                //output UV centre-surround results to ports
                if ( chromPort.getOutputCount() ){
                    chromPort.write();
                }
                //output UV centre-surround results to ports
                if ( !isYUV && VofHSVPort.getOutputCount()){
                    VofHSVPort.write();
                }

                
        #ifdef DEBUG_OPENCV
                cvNamedWindow("CS_Y");
                cvShowImage("CS_Y", (IplImage*)_Y.getIplImage());
                cvNamedWindow("CS_UV");
                cvShowImage("CS_UV", (IplImage*)_UV.getIplImage());
                cvNamedWindow("CS_V");
                cvShowImage("CS_V", (IplImage*)_V.getIplImage());
                cvWaitKey(2);
        #endif
                
            

    }
        //printf("STOP center surround \n");
       
}




void chrominanceThread::orientation() {
    //printf("start orientation \n");
    int cartesWidth  = cartIntensImg->width();
    int cartesHeight = cartIntensImg->height();

    // orientation port
    ImageOf<PixelMono>& ori0   = orientPort0.prepare();    
    ImageOf<PixelMono>& ori45  = orientPort45.prepare();    
    ImageOf<PixelMono>& ori90  = orientPort90.prepare();    
    ImageOf<PixelMono>& oriM45 = orientPortM45.prepare();
    ImageOf<PixelMono>& totImg = totalOrientImagePort.prepare();
    
    ori0.resize(cartesWidth,cartesHeight);
    ori45.resize(cartesWidth,cartesHeight);
    ori90.resize(cartesWidth,cartesHeight);
    oriM45.resize(cartesWidth,cartesHeight);
    totImg.resize(cartesWidth,cartesHeight);
/*
    ori0.zero();
    ori45.zero();
    ori90.zero();
    oriM45.zero();
    totImg.zero();
*/
 
    // using Kirsch cum positive Gaussian matrix
    kirschSalPos0->convolve2D(cartIntensImg,o0);
    kirschSalPos45->convolve2D(cartIntensImg,o45);
    kirschSalPos90->convolve2D(cartIntensImg,o90);
    kirschSalPos45->convolve2D(cartIntensImg,oM45); 

    // using Kirsch cum Negative gaussian matrix
    kirschSalNeg0->convolve2D(cartIntensImg,tmpKirschCartImage1); 
    kirschSalNeg45->convolve2D(cartIntensImg,tmpKirschCartImage2); 
    kirschSalNeg90->convolve2D(cartIntensImg,tmpKirschCartImage3); 
    kirschSalNegM45->convolve2D(cartIntensImg,tmpKirschCartImage4);   

/*    cvNamedWindow("pos90");
    cvShowImage("pos90",(IplImage*)o90->getIplImage());
    cvNamedWindow("neg0");
    cvShowImage("neg0",(IplImage*)o0->getIplImage());
    cvNamedWindow("neg45");
    cvShowImage("neg45",(IplImage*)o45->getIplImage());
    cvNamedWindow("negM45");
    cvShowImage("negM45",(IplImage*)oM45->getIplImage());
    cvWaitKey(0);
      
*/


    uchar* ori[4]= {(uchar*)ori0.getRawImage(),
                    (uchar*)ori45.getRawImage(),
                    (uchar*)ori90.getRawImage(),
                    (uchar*)oriM45.getRawImage()};

    float *ptrTotKir = (float*)totalKirsch->getRawImage();

    KirschOutputImagePtr* p[4] = { (KirschOutputImagePtr*)o0->getRawImage(),
                                    (KirschOutputImagePtr*)o45->getRawImage(),
                                    (KirschOutputImagePtr*)o90->getRawImage(),
                                    (KirschOutputImagePtr*)oM45->getRawImage()};

    KirschOutputImagePtr* n[4] = { (KirschOutputImagePtr*)tmpKirschCartImage1->getRawImage(),
                                    (KirschOutputImagePtr*)tmpKirschCartImage2->getRawImage(),
                                    (KirschOutputImagePtr*)tmpKirschCartImage3->getRawImage(),
                                    (KirschOutputImagePtr*)tmpKirschCartImage4->getRawImage()};

     
    const int pad_output = ori0.getPadding() / sizeof(uchar);
    int padK             = o0->getPadding()  / sizeof(KirschOutputImagePtr); 
       

    // LATER: Do not consider extended portion
#ifdef USE_PROPORTIONAL_KIRSCH
    float normalizingRatio[4];
    normalizingRatio[0] = 255.0 / (kirschLimits[0][0] - kirschLimits[0][1]);
    normalizingRatio[1] = 255.0 / (kirschLimits[1][0] - kirschLimits[1][1]);
    normalizingRatio[2] = 255.0 / (kirschLimits[2][0] - kirschLimits[2][1]);
    normalizingRatio[3] = 255.0 / (kirschLimits[3][0] - kirschLimits[3][1]);
#endif

    float multiplier = 1.0;
    float multiplierForNeg = .5;
    for (int row = 0; row < ori0.height(); row++) {
        for (int col = 0; col < ori0.width(); col++) {

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
                        *ptrTotKir = max(max(tmpV0,tmpV1),max(tmpV2,tmpV3)); 
                                                
                    
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
                        *ptrTotKir = tmpV0;//max(max(tmpV0,tmpV1),max(tmpV2,tmpV3)); 
                        
                }
                
            
#else //--------------------------------------------------------------

                float tmpV0 = (abs(*p[0])-multiplierForNeg*(abs(*n[1])+abs(*n[2])+abs(*n[3]))) *multiplier ;
                float tmpV1 = (abs(*p[1])-multiplierForNeg*(abs(*n[0])+abs(*n[2])+abs(*n[3]))) *multiplier ;
                float tmpV2 = (abs(*p[2])-multiplierForNeg*(abs(*n[0])+abs(*n[1])+abs(*n[3]))) *multiplier ;
                float tmpV3 = (abs(*p[3])-multiplierForNeg*(abs(*n[1])+abs(*n[2])+abs(*n[0]))) *multiplier ;
                *ori[0] = tmpV0>255?255:tmpV0<0?0:(unsigned char)tmpV0;
                *ori[1] = tmpV1>255?255:tmpV1<0?0:(unsigned char)tmpV1;
                *ori[2] = tmpV2>255?255:tmpV2<0?0:(unsigned char)tmpV2;
                *ori[3] = tmpV3>255?255:tmpV3<0?0:(unsigned char)tmpV3; 
                *ptrTotKir = max(max(tmpV0,tmpV1),max(tmpV2,tmpV3));
             
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


    IplImage* imagesToMaxAdd[2] = {(IplImage*)ori0.getIplImage(),(IplImage*)ori90.getIplImage()};
    //totalKirsch->zero();
    //cvSub((IplImage*)o0->getIplImage(),(IplImage*)o90->getIplImage(),(IplImage*)totalKirsch->getIplImage());
    cvConvertScale((IplImage*)totalKirsch->getIplImage(),(IplImage*)totImg.getIplImage(),255,0);
    //convertFloatToUchar(totalKirsch,&totImg,255*.75);
    //cvAdd((IplImage*)ori0.getIplImage(),(IplImage*)ori90.getIplImage(),(IplImage*)ori45.getIplImage());
    //maxImages(imagesToMaxAdd,2,(IplImage*)totImg.getIplImage());

//#ifdef DEBUG_OPENCV
    cvNamedWindow("Original");
    cvShowImage("Original",(IplImage*)totalKirsch->getIplImage());
    cvNamedWindow("Orient0");
    cvShowImage("Orient0",  (IplImage*)totImg.getIplImage());
    cvNamedWindow("Orient45");
    cvShowImage("Orient45", (IplImage*)ori45.getIplImage());
    cvNamedWindow("Orient90");
    cvShowImage("Orient90", (IplImage*)ori90.getIplImage());
    cvNamedWindow("OrientM45");
    cvShowImage("OrientM45",(IplImage*)oM45->getIplImage());
    cvWaitKey(2);
//#endif
     
    orientPort0.write();
    orientPort45.write();
    orientPort90.write();
    orientPortM45.write();
    totalOrientImagePort.write();
    

    //printf("STOP orientation \n \n");
  
}

void chrominanceThread::maxImages(IplImage** ImagesTobeAdded, int numberOfImages,IplImage* resultantImage) {
    
    cvSet(resultantImage,cvScalar(0));
    uchar* resImageOrigin = (uchar*)resultantImage->imageData;
    int resImageWidth = resultantImage->widthStep;
    uchar* tmpResultImage = (uchar*)resultantImage->imageData;
    for(int i=0; i< numberOfImages; ++i){
        IplImage* tmpImageTobeAdded = ImagesTobeAdded[i];
        if(tmpImageTobeAdded != 0){   // save yourself some pain    
            uchar* tmpImage = (uchar*)tmpImageTobeAdded->imageData;
            //Images must be same size, type etc
            int h = tmpImageTobeAdded->height;
            int w = tmpImageTobeAdded->width;
            if(h > resultantImage->height || w > resultantImage->width) {
                printf("Image too big. \n");
                continue ;
            }
            uchar* imgToBeAddOrigin = (uchar*)tmpImageTobeAdded->imageData;
            int imgToBeAddWidth = tmpImageTobeAdded->widthStep;
            for(int j=0; j<h; ++j){
                tmpImage = imgToBeAddOrigin + j* imgToBeAddWidth; 
                tmpResultImage = resImageOrigin + j*resImageWidth;
                for(int k=0; k<w; ++k){
                    unsigned char valNow = (unsigned char)(*tmpImage);
                    if(*tmpResultImage < valNow) *tmpResultImage = valNow; 
                    //*tmpResultImage = (unsigned char)(*tmpImage) * itsWt;
                    *tmpImage++;
                    *tmpResultImage++;
                }
            }
        }
    }

}

void chrominanceThread::threadRelease() {    

    printf("Releasing\n");

    lpMono.freeLookupTables();

    cvReleaseImage(&cs_tot_32f); 
    cvReleaseImage(&colcs_out);
    cvReleaseImage(&ycs_out);
    cvReleaseImage(&scs_out);
    cvReleaseImage(&vcs_out);

    intensityCSPort.interrupt();
    chromPort.interrupt();
    orientPort0.interrupt();
    orientPort45.interrupt();
    orientPort90.interrupt();
    orientPortM45.interrupt();    
    
    intensityCSPort.close();
    chromPort.close();
    orientPort0.close();
    orientPort45.close();
    orientPort90.close();
    orientPortM45.close();

    //deallocating resources
    delete chromYplane;
    delete chromUplane;
    delete chromVplane;
    delete kirschSalPos0;
    delete kirschSalPos45;
    delete kirschSalPos90;
    delete kirschSalPosM45;
    delete o0;
    delete o45;
    delete o90;
    delete oM45;
    delete tmpKirschCartImage1;    
    delete tmpKirschCartImage2;    
    delete tmpKirschCartImage3;    
    delete tmpKirschCartImage4;
    delete totalKirsch;    
    delete cartIntensImg;
    // CS
    delete centerSurr;
    delete img_Y;
    delete img_UV;
    delete img_V;
    
    printf("done with release\n");
    
}


