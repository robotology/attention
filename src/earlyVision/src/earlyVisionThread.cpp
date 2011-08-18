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
 * @file earlyVisionThread.cpp
 * @brief Implementation of the early stage of vision thread (see earlyVisionThread.h).
 */

#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/earlyVisionThread.h>
#include <cstring>

#define ONE_BY_ROOT_TWO 0.707106781
#define ONE_BY_ROOT_THREE 0.577350269
#define USE_PROPORTIONAL_KIRSCH
#define NO_DEBUG_OPENCV //DEBUG_OPENCV //

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


template<class T>
inline T max(T a, T b, T c) {    
    if(b > a) a = b;
    if(c > a) a = c;
    return a;
}

earlyVisionThread::earlyVisionThread() {
    
    inputImage          = new ImageOf<PixelRgb>;
    filteredInputImage  = new ImageOf<PixelRgb>;
    extendedInputImage  = new ImageOf<PixelRgb>;
    
    Rplus               = new ImageOf<PixelMono>;
    Rminus              = new ImageOf<PixelMono>;
    Gplus               = new ImageOf<PixelMono>;
    Gminus              = new ImageOf<PixelMono>;
    Bplus               = new ImageOf<PixelMono>;
    Bminus              = new ImageOf<PixelMono>;
    Yminus              = new ImageOf<PixelMono>;

    
    tmpMonoLPImage      = new ImageOf<PixelMono>;

    tmpMono16LPImage    = new ImageOf<PixelMono16>;
    tmpMono16LPImage1   = new ImageOf<PixelMono16>;
    tmpMono16LPImage2   = new ImageOf<PixelMono16>;

    

    tmpMonoSobelImage1  = new SobelOutputImage;
    tmpMonoSobelImage2  = new SobelOutputImage;

    o0      = new KirschOutputImage;
    o45     = new KirschOutputImage;
    o90     = new KirschOutputImage;
    oM45    = new KirschOutputImage;
    
    

    tmpMonoLPImageSobelHorz      = new ImageOf<PixelMono>;
    tmpMonoLPImageSobelVert      = new ImageOf<PixelMono>;

    
    
    

    edges               = new yarp::sig::ImageOf<yarp::sig::PixelMono>;
    
    YofYUV              = new ImageOf<PixelMono>;    
    intensImg           = new ImageOf<PixelMono>;
    unXtnIntensImg      = new ImageOf<PixelMono>;
    cartIntensImg       = new ImageOf<PixelMono>;
    
    redPlane            = new ImageOf<PixelMono>;
    greenPlane          = new ImageOf<PixelMono>;
    bluePlane           = new ImageOf<PixelMono>;
    yellowPlane         = new ImageOf<PixelMono>;

    Yplane            = new ImageOf<PixelMono>;
    Uplane            = new ImageOf<PixelMono>;
    Vplane            = new ImageOf<PixelMono>;

    gaborPosHorConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,0,.5,0);
    gaborPosVerConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,1,.5,0);
    gaborNegHorConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,0,.5,0);
    gaborNegVerConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,1,.5,0);

    kirschConvolution0 =  new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(3,3,K1,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschConvolution45 =  new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(3,3,K6,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschConvolution90 =  new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(3,3,K5,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    kirschConvolutionM45 = new convolve<ImageOf<PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr>(3,3,K7,KIRSCH_FACTOR,KIRSCH_SHIFT,KIRSCH_FLICKER);
    
    sobel2DXConvolution = new convolve<ImageOf<PixelMono>,uchar,SobelOutputImage,SobelOutputImagePtr>(5,5,Sobel2DXgrad,SOBEL_FACTOR,SOBEL_SHIFT);
    sobel2DYConvolution = new convolve<ImageOf<PixelMono>,uchar,SobelOutputImage,SobelOutputImagePtr>(5,5,Sobel2DYgrad,SOBEL_FACTOR,SOBEL_SHIFT);
    sobelIsNormalized = 0;
    sobelLimits[0] = 0;
    sobelLimits[1] = 2.0;

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
	
	
    
    
    
    // Let us initialize IplImage pointers to NULL

    //hRG =vRG=hGR=vGR=hBY=vBY= NULL;

    //16 bit image to avoid overflow in Sobel operator
    //tempHRG=tempVRG=tempHGR=tempVGR=tempHBY=tempVBY=NULL;    
          
    
    lambda = 0.3f;
    resized = false;

    //Logpolar to cartesian and vice versa
    xSizeValue = CART_ROW_SIZE ;         
    ySizeValue = CART_COL_SIZE;          // y dimension of the remapped cartesian image
    overlap = 1.0;         // overlap in the remapping
    numberOfRings = COL_SIZE;      // 152, number of rings in the remapping
    numberOfAngles = ROW_SIZE;     // number of angles in the remapping   
    
    //Logpolar to cartesian and vice versa
    xSizeValue = 320 ;         
    ySizeValue = 240;          // y dimension of the remapped cartesian image
    overlap = 1.0;         // overlap in the remapping
    numberOfRings = 152;      // number of rings in the remapping
    numberOfAngles = 252;     // number of angles in the remapping
}

earlyVisionThread::~earlyVisionThread() {
    
    delete inputImage;
    delete filteredInputImage;
    delete extendedInputImage;
    delete Rplus;
    delete Rminus;
    delete Gplus;
    delete Gminus;
    delete Bplus;
    delete Bminus;
    delete Yminus;
    delete tmpMonoLPImage;
    delete tmpMono16LPImage;
    delete tmpMono16LPImage1;
    delete tmpMono16LPImage2;
    delete tmpMonoLPImageSobelHorz;
    delete tmpMonoLPImageSobelVert;
    delete tmpMonoSobelImage1;
    delete tmpMonoSobelImage2;
    delete gaborPosHorConvolution;    
    delete gaborPosVerConvolution;    
    delete gaborNegHorConvolution;    
    delete gaborNegVerConvolution;
    delete kirschConvolution0;
    delete kirschConvolution45;
    delete kirschConvolution90;
    delete kirschConvolutionM45;
    delete sobel2DXConvolution;
    delete sobel2DYConvolution;
    delete o0;
    delete o45;
    delete o90;
    delete oM45;    
    delete edges;
    delete YofYUV;
    delete intensImg;
    delete unXtnIntensImg;
    delete cartIntensImg;
    delete redPlane;
    delete greenPlane;
    delete bluePlane;
    delete yellowPlane;
    delete Yplane;
    delete Uplane;
    delete Vplane;

    // CS
    delete centerSurr;
    delete img_Y;
    delete img_UV;
    delete img_V;

    
    
}

bool earlyVisionThread::threadInit() {
    printf("opening ports \n");
    /* open ports */ 
    
    
    if (!imagePortIn.open(getName("/imageRGB:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!intenPort.open(getName("/intensity:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    
    if (!chromPort.open(getName("/chrominance:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!edgesPort.open(getName("/edges:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!orientPort0.open(getName("/orientation0:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!orientPort45.open(getName("/orientation45:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!orientPort90.open(getName("/orientation90:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!orientPortM45.open(getName("/orientationM45:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!colorOpp1Port.open(getName("/colorOppR+G-:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!colorOpp2Port.open(getName("/colorOppG+R-:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!colorOpp3Port.open(getName("/colorOppB+Y-:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }   

    // CS ports
    if (!CSPort1.open(getName("/centerSurround1:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!CSPort2.open(getName("/centerSurround2:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!CSPort3.open(getName("/centerSurround3:o").c_str())) {
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

void earlyVisionThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string earlyVisionThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void earlyVisionThread::run() {
    
    
    while (isStopping() != true) {
        
        inputImage  = imagePortIn.read(true);
        /*IplImage *imgRGB;
        imgRGB = cvLoadImage("logPtemp.jpg");
        inputImage->resize(imgRGB->width,imgRGB->height);
        inputImage->zero();
        cvAdd(imgRGB,(IplImage*)inputImage->getIplImage(),(IplImage*)inputImage->getIplImage());
        
        //cvNamedWindow("cnvt");
        //cvShowImage("cnvt",(IplImage*)inputImage->getIplImage());
        //cvWaitKey(0);
*/
        if (inputImage != NULL) {
            if (!resized) {
                resize(inputImage->width(), inputImage->height());
                filteredInputImage->zero(); 
                resized = true;
            }
            
            filterInputImage();
            
            extender(maxKernelSize);
             //printf("red plus dimension in resize3  %d %d \n", cvRedPlus->width, cvRedPlus->height);
            
            // extract RGB and Y planes
            extractPlanes();
             //printf("red plus dimension in resize4  %d %d \n", cvRedPlus->width, cvRedPlus->height);
                      
            // gaussian filtering of the of RGB and Y
            filtering();
            
            // Center-surround
            centerSurrounding();

            // colourOpponency map construction
            colorOpponency();

            //printf("before colour opponency \n");
            orientation();
            // apply sobel operators on the colourOpponency maps and combine via maximisation of the 3 edges

            
            edgesExtract();
                    

            if((intensImg!=0)&&(intenPort.getOutputCount())) {
                intenPort.prepare() = *(intensImg);
                intenPort.write();
            }

            if((Yplane!=0)&&(chromPort.getOutputCount())) {
                chromPort.prepare() = *(Yplane);
                chromPort.write();
                }

            if((edges!=0)&&(edgesPort.getOutputCount())) {
                edgesPort.prepare() = *(edges);
                edgesPort.write();
            }

            
#ifdef DEBUG_OPENCV
            cvWaitKey(0);
#endif
            
            
        }
    }
}



void earlyVisionThread::resize(int width_orig,int height_orig) {


    this->width_orig = inputImage->width();//width_orig;
    this->height_orig = inputImage->height();//height_orig;
    
    width = this->width_orig+2*maxKernelSize;
    height = this->height_orig+maxKernelSize;

    
    
    //resizing yarp image 
    filteredInputImage->resize(this->width_orig, this->height_orig);
    extendedInputImage->resize(width, height);
    Rplus->resize(width, height);
    Rminus->resize(width, height);
    Gplus->resize(width, height);
    Gminus->resize(width, height);
    Bplus->resize(width, height);
    Bminus->resize(width, height);
    Yminus->resize(width, height);

    
    tmpMonoLPImage->resize(width, height);
    tmpMono16LPImage->resize(width, height);
    tmpMono16LPImage1->resize(width, height);
    tmpMono16LPImage2->resize(width, height);
    tmpMonoLPImageSobelHorz->resize(width, height);
    tmpMonoLPImageSobelVert->resize(width, height);
    tmpMonoSobelImage1->resize(width,height);
    tmpMonoSobelImage2->resize(width,height);

    
    
    

    edges->resize(width, height);
    intensImg->resize(width, height);
    unXtnIntensImg->resize(this->width_orig,this->height_orig);

    cartIntensImg->resize(CART_ROW_SIZE, CART_COL_SIZE);
    // for Kirsch
    o0->resize(CART_ROW_SIZE, CART_COL_SIZE);
    o45->resize(CART_ROW_SIZE, CART_COL_SIZE);
    o90->resize(CART_ROW_SIZE, CART_COL_SIZE);
    oM45->resize(CART_ROW_SIZE, CART_COL_SIZE);

    redPlane->resize(width, height);
    greenPlane->resize(width, height);
    bluePlane->resize(width, height);
    yellowPlane->resize(width, height);
    Yplane->resize(width, height);
    Uplane->resize(width, height);
    Vplane->resize(width, height);
    
    // allocating for CS ncsscale = 4;   

    cs_tot_32f  = cvCreateImage( cvSize(width, height),IPL_DEPTH_32F, 1  );
    colcs_out   = cvCreateImage( cvSize(width, height),IPL_DEPTH_8U, 1  );
    ycs_out     = cvCreateImage( cvSize(width, height),IPL_DEPTH_8U, 1  );
    scs_out     = cvCreateImage( cvSize(width, height),IPL_DEPTH_8U, 1  );
    vcs_out     = cvCreateImage( cvSize(width, height),IPL_DEPTH_8U, 1  );
    
    
    centerSurr  = new CenterSurround( width,height,1.0 );

    
    isYUV = true;
	img_Y->resize( this->width, this->height );

    

    img_UV->resize( this->width, this->height );

    

    img_V->resize( this->width, this->height );

    
   
    
}

void earlyVisionThread::filterInputImage() {
    
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


void earlyVisionThread::extender(int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*extendedInputImage, *filteredInputImage, maxSize);    
    
}

void earlyVisionThread::extractPlanes() {

    // We extract color planes from the RGB image. Planes are red,blue, green and yellow (AM of red & green)
    uchar* shift[4];
    uchar* YUV[3];
    int padInput;
    int padUnX;
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
    ptrIntensityImg   = (uchar*) intensImg->getRawImage();
    ptrUnXtnIntensImg = (uchar*) unXtnIntensImg->getRawImage();
    inputPointer      = (uchar*) extendedInputImage->getRawImage();
    padInput          = extendedInputImage->getPadding();
    padMono           = redPlane->getPadding();
    padUnX            = unXtnIntensImg->getPadding();
    

    const int h = extendedInputImage->height();
    const int w = extendedInputImage->width();

    for(int r = 0; r < h; r++) {       
        
        for(int c = 0; c < w; c++) {
            *shift[0] = *inputPointer++;
            *shift[1] = *inputPointer++;
            *shift[2] = *inputPointer++;

            *shift[3]++ = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));
            *ptrIntensityImg = ONE_BY_ROOT_THREE * sqrt(*shift[0] * *shift[0] +*shift[1] * *shift[1] +*shift[2] * *shift[2]);
            if(r>=maxKernelSize && c >=maxKernelSize && c< w-maxKernelSize){
                *ptrUnXtnIntensImg++ = *ptrIntensityImg;
                
            }

            // RGB to Y'UV conversion
            *YUV[0] = .299* (*shift[0]) + .587 * (*shift[1]) + .114 * (*shift[2]);
            *YUV[1] = -.14713* (*shift[0]) + -.28886 * (*shift[1]) + .436 * (*shift[2]);
            *YUV[2] = .615* (*shift[0]) + -.51499 * (*shift[1]) + -.10001 * (*shift[2]);

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
        }
        shift[0] += padMono;
        shift[1] += padMono;
        shift[2] += padMono;
        shift[3] += padMono;
        YUV[0] += padMono;
        YUV[1] += padMono;
        YUV[2] += padMono;
        
                
    } 

    lpMono.logpolarToCart(*cartIntensImg,*unXtnIntensImg);

}

void earlyVisionThread::filtering() {
    // We gaussian blur the image planes extracted before, one with positive Gaussian and then negative
    
    //Positive
    tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(redPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Rplus);
    

    tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(bluePlane,tmpMonoLPImage);
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

void earlyVisionThread::centerSurrounding(){

        // Allocate temporarily
        ImageOf<PixelMono>& _Y = CSPort1.prepare();
        _Y.resize(this->width_orig,this->height_orig);
        ImageOf<PixelMono>& _UV = CSPort2.prepare();
        _UV.resize(this->width_orig,this->height_orig);
        ImageOf<PixelMono>& _V = CSPort3.prepare();
        _V.resize(this->width_orig,this->height_orig);
        
        //performs centre-surround uniqueness analysis on first plane
        centerSurr->proc_im_8u( (IplImage*)Yplane->getIplImage(),(IplImage*)img_Y->getIplImage());
        /*cvNamedWindow("Yplane");
          cvShowImage("Yplane",(IplImage*)img_Y->getIplImage());*/
        cvSet(cs_tot_32f,cvScalar(0));
        
        
        if ( isYUV ){
            //performs centre-surround uniqueness analysis on second plane:
            centerSurr->proc_im_8u( (IplImage*)Uplane->getIplImage(),scs_out );
            cvAdd(centerSurr->get_centsur_32f(),cs_tot_32f,cs_tot_32f); // in place?
            /*cvNamedWindow("AfterCSofUplane");
              cvShowImage("AfterCSofUplane",cs_tot_32f);*/
            //Colour process V:performs centre-surround uniqueness analysis:
            centerSurr->proc_im_8u( (IplImage*)Vplane->getIplImage(), vcs_out);
            cvAdd(centerSurr->get_centsur_32f(),cs_tot_32f,cs_tot_32f);
            /*cvNamedWindow("AfterCSofVplane");
              cvShowImage("AfterCSofVplane",cs_tot_32f);*/
            
            //get min max   
            double valueMin = 1000;
            double valueMax = -1000;
            img_UV->zero();     // this is not strictly required
          	cvMinMaxLoc(cs_tot_32f,&valueMin,&valueMax);            
            if ( valueMax == valueMin || valueMin < -1000 || valueMax > 1000){ 
                valueMax = 255.0f; valueMin = 0.0f;
            }
            cvConvertScale(cs_tot_32f,(IplImage*)img_UV->getIplImage(),255/(valueMax - valueMin),-255*valueMin/(valueMax-valueMin)); //LATER
            //cvConvertScale(cs_tot_32f,(IplImage*)img_UV->getIplImage(),255,0);
            /*cvNamedWindow("AfterCSscale");
            cvShowImage("AfterCSscale",(IplImage*)img_UV->getIplImage());
            cvWaitKey(0);*/
            
            
            
        }
        else{
            //performs centre-surround uniqueness analysis on second plane:
            centerSurr->proc_im_8u( (IplImage*)Uplane->getIplImage(),(IplImage*)img_UV->getIplImage() );
            //Colour process V:performs centre-surround uniqueness analysis:
            centerSurr->proc_im_8u( (IplImage*)Vplane->getIplImage(), (IplImage*)img_V->getIplImage());           

        }


        /*cvNamedWindow("Y");
        //cvShowImage("Y",(IplImage*)img_Y->getIplImage());
        //cvNamedWindow("UV");
        //cvShowImage("UV",(IplImage*)img_UV->getIplImage());
        //cvWaitKey(0);*/
        
            
        
       

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

        for(int row=0; row<height_orig; row++) {
            for(int col=0; col<width_orig; col++) {
                *imgYo  = *imgY;
                *imgUVo = *imgUV;
                if (!isYUV) {
                    *imgVo = *imgV;
                    imgVo++;  imgV++;          
                }
                imgYo++;  imgUVo++;
                imgY++;   imgUV++;
            }    
            imgYo+=rowsize - width_orig;
            imgUVo+=rowsize - width_orig;
            imgY+=rowsize2 - width_orig;
            imgUV+=rowsize2 - width_orig;
            if (!isYUV) {
                imgVo+=rowsize - width_orig;
                imgV+=rowsize2 - width_orig;       
            }
        }

        
        //output Y centre-surround results to ports
        if ( CSPort1.getOutputCount()>0 ){
            CSPort1.write();
        }

        //output UV centre-surround results to ports
        if ( CSPort2.getOutputCount()>0 ){
            CSPort2.write();
        }
        //output UV centre-surround results to ports
        if ( !isYUV && CSPort3.getOutputCount()>0 ){
            CSPort3.write();
        }

#ifdef DEBUG_OPENCV
        cvNamedWindow("CS_Y");
        cvShowImage("CS_Y", (IplImage*)_Y.getIplImage());
        cvNamedWindow("CS_UV");
        cvShowImage("CS_UV", (IplImage*)_UV.getIplImage());
        cvNamedWindow("CS_V");
        cvShowImage("CS_V", (IplImage*)_V.getIplImage());
#endif
       
}

void earlyVisionThread::colorOpponency(){

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

    uchar* rPlus = Rplus->getRawImage();
    uchar* rMinus = Rminus->getRawImage();
    uchar* gPlus = Gplus->getRawImage();
    uchar* gMinus = Gminus->getRawImage();
    uchar* bPlus = Bplus->getRawImage();
    uchar* yMinus = Yminus->getRawImage();

    int padChannel = Rplus->getPadding();
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
#endif

}

void earlyVisionThread::orientation() {

    int cartesWidth  = cartIntensImg->width();
    int cartesHeight = cartIntensImg->height();

    // orientation port
    ImageOf<PixelMono>& ori0   = orientPort0.prepare();    
    ImageOf<PixelMono>& ori45  = orientPort45.prepare();    
    ImageOf<PixelMono>& ori90  = orientPort90.prepare();    
    ImageOf<PixelMono>& oriM45 = orientPortM45.prepare();
    
    ori0.resize(cartesWidth,cartesHeight);
    ori45.resize(cartesWidth,cartesHeight);
    ori90.resize(cartesWidth,cartesHeight);
    oriM45.resize(cartesWidth,cartesHeight);

    ori0.zero();
    ori45.zero();
    ori90.zero();
    oriM45.zero();
       
    // Using Kirsch matrix
    kirschConvolution0->convolve2D(cartIntensImg,o0);
    kirschConvolution45->convolve2D(cartIntensImg,o45);
    kirschConvolution90->convolve2D(cartIntensImg,o90);
    kirschConvolutionM45->convolve2D(cartIntensImg,oM45);
        

    uchar* ori[4]= {(uchar*)ori0.getRawImage(),
                    (uchar*)ori45.getRawImage(),
                    (uchar*)ori90.getRawImage(),
                    (uchar*)oriM45.getRawImage()};

    KirschOutputImagePtr* p[4] = { (KirschOutputImagePtr*)o0->getRawImage(),
                                    (KirschOutputImagePtr*)o45->getRawImage(),
                                    (KirschOutputImagePtr*)o90->getRawImage(),
                                    (KirschOutputImagePtr*)oM45->getRawImage()};

     
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

    for (int row = 0; row < ori0.height(); row++) {
        for (int col = 0; col < ori0.width(); col++) {

#ifdef USE_PROPORTIONAL_KIRSCH //-------------------------------------
            //if() {
                //printf("second proportional kirsch \n");
                //if (row < CART_ROW_SIZE) {
                if(kirschIsNormalized < KIRSCH_FLICKER){
                    for(int i=0; i<4; ++i){
                        kirschLimits[i][0] = kirschLimits[i][0] < *p[i] ? *p[i] : kirschLimits[i][0]; // max
                        kirschLimits[i][1] = kirschLimits[i][1] > *p[i] ? *p[i] : kirschLimits[i][1]; // min
                        *ori[i] = abs(*p[i]) * 255;                                                
                    }
                }
                else {
                    for(int i=0; i<4; ++i){
                        //float tmpV = 255.0 *abs(*p[i]);
                        float tmpV = normalizingRatio[i]*(*p[i] - kirschLimits[i][1]);
                        *ori[i] = tmpV>255?255:tmpV<0?0:(unsigned char)tmpV;
                    }
                }
                
            
#else //--------------------------------------------------------------
                for(int i = 0; i < 4; ++i) {
                    float tmpV = 1 * (*p[i]);
                    //*ori[i] = tmpV > 255 ? 255 : (unsigned char)tmpV;
                    *ori[i] = (unsigned char)tmpV;
                }                  
#endif //---------------------------------------------------------------
            ori[0]++;
            p[0]++;
            ori[1]++;
            p[1]++;
            ori[2]++;
            p[2]++;
            ori[3]++;
            p[3]++;
             
            
        }
        // padding
        ori[0] += pad_output;
        p[0] += padK; 
        ori[1] += pad_output;
        p[1] += padK; 
        ori[2] += pad_output;
        p[2] += padK; 
        ori[3] += pad_output;
        p[3] += padK; 
               
    } 

    kirschIsNormalized++;

#ifdef DEBUG_OPENCV
    cvNamedWindow("Orient0");
    cvShowImage("Orient0",  (IplImage*)o0->getIplImage());
    cvNamedWindow("Orient45");
    cvShowImage("Orient45", (IplImage*)ori45.getIplImage());
    cvNamedWindow("Orient90");
    cvShowImage("Orient90", (IplImage*)ori90.getIplImage());
    cvNamedWindow("OrientM45");
    cvShowImage("OrientM45",(IplImage*)oriM45.getIplImage());
    cvWaitKey(0);
#endif
     
    orientPort0.write();
    orientPort45.write();
    orientPort90.write();
    orientPortM45.write();  
}



void earlyVisionThread::edgesExtract() {
    
    // X derivative 
    tmpMonoSobelImage1->zero();
    sobel2DXConvolution->convolve2D(intensImg,tmpMonoSobelImage1);

    // Y derivative
    tmpMonoSobelImage2->zero();     // This can be removed 
    sobel2DYConvolution->convolve2D(intensImg,tmpMonoSobelImage2);
    
    
    

    //clearing up the previous value
    edges->zero();

    uchar* pedges= (uchar*)edges->getRawImage();
    SobelOutputImagePtr* ptrHorz = (SobelOutputImagePtr*)tmpMonoSobelImage1->getRawImage();
    SobelOutputImagePtr* ptrVert = (SobelOutputImagePtr*)tmpMonoSobelImage2->getRawImage();
     
    const int pad_edges = edges->getPadding()/sizeof(uchar);
    int padHorz = tmpMonoSobelImage1->getPadding()/sizeof(SobelOutputImagePtr);
    int padVert = tmpMonoSobelImage2->getPadding()/sizeof(SobelOutputImagePtr);

    // LATER: Do not consider extended portion
    float normalizingRatio = 255.0/(sobelLimits[0]-sobelLimits[1]);
    for (int row = 0; row < edges->height(); row++) {
        for (int col = 0; col < edges->width(); col++) {

            if (row < height_orig) {
                double rg = sqrt((*ptrHorz ) * (*ptrHorz ) + (*ptrVert ) * (*ptrVert ))*0.707106781;
                
                if(sobelIsNormalized < SOBEL_FLICKER){
                    sobelLimits[0] = sobelLimits[0]<rg?rg:sobelLimits[0];   // max
                    sobelLimits[1] = sobelLimits[1]>rg?rg:sobelLimits[1];   //min
                    *pedges = (uchar)(255*rg);
                    
                }
                else {
                    *pedges = (uchar)(normalizingRatio*(rg-sobelLimits[1]));
                }
            }
            else
                *pedges = 0;
            
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
    cvShowImage("Edges", (IplImage*)edges->getIplImage());
#endif
    
    
}




void earlyVisionThread::maxImages(IplImage** ImagesTobeAdded, int numberOfImages,IplImage* resultantImage) {
    
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

void earlyVisionThread::addImages(IplImage** ImagesTobeAdded, int numberOfImages,IplImage* resImage, float* weights) {
    IplImage* resultantImage;
    cvSet(resImage,cvScalar(0));
    resultantImage = cvCreateImage(cvGetSize(ImagesTobeAdded[0]),IPL_DEPTH_32F,1); 
    int upperThreshold = 255;
    int lowerThreshold = -30;
    cvSet(resultantImage,cvScalar(0));
    float* resImageOrigin = (float*)resultantImage->imageData;
    int resImageWidth = resultantImage->widthStep/sizeof(float);
    float* tmpResultImage = (float*)resultantImage->imageData;
    for(int i=0; i< numberOfImages; ++i){
        IplImage* tmpImageTobeAdded = ImagesTobeAdded[i];
        if(tmpImageTobeAdded != NULL){   // save yourself some pain    
            uchar* tmpImage = (uchar*)tmpImageTobeAdded->imageData;
            //Images must be same size, type etc
            int h = tmpImageTobeAdded->height;
            int w = tmpImageTobeAdded->width;
            if(h > resultantImage->height || w > resultantImage->width) {
                printf("Image too big. \n");
                return ;
            }
            float itsWt = weights[i];
            uchar* imgToBeAddOrigin = (uchar*)tmpImageTobeAdded->imageData;
            int imgToBeAddWidth = tmpImageTobeAdded->widthStep;
            for(int j=0; j<h; ++j){
                tmpImage = imgToBeAddOrigin + j* imgToBeAddWidth; 
                tmpResultImage = resImageOrigin + j*resImageWidth;
                for(int k=0; k<w; ++k){
                    float valNow = (*tmpImage)* itsWt;
                    //if(*tmpResultImage < valNow) *tmpResultImage = valNow;
                    if((valNow + *tmpResultImage) > upperThreshold) *tmpResultImage = 255;
                    else if((valNow + *tmpResultImage) < lowerThreshold) *tmpResultImage = 0;
                    else *tmpResultImage = *tmpResultImage + valNow ;
                    *tmpImage++;
                    *tmpResultImage++;
                }
            }
        }
    }

    cvConvertScaleAbs(resultantImage,resImage,1.00,0.0);

    cvReleaseImage(&resultantImage);

}

void earlyVisionThread::cropImage(int* corners, IplImage* imageToBeCropped, IplImage* retImage){

    // very lame cropping
    int imgWidth = corners[2]-corners[0];
    int imgHeight = corners[3]-corners[1];
    
    cvSet(retImage,cvScalar(0));

    
    uchar* originDestImg = (uchar*)retImage->imageData;
    uchar* originSourImg = (uchar*)imageToBeCropped->imageData;
    int widthDest = retImage->widthStep;
    int widthSour = imageToBeCropped->widthStep;
    uchar* sourceRow = originSourImg;
    uchar* destRow = originDestImg;
    size_t stride = imgWidth*sizeof(uchar);
    for(int i = 0; i<imgHeight; ++i){
        
        memcpy((originDestImg+ i * widthDest),(originSourImg + (i+corners[1])*widthSour + corners[0]),stride);
    } 
    return ;
    
}

void earlyVisionThread::cropCircleImage(int* center, float radius, IplImage* srcImg) {
    radius -= 4;
    for(int i=0; i< srcImg->height; ++i){
        for(int j=0; j< srcImg->width; ++j){
            if((i - center[0])*(i - center[0]) + (j - center[1])*(j - center[1]) >= radius*radius) {
                *(srcImg->imageData + i*srcImg->widthStep + j) = 0; //blacken the pixel out of circle
            }
        }
    }
}



void earlyVisionThread::threadRelease() {
    

    trsf.freeLookupTables();
    lpMono.freeLookupTables();

    cvReleaseImage(&cs_tot_32f); 
    cvReleaseImage(&colcs_out);
    cvReleaseImage(&ycs_out);
    cvReleaseImage(&scs_out);
    cvReleaseImage(&vcs_out);
    

    
    
    printf("Release complete!\n");
    resized = false;    
    
}

void earlyVisionThread::onStop() {

    printf("calling on-stop\n");

    imagePortIn.interrupt();
    intenPort.interrupt();
    chromPort.interrupt();
    edgesPort.interrupt();
    orientPort0.interrupt();
    orientPort45.interrupt();
    orientPort90.interrupt();
    orientPortM45.interrupt();
    colorOpp1Port.interrupt();
    colorOpp2Port.interrupt();
    colorOpp3Port.interrupt();
    
    
    imagePortIn.close();
    intenPort.close();
    chromPort.close();
    edgesPort.close();
    orientPort0.close();
    orientPort45.close();
    orientPort90.close();
    orientPortM45.close();
    colorOpp1Port.close();
    colorOpp2Port.close();
    colorOpp3Port.close();
    
    printf("done with on-stop\n");
    
}


