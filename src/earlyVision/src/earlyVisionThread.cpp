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



using namespace yarp::os;
using namespace yarp::sig;
using namespace std;


template<class T>
inline T max(T a, T b, T c) {    
    if(b > a) a = b;
    if(c > a) a = c;
    return a;
}

earlyVisionThread::earlyVisionThread():RateThread(RATE_OF_INTEN_THREAD) {
    
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
    
    YofYUV              = new ImageOf<PixelMono>;    
    intensImg           = new ImageOf<PixelMono>;
    unXtnIntensImg      = new ImageOf<PixelMono>;   
    
    redPlane            = new ImageOf<PixelMono>;
    greenPlane          = new ImageOf<PixelMono>;
    bluePlane           = new ImageOf<PixelMono>;
    yellowPlane         = new ImageOf<PixelMono>;

    Yplane            = new ImageOf<PixelMono>;
    Uplane            = new ImageOf<PixelMono>;
    Vplane            = new ImageOf<PixelMono>;
    
    unXtnYplane            = new ImageOf<PixelMono>;
    unXtnUplane            = new ImageOf<PixelMono>;
    unXtnVplane            = new ImageOf<PixelMono>;

    gaborPosHorConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,0,.5,0);
    gaborPosVerConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,1,.5,0);
    gaborNegHorConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,0,.5,0);
    gaborNegVerConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,1,.5,0);  

    
    lambda = 0.3f;
    resized = false;    
}

earlyVisionThread::~earlyVisionThread() {
    
    
       
}

bool earlyVisionThread::threadInit() {
    printf("opening ports by main thread\n");

    chromeThread = new chrominanceThread();
    chromeThread->setName(getName("/chrome").c_str());
    chromeThread->start();

    edThread = new edgesThread();
    edThread->setName(getName("/edges").c_str());
    edThread->start();

    /* open ports */ 
    
   
    if (!imagePortIn.open(getName("/imageRGB:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
   
    if (!intenPort.open(getName("/intensity:o").c_str())) {
        cout <<": unable to open port "  << endl;
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
     
        inputImage  = imagePortIn.read(false);
        /*IplImage *imgRGB;
        imgRGB = cvLoadImage("logPtemp.jpg");
        inputImage->resize(imgRGB->width,imgRGB->height);
        inputImage->zero();
        cvAdd(imgRGB,(IplImage*)inputImage->getIplImage(),(IplImage*)inputImage->getIplImage());
        cvWaitKey(2);
        cvReleaseImage(&imgRGB);
       */
        //cvNamedWindow("cnvt");
        //cvShowImage("cnvt",(IplImage*)inputImage->getIplImage());
        //cvWaitKey(0);
        
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

            // colourOpponency map construction
            colorOpponency();         

            centerSurrounding();
            //edgesExtract();                    

            if((intensImg!=0)&&(intenPort.getOutputCount())) {
                intenPort.prepare() = *(intensImg);
                intenPort.write();
            }

            
            /*
            if((Yplane!=0)&&(chromPort.getOutputCount())) {
                chromPort.prepare() = *(Yplane);
                chromPort.write();
                }
            */

            //if((edges!=0)&&(edgesPort.getOutputCount())) {
                //edgesPort.prepare() = *(edges);
                //edgesPort.write();
           // }
            
#ifdef DEBUG_OPENCV
            cvWaitKey(0);
#endif           
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
    //tmpMonoLPImageSobelHorz->resize(width, height);
    //tmpMonoLPImageSobelVert->resize(width, height);
    //tmpMonoSobelImage1->resize(width,height);
    //tmpMonoSobelImage2->resize(width,height);    

    //edges->resize(width, height);
    intensImg->resize(width, height);
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

    // allocating for CS ncsscale = 4;   

    cs_tot_32f  = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_32F, 1  );
    colcs_out   = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    ycs_out     = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    scs_out     = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    vcs_out     = cvCreateImage( cvSize(width_orig, height_orig),IPL_DEPTH_8U, 1  );
    
    
    centerSurr  = new CenterSurround( width_orig,height_orig,1.0 );

    
    isYUV = true; 
    
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
            *shift[0] = *inputPointer++;
            *shift[1] = *inputPointer++;
            *shift[2] = *inputPointer++;

            *shift[3]++ = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));
            *ptrIntensityImg = ONE_BY_ROOT_THREE * sqrt(*shift[0] * *shift[0] +*shift[1] * *shift[1] +*shift[2] * *shift[2]);
            

            // RGB to Y'UV conversion
            float red = (float)*shift[0];
            float green = (float)*shift[1];
            float blue = (float)*shift[2];

            *YUV[0] = 0.299*red + 0.587*green + 0.114*blue;
            *YUV[1] = (red-*YUV[0])*0.713 + 128.0;
            *YUV[2] = (blue-*YUV[0])*0.564 + 128.0;

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

    

    if(!chromeThread->getFlagForThreadProcessing()){
        chromeThread->copyRelevantPlanes(unXtnIntensImg);
    }

    if(!edThread->getFlagForThreadProcessing()){
        edThread->copyRelevantPlanes(unXtnIntensImg);
    }
    
    

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

void earlyVisionThread::centerSurrounding(){

        //printf("Going to centerSurrounding planes in chrome thread\n");
        
        // Allocate temporarily
        //ImageOf<PixelMono> _Y,_UV,_V;
        ImageOf<PixelMono>& _Y = intensityCSPort.prepare();
        _Y.resize(this->width_orig,this->height_orig);
        ImageOf<PixelMono>& _UV = chromPort.prepare();
        _UV.resize(this->width_orig,this->height_orig);
        
        ImageOf<PixelMono>& _V = VofHSVPort.prepare();
        _V.resize(this->width_orig,this->height_orig);
        //cvNamedWindow("test");
        //cvShowImage("test",(IplImage*)(cartIntensImg->getIplImage()));
        //cvWaitKey(0);
        
        if(true){
                //performs centre-surround uniqueness analysis on first plane
                
                centerSurr->proc_im_8u( (IplImage*)unXtnYplane->getIplImage(),(IplImage*)_Y.getIplImage());
                //cvNamedWindow("_Y");
                //cvShowImage("_Y",(IplImage*)unXtnYplane->getIplImage());
                //cvWaitKey(2);
                cvSet(cs_tot_32f,cvScalar(0));
                //cvNamedWindow("test");
                //cvShowImage("test",(IplImage*)(chromUplane->getIplImage()));
                //cvWaitKey(0);
                
                if (isYUV){                 
                
                    //performs centre-surround uniqueness analysis on second plane:
                    centerSurr->proc_im_8u( (IplImage*)(unXtnUplane->getIplImage()),scs_out);
                    cvAdd(centerSurr->get_centsur_32f(),cs_tot_32f,cs_tot_32f); // in place?                    
                    
                    //Colour process V:performs centre-surround uniqueness analysis:
                    centerSurr->proc_im_8u( (IplImage*)(this->unXtnVplane->getIplImage()), vcs_out);
                    cvAdd(centerSurr->get_centsur_32f(),cs_tot_32f,cs_tot_32f);                   
                    
                    //get min max   
                    double valueMin = 1000;
                    double valueMax = -1000;
                    //img_UV->zero();     // this is not strictly required
                  	cvMinMaxLoc(cs_tot_32f,&valueMin,&valueMax);            
                    if ( valueMax == valueMin || valueMin < -1000 || valueMax > 1000){ 
                        valueMax = 255.0f; valueMin = 0.0f;
                    }
                    cvConvertScale(cs_tot_32f,(IplImage*)_UV.getIplImage(),255/(valueMax - valueMin),-255*valueMin/(valueMax-valueMin)); //LATER
                    //cvConvertScale(cs_tot_32f,(IplImage*)img_UV->getIplImage(),255,0);                   
                    
                }
                else{
                    //performs centre-surround uniqueness analysis on second plane:
                    centerSurr->proc_im_8u( (IplImage*)(this->unXtnUplane->getIplImage()),(IplImage*)_UV.getIplImage() );
                    //Colour process V:performs centre-surround uniqueness analysis:
                    centerSurr->proc_im_8u( (IplImage*)(this->unXtnVplane->getIplImage()), (IplImage*)_V.getIplImage());
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
        
       
}

void earlyVisionThread::threadRelease() {    
    printf("Releasing\n");

    resized = false;

    // deallocating resources
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
    delete gaborPosHorConvolution;    
    delete gaborPosVerConvolution;    
    delete gaborNegHorConvolution;    
    delete gaborNegVerConvolution;
    delete YofYUV;
    delete intensImg;
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
   
    imagePortIn.interrupt();
    intenPort.interrupt();
    colorOpp1Port.interrupt();
    colorOpp2Port.interrupt();
    colorOpp3Port.interrupt();    
    
    imagePortIn.close();
    intenPort.close();
    colorOpp1Port.close();
    colorOpp2Port.close();
    colorOpp3Port.close();
    
    printf("done with release\n");
    
}




