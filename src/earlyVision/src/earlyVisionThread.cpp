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
    

    gaborPosHorConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,0,.5,0);
    gaborPosVerConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,1,.5,0);
    gaborNegHorConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,0,.5,0);
    gaborNegVerConvolution = new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,1,.5,0);
    
    gaborFiveByFive[0] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,Gab0,1.0,0);
    gaborFiveByFive[1] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,Gab45,1.0,0);
    gaborFiveByFive[2] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,Gab90,1.0,0);
    gaborFiveByFive[3] = new convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >(5,5,GabM45,1.0,0);  

    
    lambda  = 0.3f;
    resized = false;
    isYUV   = true;
}

earlyVisionThread::~earlyVisionThread() {
    printf("earlyVisionThread::~earlyVisionThread() \n");      
}

bool earlyVisionThread::threadInit() {
    printf("opening ports by main thread\n");

    chromeThread = new chrominanceThread();
    chromeThread->setName(getName("/chrome").c_str());
    chromeThread->setWHorizontal(wHorizontal);
    chromeThread->setWVertical(wVertical);
    chromeThread->setW45Degrees(w45Degrees);
    chromeThread->setWM45Degrees(wM45Degrees);
    chromeThread->start();

    edThread = new edgesThread();
    edThread->setName(getName("/edges").c_str());
    edThread->start();

    /* open ports */     
   
    if (!imagePortIn.open(getName("/imageRGB:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    if (!imagePortOut.open(getName("/imageRGB:o").c_str())) {
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
    
    //initializing logpolar mapping
    cout << "||| initializing the logpolar mapping" << endl;
    

    if (!lpTrans.allocLookupTables(BOTH, COL_SIZE, ROW_SIZE, CART_ROW_SIZE, CART_COL_SIZE, 1.0)) {
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
     
        
                
        inputImage  = imagePortIn.read(false);
        /*IplImage *imgRGB;
        imgRGB = cvLoadImage("logPVert.jpg");
        inputImage->resize(imgRGB->width,imgRGB->height);
        inputImage->zero();
        cvAdd(imgRGB,(IplImage*)inputImage->getIplImage(),(IplImage*)inputImage->getIplImage());
        cvNamedWindow("image");
        cvShowImage("image",(IplImage*)inputImage->getIplImage());
        //cvWaitKey(0);*/
        
        
        
        if (inputImage != NULL) {
            if (!resized) {
                resize(inputImage->width(), inputImage->height());
                filteredInputImage->zero(); 
                resized = true;
            }
            if((inputImage!=0)&&(imagePortOut.getOutputCount())) {
                imagePortOut.prepare() = *(inputImage);
                imagePortOut.write();
            }
            filterInputImage();
            
            extender(maxKernelSize);
             //printf("red plus dimension in resize3  %d %d \n", cvRedPlus->width, cvRedPlus->height);
            
            // extract RGB and Y planes
            extractPlanes();
             //printf("red plus dimension in resize4  %d %d \n", cvRedPlus->width, cvRedPlus->height);
               
            centerSurrounding();
            //edgesExtract();  
                      
            // gaussian filtering of the of RGB and Y
            filtering();           

            // colourOpponency map construction
            colorOpponency();         

                              

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

    YofYUVpy->resize(width_orig, height_orig);
    UofYUVpy->resize(width_orig, height_orig);
    VofYUVpy->resize(width_orig, height_orig);
    
    // Note, resizing 
    RplusUnex->resize(width_orig, height_orig);
    GplusUnex->resize(width_orig, height_orig);
    BplusUnex->resize(width_orig, height_orig);
    
    
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
            // assuming order of color channels is R,G,B. For some format it could be B,G,R.
            *shift[0] = *inputPointer++;
            *shift[1] = *inputPointer++;
            *shift[2] = *inputPointer++;

            *shift[3]++ = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));
            *ptrIntensityImg = ONE_BY_ROOT_THREE * sqrt(*shift[0] * *shift[0] +*shift[1] * *shift[1] +*shift[2] * *shift[2]);
            

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

    if(!chromeThread->getFlagForThreadProcessing()){
        //chromeThread->copyRelevantPlanes(unXtnIntensImg);
    }

    if(!edThread->getFlagForThreadProcessing()){
        edThread->copyRelevantPlanes(intensImg);
    }
    
    

}

void earlyVisionThread::filtering() {
    // We gaussian blur the image planes extracted before, one with positive Gaussian and then negative
    
    //Positive
    // This is calculated via first scale of YUV planes
    /*tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(redPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Rplus);
    

    tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(redPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Bplus);

    tmpMonoLPImage->zero();
    gaborPosHorConvolution->convolve1D(greenPlane,tmpMonoLPImage);
    gaborPosVerConvolution->convolve1D(tmpMonoLPImage,Gplus);*/  
    
    
       
    
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
    cvWaitKey(2);
#endif

}

void earlyVisionThread::centerSurrounding(){

        
        
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
                                
        
        if(true){
                //performs centre-surround uniqueness analysis on first plane                
                centerSurr->proc_im_8u( (IplImage*)unXtnYplane->getIplImage(),(IplImage*)_Y.getIplImage());
                cvConvertScale(centerSurr->get_pyramid_gauss(0),(IplImage*)YofYUVpy->getIplImage(),255,0);
                
                // Send it to other thread
                CvMat* toSend[GABOR_SCALES], *toSendGauss[GABOR_SCALES];
                for(int i=0; i<GABOR_SCALES; ++i){
                    //toSend[i] = centerSurr->get_pyramid_gauss(i);
                    toSendGauss[i] = centerSurr->get_gauss(i);
                }
                chromeThread->setFlagForDataReady(false);
                chromeThread->copyScalesOfImages(unXtnIntensImg,toSendGauss);
                //while(!chromeThread->getFlagForDataReady()) {};
                
                
                
                
                
                cvSet(cs_tot_32f,cvScalar(0));                
                if (isYUV){               
                    //performs centre-surround uniqueness analysis on second plane:
                    centerSurr->proc_im_8u( (IplImage*)(unXtnUplane->getIplImage()),scs_out);
                    cvAdd(centerSurr->get_centsur_32f(),cs_tot_32f,cs_tot_32f); // in place?
                    cvConvertScale(centerSurr->get_pyramid_gauss(0),(IplImage*)UofYUVpy->getIplImage(),255,0);
                    
                    //Colour process V:performs centre-surround uniqueness analysis:
                    centerSurr->proc_im_8u( (IplImage*)(this->unXtnVplane->getIplImage()), vcs_out);
                    cvAdd(centerSurr->get_centsur_32f(),cs_tot_32f,cs_tot_32f);
                    cvConvertScale(centerSurr->get_pyramid_gauss(0),(IplImage*)VofYUVpy->getIplImage(),255,0);          
                    
                    //get min max   
                    double valueMin = 1000;
                    double valueMax = -1000;
                    //img_UV->zero();     // this is not strictly required
                  	cvMinMaxLoc(cs_tot_32f,&valueMin,&valueMax);            
                    if ( valueMax == valueMin || valueMin < -1000 || valueMax > 1000){ 
                        valueMax = 255.0f; valueMin = 0.0f;
                    }
                    cvConvertScale(cs_tot_32f,(IplImage*)_UV.getIplImage(),255/(valueMax - valueMin),-255*valueMin/(valueMax-valueMin)); //LATER
                    
                    // calculate RGB from YUV uchar planes
                    uchar* ptrYplane = (uchar*)YofYUVpy->getRawImage();
                    uchar* ptrUplane = (uchar*)UofYUVpy->getRawImage();
                    uchar* ptrVplane = (uchar*)VofYUVpy->getRawImage();
                    uchar* ptrRplane = (uchar*)RplusUnex->getRawImage();
                    uchar* ptrGplane = (uchar*)GplusUnex->getRawImage();
                    uchar* ptrBplane = (uchar*)BplusUnex->getRawImage();
                    int padImage = YofYUVpy->getPadding();
                    int red, green, blue;
                    int htYUV = YofYUVpy->height();
                    int wdYUV = YofYUVpy->width();
                    for(int i=0 ; i< htYUV; ++i){
                        for(int j=0; j< wdYUV; ++j){
                            // should use bit-wise ops?
                            red = *ptrYplane + 1.403* *ptrVplane;
                            green = *ptrYplane - 0.344* *ptrUplane - 0.714* *ptrVplane;
                            blue = *ptrYplane + 1.770* *ptrUplane;
                            
                            *ptrRplane++ = max(0,min(255,red));
                            *ptrGplane++ = max(0,min(255,green));
                            *ptrBplane++ = max(0,min(255,blue));
                            ptrYplane++;
                            ptrUplane++;
                            ptrVplane++;
                        }
                        ptrRplane += padImage;
                        ptrGplane += padImage;
                        ptrBplane += padImage;
                        ptrYplane += padImage;
                        ptrUplane += padImage;
                        ptrVplane += padImage;
                        
                   }          
                    
                    
                    iCub::logpolar::replicateBorderLogpolar(*Rplus, *RplusUnex, maxKernelSize);  
                    iCub::logpolar::replicateBorderLogpolar(*Gplus, *GplusUnex, maxKernelSize);
                    iCub::logpolar::replicateBorderLogpolar(*Bplus, *BplusUnex, maxKernelSize);
                    
                    #ifdef DEBUG_OPENCV
                    cvNamedWindow("YofYUVpy");
                    cvShowImage("YofYUVpy",(IplImage*)YofYUVpy->getIplImage());
                    cvNamedWindow("UofYUVpy");
                    cvShowImage("UofYUVpy",(IplImage*)UofYUVpy->getIplImage());
                    cvNamedWindow("VofYUVpy");
                    cvShowImage("VofYUVpy",(IplImage*)VofYUVpy->getIplImage());
                    cvNamedWindow("redPlane");
                    cvShowImage("redPlane", (IplImage*)redPlane->getIplImage());
                    cvNamedWindow("greenPlane");
                    cvShowImage("greenPlane", (IplImage*)greenPlane->getIplImage());
                    cvNamedWindow("bluePlane");
                    cvShowImage("bluePlane", (IplImage*)bluePlane->getIplImage());
                    cvWaitKey(2);
                    #endif
                    
                    //cvConvertScale(cs_tot_32f,(IplImage*)img_UV->getIplImage(),255,0);                   
                    
                }
                else{
                    //performs centre-surround uniqueness analysis on second plane:
                    centerSurr->proc_im_8u( (IplImage*)(this->unXtnUplane->getIplImage()),(IplImage*)_UV.getIplImage() );

                    //Colour process V:performs centre-surround uniqueness analysis:
                    centerSurr->proc_im_8u( (IplImage*)(this->unXtnVplane->getIplImage()), (IplImage*)_V.getIplImage());
                }                   
             
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

void earlyVisionThread::addFloatImage(IplImage* sourceImage, CvMat* cvMatAdded, double multFactor, double shiftFactor){

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


void earlyVisionThread::threadRelease() {    
    

    resized = false;
    
    printf("----Releasing earlyVision thread ... \n");
    edThread->stop();
    printf("----Releasing chrome thread ... \n");
    chromeThread->stop();    
    printf("-----Chrome thread correctly closed \n");

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
    delete YofYUVpy;
    delete UofYUVpy;
    delete VofYUVpy;
    delete RplusUnex;
    delete GplusUnex;
    delete BplusUnex;
    
    printf("correctly deleting the images \n");
    imagePortIn.interrupt();
    imagePortOut.interrupt();
    intenPort.interrupt();
    imagePortIn.close();
    imagePortOut.close();
    intenPort.close();

    colorOpp1Port.interrupt();
    colorOpp2Port.interrupt();
    colorOpp3Port.interrupt();
    
    colorOpp1Port.close();
    colorOpp2Port.close();
    colorOpp3Port.close();
 
    printf("Done with releasing earlyVision thread.\n");
    
}




