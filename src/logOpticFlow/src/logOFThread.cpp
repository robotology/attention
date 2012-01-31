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
 * @file logOFThread.cpp
 * @brief Implementation of the early stage of vision thread (see logOFThread.h).
 */

#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/logOFThread.h>

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

inline void  copyImage(ImageOf<PixelRgb>* src,ImageOf<PixelRgb>* dest) {
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


inline void diff(ImageOf<PixelMono>* a,ImageOf<PixelMono>* b,ImageOf<PixelMono> *c) {
    unsigned char* ap = a->getRawImage();
    unsigned char* bp = b->getRawImage();
    //ImageOf<PixelMono> *c = new ImageOf<PixelMono> ;
    unsigned char* cp = c->getRawImage();
    int height  = a->height();
    int rowSize = a->getRowSize();
    int width   = a->width();
    int padding = a->getPadding();
    //c->resize(width, height);
    for (int row = 0; row < height; row++) {
        for(int col = 0; col < width; col++) {
            if(*ap - *bp > 0) 
                *cp = *ap - *bp;
            else
                *cp = *bp - *ap;
            ap++;
            bp++;
            cp++;
        }
        ap += padding;
        bp += padding;
        cp += padding;
    }
    
    //return c;
    //memcpy(destp,srcp,height * rowSize * sizeof(char));

}

logOFThread::logOFThread()/*:RateThread(RATE_OF_INTEN_THREAD)*/ {

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
    
    lambda  = 0.3f;
    resized = false;
    isYUV   = true;

}

logOFThread::~logOFThread() {
    printf("logOFThread::~logOFThread() \n");      
}

bool logOFThread::threadInit() {
    printf("opening ports by main thread\n");

    /* open ports */        
    if (!imagePortIn.open(getName("/imageRGB:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    if (!imagePortOut.open(getName("/imageRGB:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 
    if (!flowPort.open(getName("/flow:o").c_str())) {
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

    /* starting the threads */
    printf("creating the thread \n");
    pt = new plotterThread();
    pt->setName(this->getName("").c_str());
    pt->setReprPointer(flowImage);
    pt->start();


    return true;
}

void logOFThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string logOFThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void logOFThread::waitSemaphores(Semaphore** pointer) {
    Semaphore* p;
    for (int i = 0; i < COUNTCOMPUTERSX * COUNTCOMPUTERSY; i++) {
        p = pointer[i];
        p->wait();
    }
}

void logOFThread::postSemaphores(Semaphore** pointer) {
    Semaphore* p;
    for (int i = 0; i < COUNTCOMPUTERSX * COUNTCOMPUTERSY; i++) {
        p = pointer[i];
        p->post();
    }
}

void logOFThread::convertImages(ImageOf<PixelMono> *srcInt, ImageOf<PixelMono> *srcTmp){
    printf("converting Images \n");
    calculusIpl     = (IplImage*) srcInt->getIplImage();
    cvConvertScale(calculusIpl,calculusIpl32f,0.003922,0);  //  0.003922 = 1/255.0
    temporalIpl     = (IplImage*) srcTmp->getIplImage();
    cvConvertScale(temporalIpl,temporalIpl32f,0.003922,0);  //  0.003922 = 1/255.0
    printf("success in converting images \n");
}


void logOFThread::run() {   
 while(isRunning()) {
    inputImage  = imagePortIn.read(true);

    /*
    count++;
    //IplImage* img= cvCreateImage( cvSize(262, 157),IPL_DEPTH_8U,1); 
    if(count % 2 == 0){
        img = cvLoadImage("vertfinlog.jpg");
    }
    else {
        img = cvLoadImage("vertinlog.jpg");
    }
    if(!img) {
        printf("Could not load image file: \n");
    }
    else {
        printf("image correctly opened \n");
    }       
    inputImage->wrapIplImage(img);
    */
    

    
    if (inputImage != NULL) {
        if (!resized) {
            resize(inputImage->width(), inputImage->height());
            filteredInputImage->zero(); 
            resized = true;
        }            
        

        
        extender(maxKernelSize);
        //printf("wait before copying \n");
        // no need for semaphores this thread is the only thread
        copyImage(extendedInputImage, outputImage); 
        //printf("after way \n");
        
        waitSemaphores(tempSem);
        copyImage(intensImg,prevIntensImg);
        postSemaphores(tempSem);

        // extract RGB and Y planes
        extractPlanes();
        
        for (int i = 0; i < COUNTCOMPUTERSX * COUNTCOMPUTERSY; i++) {
            //printf("intensity image rowSize %d \n", intensImg->getRowSize());
            if(!ofComputer[i]->hasStarted()) {
                printf("the optic flow computer %d has not started \n", i);
                if((imagePortOut.getOutputCount()) && (imagePortIn.getInputCount())) {                       
                    printf("init flow computer %d \n", i);
                    if( (intensImg!=0) && (outputImage!=0) ) {
                        //printf("copying the intensImg before has started \n");
                        convertImages(intensImg,prevIntensImg);
            
                        waitSemaphores(calcSem);
                        cvCopy(calculusIpl32f,calculusIpl32f_copy); 
                        cvCopy(temporalIpl32f,temporalIpl32f_copy);
                        postSemaphores(calcSem);
                        
                        initFlowComputer(i);
                        ofComputer[i]->setHasStarted(true); 
                    }
                }
            }
        }
        
        
        //printf("red plus dimension in resize4  %d %d \n", cvRedPlus->width, cvRedPlus->height);
        
        //centerSurrounding();
        //edgesExtract();  
        
        // gaussian filtering of the of RGB and Y
        //filtering();           
        
        // colourOpponency map construction
        //colorOpponency();         
        
        
        if(intensImg!=0) {
            //waitSemaphores(tempSem);
            //copyImage(intensImg,prevIntensImg);
            //IplImage* imgbw =  cvCreateImage( cvSize(252, 152),IPL_DEPTH_8U,1);
            //img = cvLoadImage("vertinlog.jpg");
            //cvCvtColor(img,imgbw,CV_BGR2GRAY);
            
            //if(!imgbw) {
            //    printf("Could not load image file: \n");
            //}
            //prevIntensImg->wrapIplImage(imgbw);
            //postSemaphores(tempSem);
            convertImages(intensImg,prevIntensImg);
            waitSemaphores(calcSem);
            cvCopy(calculusIpl32f,calculusIpl32f_copy); 
            cvCopy(temporalIpl32f,temporalIpl32f_copy);
            postSemaphores(calcSem);

            
            //printf("copying once intensImage not null \n");
            //gradientHorConvolution->convolve2D(intensImg, intXgrad8u);
            
            //copyImage(intensImg,intensImgCopy);
                        
            //gradientVerConvolution->convolve2D(intensImg, intYgrad8u);
            //waitSemaphores(calcYSem);
            //copyImage(intensImg,gradientImgYCopy);
            //postSemaphores(calcYSem);
            
            //CvMat stub, *dst_mat, *grad_mat;
            //dst_mat = cvGetMat(intYgrad, &stub, 0, 0);
            
            
#ifdef DEBUG_OPENCV
            cvNamedWindow("ColorOppRG");
            cvShowImage("ColorOppRG", intYgrad);
#endif
            
            //IplImage stub, *dst_img;
            //dst_img = cvGetImage(src_mat, &stub);
                   
            if(intenPort.getOutputCount()) {
                ImageOf<PixelMono>* diffImage =new ImageOf<PixelMono>;
                diffImage->resize(width, height);
                diff(intensImg,prevIntensImg, diffImage);
                intenPort.prepare() = *diffImage;
                intenPort.write();
            }
        } 
        
        //outputImage->zero();
        waitSemaphores(reprSem);
        copyImage(outputImage, finalOutputImage);
        postSemaphores(reprSem);
        
        if((finalOutputImage!=0)&&(imagePortOut.getOutputCount())) {               
            imagePortOut.prepare() = *(finalOutputImage);                         
            imagePortOut.write();
        }
        
        if((flowImage!=0)&&(flowPort.getOutputCount())) {               
            flowPort.prepare() = *(flowImage);                         
            flowPort.write();
        }


#ifdef DEBUG_OPENCV
        cvWaitKey(0);
#endif           
    } // end if input image != NULL    
  } // end of while isRunning()
}



void logOFThread::resize(int width_orig,int height_orig) {

    this->width_orig  = inputImage->width(); //width_orig;
    this->height_orig = inputImage->height();//height_orig;
    
    width  = this->width_orig   + maxKernelSize * 2;
    height = this->height_orig  + maxKernelSize;
    printf("expressing width and height %d %d \n", width, height);


    pt->resize(width, height);
    
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
    calculusIpl    = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1); 
    temporalIpl    = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1); 
    calculusIpl32f = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
    temporalIpl32f = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1); 
    calculusIpl32f_copy = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1);
    temporalIpl32f_copy = cvCreateImage(cvSize(width,height),IPL_DEPTH_32F,1); 
    
    /* zeroing the images*/
    outputImage->zero();
    flowImage->zero();
    
    /* initialising the optic flow computers */
    printf("initialising the opticflow computers \n");
    for(int row = 0; row < COUNTCOMPUTERSY; row ++) {
        for(int col = 0; col < COUNTCOMPUTERSX; col ++) {
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
        }
    }    
}

void logOFThread::initFlowComputer(int index) {
    //printf("logOFThread::initFlowComputer \n");
    //printf("setting calculus %x pointer \n", intensImg->getRawImage());
    //ofComputer[index]->setCalculusPointer(gradientImgXCopy->getRawImage());
    //ofComputer[index]->setCalculusPointerY(gradientImgYCopy->getRawImage());
    //ofComputer[index]->setCalculusPointer(intensImgCopy->getRawImage());
    //ofComputer[index]->setCalculusRowSize(intensImgCopy->getRowSize());
    //ofComputer[index]->setCalculusImage(intensImgCopy);
    ofComputer[index]->setCalculusImageIpl(calculusIpl32f_copy);
    
    //printf("setting representation pointer %x %d \n", outputImage->getRawImage(), intensImg->getRowSize());
    ofComputer[index]->setRepresenPointer(flowImage->getRawImage());
    ofComputer[index]->setRepresenImage(flowImage);
    
    
    //printf("setting the image for temporal gradient \n");
    //ofComputer[index]->setTemporalPointer(prevIntensImg->getRawImage());
    //ofComputer[index]->setTemporalImage(prevIntensImg);
    ofComputer[index]->setTemporalImageIpl(temporalIpl32f_copy);
   
    
    //printf("setting semaphores \n");
    //ofComputer[index]->setCalculusXSem(calcXSem[index]);
    //ofComputer[index]->setCalculusYSem(calcYSem[index]);
    ofComputer[index]->setCalculusSem(calcSem[index]);
    ofComputer[index]->setRepresentSem(reprSem[index]);
    ofComputer[index]->setTemporalSem(tempSem[index]);
    
    //printf("adding the reference to the plotter"); 
    ofComputer[index]->setPlotterPointer(pt);
    pt->setReprPointer(flowImage);
}


void logOFThread::filterInputImage() {    
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


void logOFThread::extender(int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*extendedInputImage, *inputImage, maxSize);    
}

void logOFThread::extractPlanes() {

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

}

void logOFThread::filtering() {
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



void logOFThread::colorOpponency(){

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

void logOFThread::centerSurrounding(){        
        
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

void logOFThread::addFloatImage(IplImage* sourceImage, CvMat* cvMatAdded, double multFactor, double shiftFactor){

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

void logOFThread::onStop() {
    printf("logOFThread: thread releasing \n");
    imagePortIn.interrupt();
    imagePortOut.interrupt();
    flowPort.interrupt();
    intenPort.interrupt();
    intensityCSPort.interrupt();
    chromPort.interrupt();
    flowPort.close();
    imagePortIn.close();
    imagePortOut.close();

    intenPort.close();
    intensityCSPort.close();
    chromPort.close();

    colorOpp1Port.interrupt();
    colorOpp2Port.interrupt();
    colorOpp3Port.interrupt();
    colorOpp1Port.close();
    colorOpp2Port.close();
    colorOpp3Port.close();
    resized = false;    

    // deallocating resources
    delete inputImage;
    delete outputImage;
    delete flowImage;
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

    printf("correctly deleting the images \n");

    delete gaborPosHorConvolution;    
    delete gaborPosVerConvolution;    
    delete gaborNegHorConvolution;    
    delete gaborNegVerConvolution;
    delete YofYUV;
    delete intensImg;
    //delete intensXGrad;
    //delete intensYGrad;
    printf("correctly deleting second set of images \n");
    //delete intYgrad8u; 
    //delete intYgrad8u;

    
    //delete gradientImgXCopy;
    //delete gradientImgXCopy;
    delete intensImgCopy;
    delete prevIntensImg;
    delete unXtnIntensImg;
    printf("start to delete the third part \n");
    
    delete redPlane;
    delete greenPlane;
    delete bluePlane;
    delete yellowPlane;
    delete Yplane;
    delete Uplane;
    delete Vplane;

    printf("end of third part \n");
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
    
    printf("deleting semaphores \n");

    delete[] reprSem;
    delete[] tempSem;
    delete[] calcSem;
    //delete[] calcXSem;
    //delete[] calcYSem;

    printf("correctly freed memory of images \n");

    pt->stop();
    for(int j = 0 ; j < COUNTCOMPUTERSX * COUNTCOMPUTERSY; j++) {
        printf("stopping %d computer \n", j);        
        ofComputer[j]->stop();
    }       
 
    printf("Done with releasing earlyVision thread.\n");
}


void logOFThread::threadRelease() { 
   
    printf("logOFThread: thread releasing \n");
    imagePortIn.interrupt();
    imagePortOut.interrupt();
    flowPort.interrupt();
    intenPort.interrupt();
    intensityCSPort.interrupt();
    chromPort.interrupt();
    flowPort.close();
    imagePortIn.close();
    imagePortOut.close();

    intenPort.close();
    intensityCSPort.close();
    chromPort.close();

    colorOpp1Port.interrupt();
    colorOpp2Port.interrupt();
    colorOpp3Port.interrupt();
    colorOpp1Port.close();
    colorOpp2Port.close();
    colorOpp3Port.close();
    resized = false;    

    // deallocating resources
    delete inputImage;
    delete outputImage;
    delete flowImage;
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

    printf("correctly deleting the images \n");

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
    //delete[] calcXSem;
    //delete[] calcYSem;

    if(temporalIpl != 0) {
        cvReleaseImage(&temporalIpl);
        //delete temporalIpl;
    }
    if(calculusIpl!=0) {
        cvReleaseImage(&calculusIpl);
        //delete calculusIpl;
    }
    if(calculusIpl32f!=0) {
        cvReleaseImage(&calculusIpl32f);
        //delete calculusIpl32f;
    }
    if(temporalIpl32f!=0) {
        cvReleaseImage(&temporalIpl32f);
        //delete represenIpl;
    }
    if(calculusIpl32f_copy!=0) {
        cvReleaseImage(&calculusIpl32f_copy);
        //delete calculusIpl32f;
    }
    if(temporalIpl32f_copy!=0) {
        cvReleaseImage(&temporalIpl32f_copy);
        //delete represenIpl;
    } 

    printf("correctly freed memory of images \n");

    pt->stop();
    for(int j = 0 ; j < COUNTCOMPUTERSX * COUNTCOMPUTERSY; j++) {
        printf("stopping %d computer \n", j);        
        ofComputer[j]->stop();
    }       
 
    printf("Done with releasing earlyVision thread.\n");
    
}




