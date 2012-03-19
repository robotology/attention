// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file sacPlannerThread.cpp
 * @brief Implementation of the thread of prioritiser Collector(see header sacPlannerThread.h)
 */

#include <iCub/sacPlannerThread.h>
#include <cstring>

using namespace iCub::logpolar;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10
#define THCORR 0.99    // remember to change the counterpart in attPrioritiserThread
#define _shiftLevels 1
#define corrStep 10    // remember to change the counterpart in attPrioritiserThread

inline void copy_8u_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelSize();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

inline void copy_8u_C3R(ImageOf<PixelRgb>* src, ImageOf<PixelRgb>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelSize();
    //printf("channel of the input image %d \n", channels);
    int width = src->width();
    int height = src->height();
    //printf("dimension of the input image %d %d \n", width, height);
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            for (int j = 0; j < channels; j++) {
                *pdest++ = (unsigned char) *psrc++;
            }
        }            
        pdest += padding;
        psrc += padding;
    }
}

sacPlannerThread::sacPlannerThread() {
    
}

sacPlannerThread::sacPlannerThread(string moduleName) {
    printf("setting the name in the saccPLanner Thread \n");
    setName(moduleName);
}

sacPlannerThread::~sacPlannerThread() {
    
}


bool sacPlannerThread::threadInit() {
    idle = false;
    sleep = true;
    compare = false;
    direction = 0;
    countDirection = 0;
    printf("starting the thread.... \n");
    /* open ports */
    string rootName("");
    rootName.append(getName("/sacPlanner/img:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inImagePort.open(rootName.c_str());
    //inCommandPort.open(rootName.c_str());

    string rootNameCorr("");
    rootNameCorr.append(getName("/sacPlanner/corr:o"));
    printf("opening ports with rootname %s .... \n", rootNameCorr.c_str());
    corrPort.open(rootNameCorr.c_str());

    //initializing logpolar mapping
    cout << "||| initializing the logpolar mapping" << endl;
    int numberOfRings = 152;
    int numberOfAngles = 252;
    int xSizeValue = 320;
    int ySizeValue = 240;
    double overlap = 1.0;
    if (!trsfL2C.allocLookupTables(BOTH, numberOfRings, numberOfAngles, xSizeValue, ySizeValue, overlap)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "||| lookup table L2C allocation done" << endl;

    if (!trsfC2L.allocLookupTables(C2L, numberOfRings, numberOfAngles,xSizeValue, ySizeValue,  overlap)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "||| lookup table C2L allocation done" << endl;

    outputImageUp      = new ImageOf<PixelRgb>;
    outputImageDown    = new ImageOf<PixelRgb>;
    outputImageLeft    = new ImageOf<PixelRgb>;
    outputImageRight   = new ImageOf<PixelRgb>;
    predictedImage     = new ImageOf<PixelRgb>;

    return true;
}

void sacPlannerThread::interrupt() {
    //inCommandPort.interrupt();
}

void sacPlannerThread::setName(string str) {
    this->name = str;
    printf("name: %s", name.c_str());
}

std::string sacPlannerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void sacPlannerThread::setSaccadicTarget(int r, int t) {
    printf("setting Saccadic Planner ----- ");
    rho   = r;
    theta = t;
    printf("rho %d theta %d \n", rho, theta);
}

void sacPlannerThread::resizeImages(int logwidth, int logheight) {
    //
    predictedImage->resize(logwidth, logheight);
}

void sacPlannerThread::run() {
    ImageOf<PixelRgb>* intermImage  = new ImageOf<PixelRgb>;
    ImageOf<PixelRgb>* intermImage2 = new ImageOf<PixelRgb>;
    ImageOf<PixelRgb>& outputImage        = corrPort.prepare();               
    int width  = inputImage->width();
    int height = inputImage->height();
    resizeImages(width, height);
    outputImage.resize(320,240);
    outputImage.zero();
    printf("prepared the output port \n");  


    while(isStopping() != true){        
        //Bottle* b=inCommandPort.read(true);       
        //printf("sacPlanner::run");
        if(!idle) {
            inputImage = inImagePort.read(false);           
            // check whether it must be sleeping
            if((!sleep)&&(corrPort.getOutputCount())&&(inputImage!=NULL)) {
                printf("performing correlation measures \n");
                //here it comes if only if it is not sleeping
                        

                //!sleep and compare section
                mutex.wait();
                if((!sleep)&&(!compare)) {                    
                    mutex.post();
                    printf("in the sleep and compare branch \n");
                    
                    intermImage->resize(320,240);
                    intermImage2->resize(320,240);                                        
                    
                    outputImageUp->resize(width,height);
                    outputImageDown->resize(width,height);
                    outputImageLeft->resize(width,height);
                    outputImageRight->resize(width,height);
                    
                    int xPos = theta;
                    int yPos = rho;

                    //outputImage.copy(*predictedImage); 
                    //predictedImage->copy(outputImage); 
                    //printf("outputing the image \n");
                    //copy_8u_C3R(&outputImage, predictedImage);
                    //corrPort.write();
                    
                    
                    // preparing the set of predicted images
                    bool fromLogpolar = true;
                    if(fromLogpolar) {
                        trsfL2C.logpolarToCart(*intermImage,*inputImage);
                    }
                    else {
                        // we gain few pixel more but we don`t have the pure input logpolar image
                        copy_8u_C3R(inputImage, intermImage);
                    }


                    shiftROI(intermImage,intermImage2, xPos, yPos);
                    trsfL2C.cartToLogpolar(*predictedImage, *intermImage2);

                    //created alternative shifts
                    shiftROI(intermImage,intermImage2, xPos, yPos + corrStep);
                    trsfL2C.cartToLogpolar(*outputImageUp, *intermImage2);
                    
                    shiftROI(intermImage,intermImage2, xPos, yPos - corrStep);
                    trsfL2C.cartToLogpolar(*outputImageDown, *intermImage2);
                    
                    shiftROI(intermImage,intermImage2, xPos - corrStep, yPos);
                    trsfL2C.cartToLogpolar(*outputImageLeft, *intermImage2);
                    
                    shiftROI(intermImage,intermImage2, xPos + corrStep, yPos);
                    trsfL2C.cartToLogpolar(*outputImageRight, *intermImage2);    

                    //preparing the image output
                    copy_8u_C3R(intermImage, &outputImage);
                    //(img, center, radius, color, thickness=1, lineType=8, shift=0) → None
                    CvPoint p = cvPoint(10,10);
                    cvCircle(outputImage.getIplImage(),cvPoint(10,10),10,cvScalar(255,0,0));

                   
                    mutex.wait();
                    sleep   = true;
                    compare = false;
                    mutex.post();
                }
                else {
                    mutex.post();
                }
            
                //sleep and compare section
                //goes into the sleep mode waiting for the flag to be set by observable            
                printf("sleep %d compare %d \n" ,sleep, compare);
                if((!sleep)&&(compare)&&(corrPort.getOutputCount())&&(inputImage!=NULL)) {
                    //ImageOf<PixelRgb>& outputImage =  corrPort.prepare();
                    printf("Entering checkSleep with compare \n");
                    // it has been waken up by observable
                    // it compares the predictic pre-saccadic image with the post-saccadic image  
                    double* pCorr = new double;
                    //ImageOf<PixelRgb>* pOutputImage      = &outputImage;
                    //ImageOf<PixelRgb>* pOutputImageLeft  = &outputImageLeft;
                    //ImageOf<PixelRgb>* pOutputImageRight = &outputImageRight;
                    //ImageOf<PixelRgb>* pOutputImageUp    = &outputImageUp;
                    //ImageOf<PixelRgb>* pOutputImageDown  = &outputImageDown;
                    
                    
                    logCorrRgbSum(inputImage, predictedImage, pCorr,1);                    
                    corrValue = *pCorr;
                    printf("correlation between the predicted saccadic image with the actual %f \n", corrValue);
                    
                    
                    if(*pCorr < THCORR) {
                        // the saccadic planner triggers the error
                        // calculating the max correlation 
                        double *leftCorr  = new double;
                        double *rightCorr = new double;
                        double *upCorr    = new double;
                        double *downCorr  = new double;
                        double max = 0.0;
                        
                        logCorrRgbSum(inputImage, outputImageLeft, leftCorr ,1);
                        printf("saccadic correlation left %f \n", *leftCorr);
                        if(*leftCorr > max) {
                            max = *leftCorr;
                            direction = 180.0;
                            //countDirection++;
                            //direction /= countDirection;
                        }
                        logCorrRgbSum(inputImage, outputImageRight, rightCorr,1);
                        printf("saccadic correlation right %f \n", *rightCorr);
                        if(*rightCorr > max) {
                            max = *rightCorr;
                            direction = 0.0;
                            //direction += 180;
                            //countDirection++;
                            //direction /= countDirection;
                        }
                        logCorrRgbSum(inputImage, outputImageUp, upCorr,1);
                        printf("saccadic correlation up %f \n", *upCorr);
                        if(*upCorr > max) {
                            max = *upCorr;
                            direction = 90.0;
                            //direction += 90;
                            //countDirection++;
                            //direction /= countDirection;
                        }
                        logCorrRgbSum(inputImage, outputImageDown, downCorr,1);
                        printf("saccadic correlation down %f \n", *downCorr);
                        if(*downCorr > max) {
                            max = *downCorr;
                            direction = 270.0;
                            //direction += 270;
                            //countDirection++;
                            //direction /= countDirection;
                        }

                        delete leftCorr;
                        delete rightCorr;
                        delete upCorr;
                        delete downCorr;
                    }
                    else {
                        // the saccadic event has been successfully  performed
                        printf("saccadic event has been successfully performed \n");
                        direction = -1;                   
                    }
                    
                    mutex.wait();
                    compare = false;
                    sleep = true;
                    mutex.post();
                }
                corrPort.write();
            }
        } // end if idle
        //sending the image out
        //corrPort.prepare() = *inputImage;
        //corrPort.write();
        

        Time::delay(0.05);
    } //end of while
     delete intermImage;
     delete intermImage2;
}

void sacPlannerThread::referenceRetina(ImageOf<PixelRgb>* ref) {
    inputImage = ref;
}

void sacPlannerThread::shiftROI(ImageOf<PixelRgb>* inImg,ImageOf<PixelRgb>* outImg,int x, int y) {
    int width   = inImg->width();
    int height  = inImg->height(); 
    int centerX = width>>1;
    int centerY = height>>1;
    int dx = x - centerX;
    int dy = y - centerY;
    int channel = inImg->getPixelSize();

    unsigned char* pinput  = inImg->getRawImage();
    unsigned char* poutput = outImg->getRawImage();
    int padding = inImg->getPadding();
    int rowsize = inImg->getRowSize();
 
    int direction;
    if(dx > 0) {
        direction = -1;
        printf("jumping in x \n");
        pinput += channel * dx;
    }
    else {
        direction = 1;
    }
    
    if(dy > 0) {
        pinput += dy * rowsize;
    }
    for(int row = 0; row < height; row++) {        
        if((row + dy <= 0)||(row + dy >= height)){                        
            for(int col = 0; col < inImg->width(); col++) {
                //zero
                *poutput = (unsigned char) 0;
                poutput++;
                *poutput = (unsigned char) 0;
                poutput++;
                *poutput = (unsigned char) 0;
                poutput++;
            }
            poutput += padding;
        }        
        else {
            for(int col = 0; col < width; col++) {
                if((col + dx <= 0)||(col + dx > width)){
                    //zero
                    *poutput++ = (unsigned char) 0;
                    *poutput++ = (unsigned char) 0;
                    *poutput++ = (unsigned char) 0;
                }
                else {
                    //copying
                    *poutput = *pinput; //red
                    poutput++; pinput++;
                    *poutput = *pinput; //green
                    poutput++; pinput++;
                    *poutput = *pinput; //blue
                    poutput++; pinput++;
                }
            }

            poutput += padding;            
            pinput  += ( rowsize -  (width + dx * direction)  * channel + channel * direction);
        }        
    }        
}


void sacPlannerThread::logCorrRgbSum(ImageOf<PixelRgb>* imgA, ImageOf<PixelRgb>* imgB,double* pCorr, int step) {
    int _actRings   = imgA->height();
    int _sizeTheta  = imgA->width();
    if((_actRings!=imgB->height())||(_sizeTheta!=imgB->width())) {
        printf("ERROR : the dimension of the two images do not match! \n");
        return;
    }

    if((imgA==NULL)||(imgB==NULL)) {
        printf("NULL images \n");
    }
   

    unsigned char* aPtr = imgA->getRawImage();
    unsigned char* bPtr = imgB->getRawImage();
    int padding = imgA->getPadding();
    int _count[_shiftLevels];
    double _corrFunct[_shiftLevels];
    //pCorr = &_corrFunct;
    
    
    //printf("starting the cycle with %d shiftlevels \n", _shiftLevels);
    //Correlation Function Computation
    for (int k = 0; k < _shiftLevels; k++) {
        
        int iA, iB;

        double average_Ar = 0;
        double average_Ag = 0;
        double average_Ab = 0;
        double average_Br = 0;
        double average_Bg = 0;
        double average_Bb = 0;
        
        double R_corr = 0;
        double G_corr = 0;
        double B_corr = 0;
        
        double pixelA = 0;
        double pixelB = 0;
        
        // k1 = k *_img.Size_LP;  ???????
        
        if (_count[k] != 0) {
            average_Ar /= _count[k];
            average_Br /= _count[k];
            average_Ag /= _count[k];
            average_Bg /= _count[k];
            average_Ab /= _count[k];
            average_Bb /= _count[k];
        }

        double numr   = 0;
        double den_Ar = 0;
        double den_Br = 0;
        double numg   = 0;
        double den_Ag = 0;
        double den_Bg = 0;
        double numb   = 0;
        double den_Ab = 0;
        double den_Bb = 0;

        //for (int j = _actRings - 1; j >= 0; j-=step) {
        //    for (int i = _sizeTheta - 1; i >= 0; i-=step) {
                
        for (int j = 0 ; j < (_actRings>>1) ; j+=step) {
            for (int i = 0;i <  _sizeTheta ; i+=step) {

                //iR = _shiftMap[k1 + j*_img.Size_Theta + i];
                iA = 3 * (j * _sizeTheta + i); 
                iB = 3 * (j * _sizeTheta + i);
                //aPtr += iA;
                //bPtr += iB;

                if (iA > 0) {
                    //Red
                    pixelA = *aPtr - average_Ar;
                    pixelB = *bPtr - average_Br;
                    numr   += (pixelA * pixelB);
                    den_Ar += (pixelA * pixelA);
                    den_Br += (pixelB * pixelB);
                    aPtr++;bPtr++;
                    
                    //Green
                    pixelA = *aPtr - average_Ag;
                    pixelB = *bPtr - average_Bg;
                    numg   += (pixelA * pixelB);
                    den_Ag += (pixelA * pixelA);
                    den_Bg += (pixelB * pixelB);
                    aPtr++;bPtr++;

                    //Blue
                    pixelA = *aPtr - average_Ab;
                    pixelB = *bPtr - average_Bb;
                    numb   += (pixelA * pixelB);
                    den_Ab += (pixelA * pixelA);
                    den_Bb += (pixelB * pixelB);
                    aPtr++;bPtr++;
                }
            }
            aPtr += padding;
            bPtr += padding;            
        }

        //printf("RGB normalisation %f %f %f \n", numr, numg, numb);
        //printf("denominator %f %f %f \n", den_Ar, den_Ag, den_Ab);
        R_corr = numr / sqrt(den_Ar * den_Br + 0.00001);
        G_corr = numg / sqrt(den_Ag * den_Bg + 0.00001);
        B_corr = numb / sqrt(den_Ab * den_Bb + 0.00001);
        //printf("mean between %f %f %f \n",R_corr, G_corr, B_corr);
        _corrFunct[k] = (R_corr + G_corr + B_corr) / 3.0;
        //_corrFunct[k] *= _count[k];
        //printf("before dividing by 255 %f \n", _corrFunct[k]);
        //double _maxCount = 255.0;             // normalisation hard coded
        //_corrFunct[k] /= _maxCount;           // normalisation
        *pCorr = _corrFunct[k];
    }
    
}

void sacPlannerThread::onStop() {
    //inCommandPort.interrupt();
    //inCommandPort.close();
    corrPort.interrupt();
    inImagePort.interrupt();
    corrPort.close();
    inImagePort.close();
}

void sacPlannerThread::threadRelease() {
    delete outputImageUp;                                 
    delete outputImageDown;                          
    delete outputImageLeft;                    
    delete outputImageRight;
    delete predictedImage;
}


