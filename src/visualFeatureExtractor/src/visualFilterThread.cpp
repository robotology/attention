// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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
 * @file visualFilterThread.cpp
 * @brief Implementation of the visual filter thread (see visualFilterThread.h).
 */

#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/visualFilterThread.h>
#include <cstring>

#define ONE_BY_ROOT_TWO 0.707106781

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;

const int maxKernelSize = 5;

template<class T>
inline T max(T a, T b, T c) {    
    if(b > a) a = b;
    if(c > a) a = c;
    return a;
}

visualFilterThread::visualFilterThread() {
    
    inputExtImage = new ImageOf<PixelRgb>;
    inputImageFiltered = new ImageOf<PixelRgb>;
    
    edges = new ImageOf<PixelMono>;
    
    lambda = 0.1f;

    resized = false;
}

visualFilterThread::~visualFilterThread() {
    
    delete inputExtImage;
    delete inputImageFiltered;
    delete inputImage;

    
    delete edges;

    //if(kernel!=0)
        //cvReleaseMatHeader(kernel);
}

bool visualFilterThread::threadInit() {
    /* open ports */ 
    if (!imagePortIn.open(getName("/image:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imagePortOut.open(getName("/image:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imagePortExt.open(getName("/imageExt:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!rgPort.open(getName("/rg:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!grPort.open(getName("/gr:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!byPort.open(getName("/by:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    return true;
}

void visualFilterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string visualFilterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void visualFilterThread::run() {

    //cvNamedWindow( "test1", 1);  
    //cvNamedWindow( "test2", 1);
    //cvNamedWindow( "test3", 1); 

    while (isStopping() != true) {
        inputImage = imagePortIn.read(true);

        if (inputImage != NULL) {
            if (!resized) {
                resize(inputImage->width(), inputImage->height());
                resized = true;
            }
            else {
                filterInputImage();
            }
          
            // extend logpolar input image
            extender(inputImage, maxKernelSize);
            // extract RGB and Y planes
            extractPlanes();

            
            // gaussian filtering of the of RGB and Y
            filtering();
            // colourOpponency map construction
            colourOpponency();
            // apply sobel operators on the colourOpponency maps and combine via maximisation of the 3 edges
            edgesExtract();
            //cvShowImage( "test", hRG); cvWaitKey(0);           
            
            // sending the edge image on the outport            
                 
            // the copy to the port object can be avoided...
            if((edges!=0)&&(imagePortOut.getOutputCount())) {
                imagePortOut.prepare() = *(edges);
                imagePortOut.write();
            }
            if((redGreen!=0)&&(rgPort.getOutputCount())) {
                rgPort.prepare() = *(redGreen);
                rgPort.write();
            }
            if((greenRed!=0)&&(grPort.getOutputCount())) {
                grPort.prepare() = *(greenRed);
                grPort.write();
            }
            if((blueYellow!=0)&&(byPort.getOutputCount())) {
                byPort.prepare() = *(blueYellow);
                byPort.write();
            }
            if((inputExtImage!=0)&&(imagePortExt.getOutputCount())) {
                imagePortExt.prepare() = *(inputExtImage);
                imagePortExt.write();
            }
        }
   }
}

void visualFilterThread::resize(int width_orig,int height_orig) {


    this->width_orig = width_orig;
    this->height_orig = height_orig;
    this->width = width_orig+2*maxKernelSize;
    this->height = height_orig+maxKernelSize;

    
    edges->resize(width_orig, height_orig);
    inputImageFiltered->resize(width_orig, height_orig);
    inputImageFiltered->zero();
 
    inputExtImage->resize(width,height);


    //allocate IplImages for color planes
    cvRedPlane = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    cvGreenPlane = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    cvBluePlane = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    cvYellowPlane = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    //allocate IplImages for color opponents
    redG = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    greenR = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    blueY = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    //allocate IplImages for positive and negative gaussian convolution on image planes
    cvRedMinus = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    cvRedPlus = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    cvGreenMinus = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    cvGreenPlus = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    cvYellowMinus = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    cvBluePlus = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    
    
    //allocate IplImages for horizontal and vertical components of color opponents (after Sobel operator is applied)
    hRG = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    vRG = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    hGR = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    vGR = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    hBY = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    vBY = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    //allocate temporary 16 bit deep IplImages for Sobel operator result
    tempHRG = cvCreateImage(cvSize(width,height),IPL_DEPTH_16S, 1 );
    tempVRG = cvCreateImage(cvSize(width,height),IPL_DEPTH_16S, 1 );
    tempHGR = cvCreateImage(cvSize(width,height),IPL_DEPTH_16S, 1 );
    tempVGR = cvCreateImage(cvSize(width,height),IPL_DEPTH_16S, 1 );
    tempHBY = cvCreateImage(cvSize(width,height),IPL_DEPTH_16S, 1 );
    tempVBY = cvCreateImage(cvSize(width,height),IPL_DEPTH_16S, 1 );

    // 7x7 kernel for negative gaussian 
    float K[] = {
        0.0113f, 0.0149f, 0.0176f, 0.0186f, 0.0176f, 0.0149f, 0.0113f,
        0.0149f, 0.0197f, 0.0233f, 0.0246f, 0.0233f, 0.0197f, 0.0149f,
        0.0176f, 0.0233f, 0.0275f, 0.0290f, 0.0275f, 0.0233f, 0.0176f,
        0.0186f, 0.0246f, 0.0290f, 0.0307f, 0.0290f, 0.0246f, 0.0186f,
        0.0176f, 0.0233f, 0.0275f, 0.0290f, 0.0275f, 0.0233f, 0.0176f,
        0.0149f, 0.0197f, 0.0233f, 0.0246f, 0.0233f, 0.0197f, 0.0149f,
        0.0113f, 0.0149f, 0.0176f, 0.0186f, 0.0176f, 0.0149f, 0.0113f
    };

    kernel = cvCreateMat( 7, 7, CV_32FC1 );
    cvSetData ( kernel, (float*)K, sizeof ( float ) * 7 );
    //cvInitMatHeader(kernel,7 ,7, CV_32FC1, K);

    minVal = 256;

}

void visualFilterThread::filterInputImage() {
    int i;
    const int sz = inputImage->getRawImageSize();
    unsigned char * pFiltered = inputImageFiltered->getRawImage();
    unsigned char * pCurr = inputImageFiltered->getRawImage();
    const float ul = 1.0f - lambda;
    for (i = 0; i < sz; i++) {
        *pFiltered = (unsigned char)(lambda * *pCurr++ + ul * *pFiltered + .5f);
        pFiltered ++;
    }
}

ImageOf<PixelRgb>* visualFilterThread::extender(ImageOf<PixelRgb>* inputOrigImage, int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*inputExtImage, *inputOrigImage, maxSize);
    return inputExtImage;
}

void visualFilterThread::extractPlanes() {
    // We extract color planes from the RGB image. Planes are red,blue, green and yellow (AM of red & green)
    uchar* shift[3];
    uchar* yellowP;
    
    // Pointers to raw openCV monochrome image
    shift[0] = (uchar*) cvRedPlane->imageData; 
    shift[1] = (uchar*)cvGreenPlane->imageData;
    shift[2] = (uchar*)cvBluePlane->imageData;
    yellowP = (uchar*)cvYellowPlane->imageData;

    // Pointer to raw extended input (RGB) image
    unsigned char* inputPointer = inputExtImage->getRawImage();

    /* We can avoid padding for openCV images*/
    int paddingMono = inputExtImage->getPadding(); 
    int padding3C = inputExtImage->getPadding(); 

    const int h = inputExtImage->height();
    const int w = inputExtImage->width();

    for(int r = 0; r < h; r++) {
        for(int c = 0; c < w; c++) {
            *shift[0] = *inputPointer++;
            *shift[1] = *inputPointer++;
            *shift[2] = *inputPointer++;

            *yellowP++ = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));

            shift[0]++;
            shift[1]++;
            shift[2]++;
        }

        inputPointer += padding3C;
        shift[0] += paddingMono;
        shift[1] += paddingMono;
        shift[2] += paddingMono;
        yellowP += paddingMono;
    }

}

void visualFilterThread::filtering() {




    // We gaussian blur the image planes extracted before, one with positive Gaussian and then negative

    //Positive
    cvSmooth( cvRedPlane, cvRedPlus, CV_GAUSSIAN, 5, 5 );
    cvSmooth( cvBluePlane, cvBluePlus, CV_GAUSSIAN, 5, 5 );
    cvSmooth( cvGreenPlane, cvGreenPlus, CV_GAUSSIAN, 5, 5 );


    cvSmooth( cvRedPlane, cvRedMinus, CV_GAUSSIAN, 7, 7,3 );
    cvSmooth( cvYellowPlane, cvYellowMinus, CV_GAUSSIAN, 7, 7,3 );
    cvSmooth( cvGreenPlane, cvGreenMinus, CV_GAUSSIAN, 7, 7,3 );
    
/*
    //Negative
    cvFilter2D(cvRedPlane, cvRedMinus, kernel, cvPoint(-1,-1));
    cvFilter2D(cvYellowPlane, cvYellowMinus, kernel, cvPoint(3,3));
    cvFilter2D(cvGreenPlane, cvGreenMinus, kernel, cvPoint(3,3));


    //cvShowImage("test3",&kernel);
    //IplImage* tem;
    //tem = cvCreateImage(cvGetSize(cvRedPlane),32,1);
    cvSmooth(cvRedPlane, cvRedMinus, CV_GAUSSIAN,7,7,3);
*/
    //cvShowImage("test2",cvRedMinus);
    //IplImage* sI;
    //sI = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    //cvSub(cvRedPlus,cvGreenMinus, sI);
    

 

}

void visualFilterThread::colourOpponency() {
    
    // we want RG = (G- - R+)/2 -128, GR = (R- - G+)/2 and BY = (Y- -B+)/2 -128. These values are obtained after filtering with positive and negative Gaussian.

    const int h = height;
    const int w = width;
    int pad = inputExtImage->getPadding();

    uchar* rMinus = (uchar*)cvRedMinus->imageData;
    uchar* rPlus = (uchar*)cvRedPlus->imageData;

    uchar* gMinus = (uchar*)cvGreenMinus->imageData;
    uchar* gPlus = (uchar*)cvGreenPlus->imageData;;

    uchar* yMinus = (uchar*)cvYellowMinus->imageData;
    uchar* bPlus = (uchar*)cvBluePlus->imageData;

    uchar* RG = (uchar*)redG->imageData;
    uchar* GR = (uchar*)greenR->imageData;
    uchar* BY = (uchar*)blueY->imageData;

    for(int r = 0; r < h; r++) {
        for(int c = 0; c < w; c++) {
            
            *RG++ = ((*rPlus >> 1)  + 128 - (*gMinus >> 1) );
            *GR++ = ((*gPlus >> 1) + 128 - (*rMinus >> 1) );
            *BY++ = ((*bPlus >> 1) + 128 - (*yMinus >> 1) );

            rMinus++;
            rPlus++;
            gMinus++;
            gPlus++;
            yMinus++;
            bPlus++;
        }

        rMinus += pad;
        rPlus += pad;
        gMinus += pad;
        gPlus += pad;
        yMinus += pad;
        bPlus += pad;
        RG += pad;
        GR += pad;
        BY += pad;

    } 

    //REMOVE
    //cvShowImage("test1",blueY);
    //cvShowImage("test2",redG);
    //cvShowImage("test3",greenR);
    //cvWaitKey(0);  
  
}



void visualFilterThread::edgesExtract() {

    
    // Since Sobel doesnt do scaling, destination is 16 bit
    cvSobel(redG,tempHRG,1,0,3);   
    cvSobel(redG,tempVRG,0,1,3);
    cvSobel(greenR,tempHGR,1,0,3);
    cvSobel(greenR,tempVGR,0,1,3);
    cvSobel(blueY,tempHBY,1,0,3);
    cvSobel(blueY,tempVBY,0,1,3);

    short* ptrtempHRG;
    char* temphRG;
    short maxVal = -256;
   temphRG = hRG->imageData;
    minVal = 257; 

            


    // Scaling the destination back to 8 bit
    cvConvertScaleAbs(tempHRG,hRG,1.00,0.0);
    cvConvertScaleAbs(tempVRG,vRG,1.00,0.0);
    cvConvertScaleAbs(tempHGR,hGR,1.00,0.0);
    cvConvertScaleAbs(tempVGR,vGR,1.00,0.0);
    cvConvertScaleAbs(tempHBY,hBY,1.00,0.0);
    cvConvertScaleAbs(tempVBY,vBY,1.00,0.0);
    ptrtempHRG = (short*)tempHRG->imageData;
    maxVal = 256;
    for(int i = 0; i<width; ++i){
        for(int j=0; j<height; ++j){
            if(maxVal > *ptrtempHRG) maxVal = *ptrtempHRG; 
            ptrtempHRG++;
        }
    }
    //printf("Value of max %d \n",maxVal);
    //cvShowImage("test2",vBY);
    //cvWaitKey(0);
    

    //clearing up the previous value
    edges->zero();
    unsigned char* pedges=edges->getRawImage();
    const int pad_edges = inputImage->getPadding();
    

    //cvShowImage( "test1", hBY); cvWaitKey(0);

    uchar* ptrHRG = (uchar*)hRG->imageData;
    uchar* ptrVRG = (uchar*)vRG->imageData;
    uchar* ptrHGR = (uchar*)hGR->imageData;
    uchar* ptrVGR = (uchar*)vGR->imageData;
    uchar* ptrHBY = (uchar*)hBY->imageData;
    uchar* ptrVBY = (uchar*)vBY->imageData;

    //The extra portion of extended image can be neglected, hence the pointers are shifted accordingly. 
    int j = (width+1)*(height-height_orig)+ (width-width_orig); 

    // At the end of the original width, extended image pointers need to jump more 
    int gapInWidth = width - width_orig;
    const int padXtnd = gapInWidth + inputExtImage->getPadding();

    ptrHRG += j; ptrVRG += j;
    ptrHGR += j; ptrVGR += j;
    ptrHBY += j; ptrVBY += j;


    

    // edges extraction
    minVal = -256;
    for (int row = 0; row < height_orig; row++) {
        for (int col = 0; col < width_orig; col++) {

            //if((*ptrHRG) > minVal) {minVal = *ptrHRG; printf(" HRG is more than %d \n", minVal);}
            double rg = (*ptrHRG ) * (*ptrHRG ) + (*ptrVRG ) * (*ptrVRG );
            //printf("rg %f \t",sqrt(rg));
            double gr = (*ptrHGR ) * (*ptrHGR ) + (*ptrVGR) * (*ptrVGR);
            double by = (*ptrHBY) * (*ptrHBY ) + (*ptrVBY ) * (*ptrVBY );
            if (row < height_orig) {
                *pedges = (unsigned char)(sqrt(max<double> (rg, gr, by))*ONE_BY_ROOT_TWO); //*(255.0 / 1024)); //normalised with theoric max-response 1448.16, 362.03                
            }
            else
                *pedges = 0;
            
            pedges++;
            ptrHRG++; ptrVRG++;
            ptrHGR++; ptrVGR++;
            ptrHBY++; ptrVBY++;
        }
        // padding
        pedges += pad_edges;
        ptrHRG += padXtnd; ptrVRG += padXtnd;
        ptrHGR += padXtnd; ptrVGR += padXtnd;
        ptrHBY += padXtnd; ptrVBY += padXtnd;
    }
    
}

void visualFilterThread::threadRelease() {
    resized = false;
}

void visualFilterThread::onStop() {
    imagePortIn.interrupt();
    imagePortOut.interrupt();
    imagePortExt.interrupt();
    rgPort.interrupt();
    grPort.interrupt();
    byPort.interrupt();
    
    imagePortOut.close();
    imagePortExt.close();
    rgPort.close();
    grPort.close();
    byPort.close();
    imagePortIn.close();
}

