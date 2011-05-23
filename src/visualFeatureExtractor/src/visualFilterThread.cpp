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
    inputImage = new ImageOf<PixelRgb>;
    inputImageFiltered = new ImageOf<PixelRgb>;
    logPolarImage = new ImageOf<PixelRgb>;
    pyImage = new ImageOf<PixelMono>;

    //getKernels(sigma, lambda, psi, gamma);
    sigma = 1.2;
    gLambda = 128;
    psi = 0;
    gamma = 30;
    kernelUsed = 2;
    dwnSam = 2;
    
    getKernels();
    kernelSize[0]=kernelSize[1]=5;      
    
    edges = new ImageOf<PixelMono>;
    
    lambda = 0.1f;

    resized = false;

    St = yarp::os::Stamp(0,0);
}

visualFilterThread::~visualFilterThread() {
    
    delete inputExtImage;
    delete inputImageFiltered;
    delete inputImage;   
    delete edges;
    delete pyImage;
    delete logPolarImage;
    for(int i=0; i<4; ++i){
        if(gabKer[i]!=0)
            cvReleaseMatHeader(&gabKer[i]);
    }
    printf("Calling destructor \n");
    //delete dwnImage;

    
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
    if (!pyImgPort.open(getName("/pyImg:o").c_str())) {
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

    double t1,t2,t3,t4,t5;
    long c1,c2;
    c1 = c2 = 0;
    t1=t2=t3=t4=t5=0;
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
            St.update();
            t1 = St.getTime();
            c1 = clock(); //LINUX only??
            // extend logpolar input image
            extender(inputImage, maxKernelSize);
            // extract RGB and Y planes
            extractPlanes();
            St.update();
            t2 = St.getTime();            
                      
            // gaussian filtering of the of RGB and Y
            filtering();
            St.update();
            t3 = St.getTime();
            // colourOpponency map construction
            colourOpponency();
            // apply sobel operators on the colourOpponency maps and combine via maximisation of the 3 edges
            St.update();
            t4 = St.getTime();
            edgesExtract();

            St.update();
            t5 = St.getTime();
            c2 = clock();
            //cvShowImage( "test", hRG); cvWaitKey(0);           
            
            // sending the edge image on the outport            
             printf("All delta times recorded %f, %f, %f, %f \n",t2-t1,t3-t2,t4-t3,t5-t1);
                printf("The time taken totally %lf \n",(c2-c1)/CLOCKS_PER_SEC);    
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
            if((pyImage!=0)&&(pyImgPort.getOutputCount())) {
                pyImgPort.prepare() = *(pyImage);
                pyImgPort.write();
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

    //allocate space for openCV images for Gabor
    intensityImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    filteredIntensityImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    filteredIntensityImage1 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    filteredIntensityImage2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    totImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 ); // this is later resized
    dwnSample2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample4 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample8 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample4 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample8 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    /****************************************/
    dwnSample2a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample4a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample8a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample2a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample4a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample8a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    dwnSample2b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample4b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample8b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample2b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample4b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample8b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    /******************************************/

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

    // initialize Gabor kernels
    getKernels();     
    
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
    uchar* tmpIntensityImage;
    unsigned char* inputPointer;
    
    // Pointers to raw openCV monochrome image
    shift[0] = (uchar*) cvRedPlane->imageData; 
    shift[1] = (uchar*)cvGreenPlane->imageData;
    shift[2] = (uchar*)cvBluePlane->imageData;
    yellowP = (uchar*)cvYellowPlane->imageData;

    // Pointer to raw extended input (RGB) image
    inputPointer = inputExtImage->getRawImage();

    tmpIntensityImage = (uchar*)intensityImage->imageData;

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
            *tmpIntensityImage++ = (uchar)((*shift[0]+ *shift[1] + *shift[2])/3);

            shift[0]++;
            shift[1]++;
            shift[2]++;
        }

        inputPointer += padding3C;
        shift[0] += paddingMono;
        shift[1] += paddingMono;
        shift[2] += paddingMono;
        yellowP += paddingMono;
        tmpIntensityImage += paddingMono;
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

    if(redG == NULL) return;
    // downsample the images
    //downSampleImage(redG, dwnSample2,2);
    downSampleImage(redG, dwnSample4,4);
    downSampleImage(greenR, dwnSample2,4);
    downSampleImage(blueY, dwnSample8,4);
    //downSampleImage(redG, dwnSample8,8);

    // filter downsampled images
    dwnSample2Fil = cvCreateImage(cvGetSize(dwnSample2),IPL_DEPTH_8U, 1 );
    dwnSample4Fil = cvCreateImage(cvGetSize(dwnSample4),IPL_DEPTH_8U, 1 );
    dwnSample8Fil = cvCreateImage(cvGetSize(dwnSample8),IPL_DEPTH_8U, 1 );

    cvFilter2D(dwnSample2,dwnSample2Fil,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample4,dwnSample4Fil,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample8,dwnSample8Fil,gabKer[kernelUsed],cvPoint(-1,-1));

    //up-sample the filtered images
    upSampleImage(dwnSample2Fil,upSample2,4);
    upSampleImage(dwnSample4Fil,upSample4,4);
    upSampleImage(dwnSample8Fil,upSample8,4);

    // add the images with defined weightages
    IplImage* imagesToAdd[3]= {upSample2,upSample4,upSample8};
    float weight[3]= {.33,.33,.33};
    cvSet(filteredIntensityImage, cvScalar(0));
    maxImages(imagesToAdd,3,filteredIntensityImage,weight);

    //printf("COMES HERE \n");
    /********************************************/

    //downSampleImage(redG, dwnSample2,2);
    downSampleImage(redG, dwnSample4a,2);
    downSampleImage(greenR, dwnSample2a,2);
    downSampleImage(blueY, dwnSample8a,2);
    //downSampleImage(redG, dwnSample8,8);

    // filter downsampled images
    dwnSample2Fila = cvCreateImage(cvGetSize(dwnSample2a),IPL_DEPTH_8U, 1 );
    dwnSample4Fila = cvCreateImage(cvGetSize(dwnSample4a),IPL_DEPTH_8U, 1 );
    dwnSample8Fila = cvCreateImage(cvGetSize(dwnSample8a),IPL_DEPTH_8U, 1 );

    cvFilter2D(dwnSample2a,dwnSample2Fila,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample4a,dwnSample4Fila,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample8a,dwnSample8Fila,gabKer[kernelUsed],cvPoint(-1,-1));

    //up-sample the filtered images
    upSampleImage(dwnSample2Fila,upSample2a,2);
    upSampleImage(dwnSample4Fila,upSample4a,2);
    upSampleImage(dwnSample8Fila,upSample8a,2);

    // add the images with defined weightages
    IplImage* imagesToAdd2[3]= {upSample2a,upSample4a,upSample8a};
    //float weight[3]= {.33,.33,.33};
    cvSet(filteredIntensityImage2, cvScalar(0));
    maxImages(imagesToAdd2,3,filteredIntensityImage2,weight);

    
    //downSampleImage(redG, dwnSample2,2);
    downSampleImage(redG, dwnSample4b,8);
    downSampleImage(greenR, dwnSample2b,8);
    downSampleImage(blueY, dwnSample8b,8);
    //downSampleImage(redG, dwnSample8,8);

    // filter downsampled images
    dwnSample2Filb = cvCreateImage(cvGetSize(dwnSample2b),IPL_DEPTH_8U, 1 );
    dwnSample4Filb = cvCreateImage(cvGetSize(dwnSample4b),IPL_DEPTH_8U, 1 );
    dwnSample8Filb = cvCreateImage(cvGetSize(dwnSample8b),IPL_DEPTH_8U, 1 );

    cvFilter2D(dwnSample2b,dwnSample2Filb,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample4b,dwnSample4Filb,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample8b,dwnSample8Filb,gabKer[kernelUsed],cvPoint(-1,-1));

    //up-sample the filtered images
    upSampleImage(dwnSample2Filb,upSample2b,8);
    upSampleImage(dwnSample4Filb,upSample4b,8);
    upSampleImage(dwnSample8Filb,upSample8b,8);

    // add the images with defined weightages
    IplImage* imagesToAdd8[3]= {upSample2b,upSample4b,upSample8b};
    //float weight[3]= {.33,.33,.33};
    cvSet(filteredIntensityImage1, cvScalar(0));
    maxImages(imagesToAdd8,3,filteredIntensityImage1,weight);
 
    /********************************************/
    
    
    //convert openCV image to YARP image
    float wt[3]={.05,.45,.5};
    IplImage* imagesToAddX[3]={filteredIntensityImage2,filteredIntensityImage,filteredIntensityImage1};
    cvSet(totImage,cvScalar(0));
    addImages(imagesToAddX,3,totImage,wt);
    
    openCVtoYARP(totImage,pyImage,1);
  /*  cvNamedWindow("test1");
    cvShowImage("test1",filteredIntensityImage);
    cvNamedWindow("test2");
    cvShowImage("test2",filteredIntensityImage1);
    cvNamedWindow("test3");
    cvShowImage("test3",filteredIntensityImage2);
    cvWaitKey(0);
*/

    
  
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

void visualFilterThread::setPar(int par, double value) {
    if(par == 1) this->sigma = value;
    else if(par == 2 ) this->gLambda = value;
    else if(par == 3) this->psi = value;
    else if(par == 4) this->gamma = value;
    else if(par == 5) this->kernelUsed = value;
    else if(par == 6) this->dwnSam = value;
    else if(par == 7) this->whichScale = value;
    getKernels();
}

void visualFilterThread::getKernels(){

    double sigmaX = sigma;
    double sigmaY = sigma/gamma;
    double theta[4] = {0, PI/4.0, PI/2.0, 3.0*PI/4.0};
    
    int stdDev = 3;

    for(int i = 0; i < 4 ; ++i) {
     
        double xLim = max<double>(1.0,max<double>(abs(cos(theta[i])*sigmaX*stdDev),abs(sin(theta[i])*sigmaY*stdDev)));
        double yLim = max<double>(1.0,max<double>(abs(sin(theta[i])*sigmaX*stdDev),abs(cos(theta[i])*sigmaY*stdDev)));

        double xStepSize = xLim/(kernelSize[0]/2);
        double yStepSize = yLim/(kernelSize[1]/2);
        //float* tmpKer = gaborKernel[0][0];
        double xThre = xStepSize/2;
        double yThre = yStepSize/2;
        int k = 0; int j = 0;
        for(double xV = -xLim; xV < xLim + xThre; xV += xStepSize) {
            k=0;
            for(double yV = -yLim; yV < yLim + yThre; yV += yStepSize) {
                double xT = xV * cos(theta[i]) + yV * sin(theta[i]);
                double yT = -xV * sin(theta[i]) + yV * cos(theta[i]);
                gK[i][j][k]= exp(-.5*(xT*xT/(sigmaX*sigmaX) + yT*yT/(sigmaY*sigmaY)))*cos(2.0*PI*xT/gLambda + psi)/(2.0*PI*sigmaX*sigmaY);
                gK[i][j][k] *= .3;
                //tmpKer++;
                k++;
            }
            j++;
            printf("\n");
        }

        gabKer[i] = cvCreateMat( 5, 5, CV_32FC1 );
        cvSetData ( gabKer[i], (float*)gK[i], sizeof ( float ) * 5 );
        printf("\n\n\n");
    } 

}

void visualFilterThread::downSampleImage(IplImage* OrigImg,IplImage* DwnImg, int factor) {

    // prepare the destination image, can be avoided for optimization
    cvReleaseImage(&DwnImg);
    DwnImg = cvCreateImage(cvSize(OrigImg->width/factor,OrigImg->height/factor),IPL_DEPTH_8U, 1 );
    cvSet(DwnImg,cvScalar(0));

    uchar* tmpOrigImg = (uchar*)OrigImg->imageData;
    uchar* tmpDwnImg = (uchar*)DwnImg->imageData;    
    uchar* origin = (uchar*)OrigImg->imageData;
    uchar* originDwn = (uchar*)DwnImg->imageData;

    // actual width of images, including paddings    
    int origWidth = OrigImg->widthStep;
    int dwnWidth = DwnImg->widthStep;

    for(int i=0; i<DwnImg->height*factor; ++i){
        //jump to beginning of row
        tmpOrigImg = origin + (i)*origWidth;
        for(int j=0; j<DwnImg->width*factor; ++j){
            //printf("i,j, %d,%d by fact %d,%d counter%d ",i,j,i/factor,j/factor,counter);
            *(tmpDwnImg + ((i)/factor)*dwnWidth + (j)/factor) += *tmpOrigImg/(factor*factor);
            tmpOrigImg++;
        }
        
        
    }       
                    
}

void visualFilterThread::upSampleImage(IplImage* OrigImg,IplImage* UpImg, int factor) {

    // prepare the destination image
    cvReleaseImage(&UpImg);
    UpImg = cvCreateImage(cvSize(OrigImg->width*factor,OrigImg->height*factor),IPL_DEPTH_8U, 1 );
    cvSet(UpImg,cvScalar(0));

    uchar* tmpOrigImg = (uchar*)OrigImg->imageData;    
    uchar* tmpUpImg = (uchar*)UpImg->imageData;
    uchar* originUpImg = (uchar*)UpImg->imageData;
    
    // actual width of images, including paddings
    int UpWidth = UpImg->widthStep;
    int OrigWidth = OrigImg->widthStep;

    for(int i=0; i<UpImg->height; ++i){
        for(int j=0; j<UpImg->width; ++j){
            *tmpUpImg = *(tmpOrigImg + (i/factor)*OrigWidth + j/factor);
            tmpUpImg++;
        }
        // jump to next row
        tmpUpImg = originUpImg + (i+1)*UpWidth;        
    }

}            

void visualFilterThread::downSampleMultiScales(IplImage* OrigImg) {

    // assuming 3 scaled down images are prepared already
    // prepare the destination image, can be avoided for optimization
    cvReleaseImage(&dwnSample2);
    dwnSample2 = cvCreateImage(cvSize(OrigImg->width/2,OrigImg->height/2),IPL_DEPTH_8U, 1 );
    cvSet(dwnSample2,cvScalar(0));
    cvReleaseImage(&dwnSample4);
    dwnSample4 = cvCreateImage(cvSize(OrigImg->width/4,OrigImg->height/4),IPL_DEPTH_8U, 1 );
    cvSet(dwnSample2,cvScalar(0));
    cvReleaseImage(&dwnSample8);
    dwnSample8 = cvCreateImage(cvSize(OrigImg->width/8,OrigImg->height/8),IPL_DEPTH_8U, 1 );
    cvSet(dwnSample2,cvScalar(0));

    uchar* tmpOrigImg = (uchar*)OrigImg->imageData;
    uchar* origin = (uchar*)OrigImg->imageData;    
    uchar* tmpDwnImg2 = (uchar*)dwnSample2->imageData;    
    uchar* originDwn2 = (uchar*)dwnSample2->imageData;
    uchar* tmpDwnImg4 = (uchar*)dwnSample4->imageData;    
    uchar* originDwn4 = (uchar*)dwnSample4->imageData;
    uchar* tmpDwnImg8 = (uchar*)dwnSample8->imageData;    
    uchar* originDwn8 = (uchar*)dwnSample8->imageData;

    // actual width of images, including paddings    
    int origWidth = OrigImg->widthStep;
    int dwnWidth2 = dwnSample2->widthStep;
    int dwnWidth4 = dwnSample4->widthStep;
    int dwnWidth8 = dwnSample8->widthStep;

    int htUp = (OrigImg->height/8)*8;
    int wdUp = (OrigImg->width/8)*8;
    for(int i=0; i<htUp; ++i){
        for(int j=0; j<wdUp; ++j){
            //printf("i,j, %d,%d by fact %d,%d counter%d ",i,j,i/factor,j/factor,counter);
            *(tmpDwnImg2 + (i/2)*dwnWidth2 + j/2) += *tmpOrigImg/(4);
            *(tmpDwnImg4 + (i/4)*dwnWidth4 + j/4) += *tmpOrigImg/(16);
            *(tmpDwnImg8 + (i/8)*dwnWidth8 + j/8) += *tmpOrigImg/(64);
            tmpOrigImg++;
        }
        //jump to next row
        tmpOrigImg = origin + (i+1)*origWidth;
        
    }
                       
}

void visualFilterThread::upSampleMultiScales(IplImage* UpImg) {

    // assuming the destination image UpImg is prepared
    

    uchar* tmpOutImg = (uchar*)UpImg->imageData;
    uchar* originUpImg = (uchar*)UpImg->imageData;   
    uchar* tmpUpImg2 = (uchar*)dwnSample2->imageData;
    uchar* originUpImg2 = (uchar*)dwnSample2->imageData;
    uchar* tmpUpImg4 = (uchar*)dwnSample4->imageData;
    uchar* originUpImg4 = (uchar*)dwnSample4->imageData;
    uchar* tmpUpImg8 = (uchar*)dwnSample8->imageData;
    uchar* originUpImg8 = (uchar*)dwnSample8->imageData;
    
    // actual width of images, including paddings
    int UpWidth2 = dwnSample2->widthStep;
    int UpWidth4 = dwnSample4->widthStep;
    int UpWidth8 = dwnSample8->widthStep;
    int OrigWidth = UpImg->widthStep;

    float weight2 = .33;
    float weight4 = .33;
    float weight8 = .33;

    for(int i=0; i<UpImg->height; ++i){
        for(int j=0; j<UpImg->width; ++j){
            *tmpOutImg = weight2*(*(tmpUpImg2 + (i/2)*UpWidth2 + j/2))
                        + weight4*(*(tmpUpImg4 + (i/4)*UpWidth4 + j/4))
                        + weight8*(*(tmpUpImg8 + (i/8)*UpWidth8 + j/8));
            tmpOutImg++;
        }
        // jump to next row
        tmpOutImg = originUpImg + (i+1)*OrigWidth;        
    }

} 

void visualFilterThread::maxImages(IplImage** ImagesTobeAdded, int numberOfImages,IplImage* resultantImage, float* weights) {
    
    
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
                return ;
            }
            float itsWt = weights[i];
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

void visualFilterThread::addImages(IplImage** ImagesTobeAdded, int numberOfImages,IplImage* resultantImage, float* weights) {
    
    
    uchar* resImageOrigin = (uchar*)resultantImage->imageData;
    int resImageWidth = resultantImage->widthStep;
    uchar* tmpResultImage = (uchar*)resultantImage->imageData;
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
                    unsigned char valNow = (unsigned char)(*tmpImage)* itsWt;
                    //if(*tmpResultImage < valNow) *tmpResultImage = valNow;
                    if((valNow + *tmpResultImage) > 255) *tmpResultImage = 255;
                    else *tmpResultImage = *tmpResultImage + valNow ;
                    *tmpImage++;
                    *tmpResultImage++;
                }
            }
        }
    }

}


void visualFilterThread::openCVtoYARP(IplImage* originalImage, ImageOf<PixelMono>* yarpedImage, int type = 1) {

    //assuming everything is well
    yarpedImage->resize(originalImage->width,originalImage->height);
    if(type==1){
        uchar* originCVImage = (uchar*)originalImage->imageData;
        uchar* originYARPImage = yarpedImage->getRawImage();

        //padding might be different
        int widthCV = originalImage->widthStep;
        int widthY = yarpedImage->getRowSize();
        int minWid = widthCV>widthY? widthY:widthCV;
        for(int i=0; i<originalImage->height; ++i){
            uchar* ptrCV = originCVImage + i*widthCV;
            uchar* ptrY = originYARPImage + i*widthY;
            memcpy(ptrY,ptrCV,minWid);
        }
    }
}

void visualFilterThread::threadRelease() {
    resized = false;
    /*
    if(cvRedPlane != NULL) cvReleaseImage(&cvRedPlane);
    if(cvGreenPlane != NULL) cvReleaseImage(&cvGreenPlane);
    if(cvBluePlane != NULL) cvReleaseImage(&cvBluePlane);
    if(cvYellowPlane != NULL) cvReleaseImage(&cvYellowPlane);
    if(cvRedPlus != NULL) cvReleaseImage(&cvRedPlus);
    if(cvRedMinus != NULL) cvReleaseImage(&cvRedMinus);
    if(cvGreenPlus != NULL) cvReleaseImage(&cvGreenPlus);
    if(cvGreenMinus != NULL) cvReleaseImage(&cvGreenMinus);
    if(cvBluePlus != NULL) cvReleaseImage(&cvBluePlus);
    if(cvYellowMinus != NULL) cvReleaseImage(&cvYellowMinus);
    if(redG != NULL) cvReleaseImage(&redG);
    if(greenR != NULL) cvReleaseImage(&greenR);
    if(blueY != NULL) cvReleaseImage(&blueY);
    if(hRG != NULL) cvReleaseImage(&hRG);
    if(vRG != NULL) cvReleaseImage(&vRG);
    if(hGR != NULL) cvReleaseImage(&hGR);
    if(vGR != NULL) cvReleaseImage(&vGR);
    if(hBY != NULL) cvReleaseImage(&hBY);
    if(vBY != NULL) cvReleaseImage(&vBY);
    if(tempHRG != NULL) cvReleaseImage(&tempHRG);
    if(tempVRG != NULL) cvReleaseImage(&tempVRG);
    if(tempHGR != NULL) cvReleaseImage(&tempHGR);
    if(tempVGR != NULL) cvReleaseImage(&tempVGR);
    if(tempHBY != NULL) cvReleaseImage(&tempHBY);
    if(tempVBY != NULL) cvReleaseImage(&tempVBY);
    if(dwnSample2 != NULL) cvReleaseImage(&dwnSample2);
    if(dwnSample4 != NULL) cvReleaseImage(&dwnSample4);
    if(dwnSample8 != NULL) cvReleaseImage(&dwnSample8);
    if(dwnSample2Fil != NULL) cvReleaseImage(&dwnSample2Fil);
    if(dwnSample4Fil != NULL) cvReleaseImage(&dwnSample4Fil);
    if(dwnSample8Fil != NULL) cvReleaseImage(&dwnSample8Fil);
    if(upSample2 != NULL) cvReleaseImage(&upSample2);
    if(upSample4 != NULL) cvReleaseImage(&upSample4);
    if(upSample8 != NULL) cvReleaseImage(&upSample8);
    if(intensityImage != NULL) cvReleaseImage(&intensityImage); 
    if(filteredIntensityImage != NULL) cvReleaseImage(&filteredIntensityImage);
    if(dwnImage != NULL) cvReleaseImage(&dwnImage);
    */
    
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

