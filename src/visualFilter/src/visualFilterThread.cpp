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

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

const int maxKernelSize = 5;

template<class T>
inline T max(T a, T b, T c) {    
    if(b>a) a=b;
    if(c>a) a=c;
    return a;
}

visualFilterThread::visualFilterThread() {
    redPlane=new ImageOf<PixelMono>;
    redPlane2=new ImageOf<PixelMono>;
    redPlane3=new ImageOf<PixelMono>;
    greenPlane=new ImageOf<PixelMono>;
    greenPlane2=new ImageOf<PixelMono>;
    greenPlane3=new ImageOf<PixelMono>;
    bluePlane=new ImageOf<PixelMono>;
    bluePlane2=new ImageOf<PixelMono>;
    bluePlane3=new ImageOf<PixelMono>;
    yellowPlane=new ImageOf<PixelMono>;
    yellowPlane2=new ImageOf<PixelMono>;
    inputExtImage=new ImageOf<PixelRgb>;
    inputImageFiltered=new ImageOf<PixelRgb>;

    redPlus=new ImageOf<PixelMono>;
    redMinus=new ImageOf<PixelMono>;
    greenPlus=new ImageOf<PixelMono>;
    greenMinus=new ImageOf<PixelMono>;
    bluePlus=new ImageOf<PixelMono>;
    yellowMinus=new ImageOf<PixelMono>;

    redGreen=new ImageOf<PixelMono>;
    greenRed=new ImageOf<PixelMono>;
    blueYellow=new ImageOf<PixelMono>;
    edges=new ImageOf<PixelMono>;

    buffer =0 ;
    redGreenH8s   = 0;
    greenRedH8s   = 0;
    blueYellowH8s = 0;
    redGreenV8s   = 0;
    greenRedV8s   = 0;
    blueYellowV8s = 0;
    lambda = 0.1f;

    resized=false;
}

visualFilterThread::~visualFilterThread() {
    delete redPlane;
    delete redPlane2;
    delete redPlane3;
    delete greenPlane;
    delete greenPlane2;
    delete greenPlane3;
    delete bluePlane;
    delete bluePlane2;
    delete bluePlane3;
    delete yellowPlane;
    delete yellowPlane2;
    delete inputExtImage;
    delete inputImageFiltered;
    delete inputImage;

    delete redPlus;
    delete redMinus;
    delete greenPlus;
    delete greenMinus;
    delete bluePlus;
    delete yellowMinus;

    delete redGreen;
    delete greenRed;
    delete blueYellow;
    delete edges;

    if(buffer!=0)
        ippsFree(buffer);
    if(redGreenH8s!=0)
        ippsFree(redGreenH8s);
    if(greenRedH8s!=0)
        ippsFree(greenRedH8s);
    if(blueYellowH8s!=0)
        ippsFree(blueYellowH8s);
    if(redGreenV8s!=0)
        ippsFree(redGreenV8s);
    if(greenRedV8s!=0)
        ippsFree(greenRedV8s);
    if(blueYellowV8s!=0)
        ippsFree(blueYellowV8s);
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
    this->width_orig=width_orig;
    this->height_orig=height_orig;
    this->width=width_orig+2*maxKernelSize;
    this->height=height_orig+maxKernelSize;

    // resizing the ROI
    originalSrcsize.height=height_orig;
    originalSrcsize.width=width_orig;
    srcsize.width=width;
    srcsize.height=height;

    // resizing plane images
    edges->resize(width_orig, height_orig);
    inputImageFiltered->resize(width_orig, height_orig);
    inputImageFiltered->zero();
 
    inputExtImage->resize(width,height);
    redPlane->resize(width,height);
    redPlane2->resize(width,height);
    redPlane3->resize(width,height);
    greenPlane->resize(width,height);
    greenPlane2->resize(width,height);
    greenPlane3->resize(width,height);
    bluePlane->resize(width,height);
    bluePlane2->resize(width,height);
    bluePlane3->resize(width,height);
    yellowPlane->resize(width,height);
    yellowPlane2->resize(width,height);

    redPlus->resize(width,height);
    redMinus->resize(width,height);
    greenPlus->resize(width,height);
    greenMinus->resize(width,height);
    bluePlus->resize(width,height);
    yellowMinus->resize(width,height);

    redGreen->resize(width, height);
    greenRed->resize(width, height);
    blueYellow->resize(width, height);

    ippiFilterSobelHorizGetBufferSize_8u16s_C1R(srcsize, ippMskSize3x3, &size1);
    buffer = ippsMalloc_8u(size1);
    redGreenH8s= ippsMalloc_8s(width*height*sizeof(Ipp8s)*2);
    greenRedH8s= ippsMalloc_8s(width*height*sizeof(Ipp8s)*2);
    blueYellowH8s= ippsMalloc_8s(width*height*sizeof(Ipp8s)*2);
    redGreenV8s= ippsMalloc_8s(width*height*sizeof(Ipp8s)*2);
    greenRedV8s= ippsMalloc_8s(width*height*sizeof(Ipp8s)*2);
    blueYellowV8s= ippsMalloc_8s(width*height*sizeof(Ipp8s)*2);

    // populating once and for all the look up tables to avoid costly float calculations with in the loop
    float ul = 1.0f - lambda;
    for(int i=0;i<CHAR_LIMIT;++i) {
        for(int j=0;j<CHAR_LIMIT;++j) {
           edges_LUT[i*CHAR_LIMIT+j]=(unsigned char)( sqrt((i-CHAR_LIMIT/2.0)*(i-CHAR_LIMIT/2.0) 
                                        +(j-CHAR_LIMIT/2.0)*(j-CHAR_LIMIT/2.0))*(255.0/1024));
           filter_LUT[i*CHAR_LIMIT+j]=(unsigned char)(lambda * i + ul * j + .5f);
        }
    }    
}

void visualFilterThread::filterInputImage() {
    int i;
    const int sz = inputImage->getRawImageSize();
    unsigned char * pFiltered = inputImageFiltered->getRawImage();
    unsigned char * pCurr = inputImageFiltered->getRawImage();
    const float ul = 1.0f - lambda;
    for (i = 0; i < sz; i++) {
        pCurr++;
        int k= *pCurr; k *= CHAR_LIMIT; k += *pFiltered;
        *pFiltered = filter_LUT[k];
        //*pFiltered = (unsigned char)(lambda * *pCurr++ + ul * *pFiltered + .5f);
        pFiltered ++;
    }
}

ImageOf<PixelRgb>* visualFilterThread::extender(ImageOf<PixelRgb>* inputOrigImage, int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*inputExtImage, *inputOrigImage, maxSize);
    return inputExtImage;
}

void visualFilterThread::extractPlanes() {
    /* check ipp for the existence of functions with output by plane (rather than by pixel) */
    Ipp8u* shift[3];
    Ipp8u* yellowP;
    
    shift[0] = redPlane->getRawImage();
    shift[1] = greenPlane->getRawImage();
    shift[2] = bluePlane->getRawImage();
    yellowP = yellowPlane->getRawImage();
    Ipp8u* inputPointer = inputExtImage->getRawImage();

    /* use getPadding!!!! */
    int paddingMono = redPlane->getPadding(); //redPlane->getRowSize()-redPlane->width();
    int padding3C = inputExtImage->getPadding(); //inputExtImage->getRowSize()-inputExtImage->width()*3;

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
    IppiSize srcPlusSize = { 5, 5 };    // variance = 1
    IppiSize srcMinusSize = { 7, 7 };   // variance = 3 which is 3 times the variance 1
    const int halfSize = 3;             // 7/3 (square kernel always)
    IppiSize gaussRoi = { width_orig+4, height_orig+2 };    // the size of the Roi is determined by the remainder of the border (5-halfSize)

    static Ipp32f srcMinus[7*7] = {
        0.0113f, 0.0149f, 0.0176f, 0.0186f, 0.0176f, 0.0149f, 0.0113f,
        0.0149f, 0.0197f, 0.0233f, 0.0246f, 0.0233f, 0.0197f, 0.0149f,
        0.0176f, 0.0233f, 0.0275f, 0.0290f, 0.0275f, 0.0233f, 0.0176f,
        0.0186f, 0.0246f, 0.0290f, 0.0307f, 0.0290f, 0.0246f, 0.0186f,
        0.0176f, 0.0233f, 0.0275f, 0.0290f, 0.0275f, 0.0233f, 0.0176f,
        0.0149f, 0.0197f, 0.0233f, 0.0246f, 0.0233f, 0.0197f, 0.0149f,
        0.0113f, 0.0149f, 0.0176f, 0.0186f, 0.0176f, 0.0149f, 0.0113f
    };

    IppiPoint anchor = {3,3};
    
    /* gaussian is separable and therefore it's cheaper to comput by row and then by columns  */
    ippiFilter32f_8u_C1R(redPlane->getPixelAddress(halfSize,halfSize), redPlane->getRowSize(), redMinus->getPixelAddress(halfSize,halfSize), redMinus->getRowSize(), gaussRoi, srcMinus, srcMinusSize, anchor);
    ippiFilter32f_8u_C1R(yellowPlane->getPixelAddress(halfSize,halfSize), yellowPlane->getRowSize(), yellowMinus->getPixelAddress(halfSize,halfSize), yellowMinus->getRowSize(), gaussRoi, srcMinus, srcMinusSize, anchor);
    ippiFilter32f_8u_C1R(greenPlane->getPixelAddress(halfSize,halfSize), greenPlane->getRowSize(), greenMinus->getPixelAddress(halfSize,halfSize), greenMinus->getRowSize(), gaussRoi, srcMinus, srcMinusSize, anchor);

    ippiFilterGauss_8u_C1R(bluePlane->getPixelAddress(halfSize,halfSize), bluePlane->getRowSize(), bluePlus->getPixelAddress(halfSize,halfSize), bluePlus->getRowSize(), gaussRoi, ippMskSize5x5);
    ippiFilterGauss_8u_C1R(redPlane->getPixelAddress(halfSize,halfSize), redPlane->getRowSize(), redPlus->getPixelAddress(halfSize,halfSize), redPlus->getRowSize(), gaussRoi, ippMskSize5x5);
    ippiFilterGauss_8u_C1R(greenPlane->getPixelAddress(halfSize,halfSize), greenPlane->getRowSize(), greenPlus->getPixelAddress(halfSize,halfSize), greenPlus->getRowSize(), gaussRoi, ippMskSize5x5);
}

void visualFilterThread::colourOpponency() {
    // and in-place operations???
    // also, the original paper used a weighing factor of 1.5 to 1 in taking the rg, gr and by difference (1.5 to the gaussian w/ sigma=3).
    ippiRShiftC_8u_C1R(redPlus->getRawImage(), redPlane->getRowSize(), 1, redPlane2->getRawImage(), redPlane2->getRowSize(), srcsize);
    ippiAddC_8u_C1RSfs(redPlane2->getRawImage(), redPlane2->getRowSize(), 128, redPlane3->getRawImage(), redPlane3->getRowSize(), srcsize, 0);
    ippiRShiftC_8u_C1R(redMinus->getRawImage(), redMinus->getRowSize(), 1, redPlane2->getRawImage(), redPlane2->getRowSize(), srcsize);
    ippiRShiftC_8u_C1R(greenPlus->getRawImage(), greenPlus->getRowSize(),1, greenPlane2->getRawImage(), greenPlane2->getRowSize(), srcsize);
    ippiAddC_8u_C1RSfs(greenPlane2->getRawImage(), greenPlane2->getRowSize(), 128, greenPlane3->getRawImage(), greenPlane3->getRowSize(), srcsize, 0);
    ippiRShiftC_8u_C1R(greenMinus->getRawImage(), greenMinus->getRowSize(), 1, greenPlane2->getRawImage(), greenPlane2->getRowSize(), srcsize);
    ippiRShiftC_8u_C1R(bluePlus->getRawImage(), bluePlus->getRowSize(), 1, bluePlane2->getRawImage(), bluePlane2->getRowSize(), srcsize);
    ippiAddC_8u_C1RSfs(bluePlane2->getRawImage(), bluePlane2->getRowSize(), 128, bluePlane3->getRawImage(), bluePlane3->getRowSize(), srcsize, 0);
    ippiRShiftC_8u_C1R(yellowMinus->getRawImage(), yellowMinus->getRowSize(), 1, yellowPlane2->getRawImage(), yellowPlane2->getRowSize(), srcsize);

    ippiSub_8u_C1RSfs(greenPlane2->getRawImage(), greenPlane2->getRowSize(), redPlane3->getRawImage(), redPlane3->getRowSize(), redGreen->getRawImage(), redGreen->getRowSize(), srcsize, 0);
    ippiSub_8u_C1RSfs(redPlane2->getRawImage(), redPlane2->getRowSize(), greenPlane3->getRawImage(), greenPlane3->getRowSize(), greenRed->getRawImage(), greenRed->getRowSize(), srcsize, 0);
    ippiSub_8u_C1RSfs(yellowPlane2->getRawImage(), yellowPlane2->getRowSize(), bluePlane3->getRawImage(), bluePlane3->getRowSize(), blueYellow->getRawImage(), blueYellow->getRowSize(), srcsize, 0);
}



void visualFilterThread::edgesExtract() {
    psb16s = redGreen->getRowSize() * sizeof(Ipp8s);
    ippiFilterSobelHorizBorder_8u8s_C1R(redGreen->getRawImage(), redGreen->getRowSize(), redGreenH8s, psb16s, srcsize, ippMskSize3x3, ippBorderRepl, 0, 1, buffer);
    ippiFilterSobelHorizBorder_8u8s_C1R(greenRed->getRawImage(), greenRed->getRowSize(), greenRedH8s, psb16s, srcsize, ippMskSize3x3, ippBorderRepl, 0, 1, buffer);
    ippiFilterSobelHorizBorder_8u8s_C1R(blueYellow->getRawImage(), blueYellow->getRowSize(), blueYellowH8s, psb16s, srcsize, ippMskSize3x3, ippBorderRepl, 0, 1, buffer);
    ippiFilterSobelVertBorder_8u8s_C1R(redGreen->getRawImage(), redGreen->getRowSize(), redGreenV8s, psb16s, srcsize, ippMskSize3x3, ippBorderRepl, 0, 1, buffer);
    ippiFilterSobelVertBorder_8u8s_C1R(greenRed->getRawImage(), greenRed->getRowSize(), greenRedV8s, psb16s, srcsize, ippMskSize3x3, ippBorderRepl, 0, 1, buffer);
    ippiFilterSobelVertBorder_8u8s_C1R(blueYellow->getRawImage(), blueYellow->getRowSize(), blueYellowV8s, psb16s, srcsize, ippMskSize3x3, ippBorderRepl, 0, 1, buffer);    

    unsigned char* pedges=edges->getRawImage();
    const int pad_edges = edges->getPadding();
    const int pad_16s = (psb16s / sizeof(Ipp8s)) - width_orig;

    int j = maxKernelSize*(psb16s/sizeof(Ipp8s))+maxKernelSize;
    redGreenH8s+=j;redGreenV8s+=j;
    greenRedH8s+=j;greenRedV8s+=j;
    blueYellowH8s+=j;blueYellowV8s+=j;
    // edges extraction
    for (int row = 0; row < height_orig; row++) {
        for (int col = 0; col < width_orig; col++) {
            double rg = *redGreenH8s * *redGreenH8s + *redGreenV8s * *redGreenV8s;
            double gr = *greenRedH8s * *greenRedH8s + *greenRedV8s * *greenRedV8s;
            double by = *blueYellowH8s * *blueYellowH8s + *blueYellowV8s * *blueYellowV8s;            
            if (row < height_orig - 2) {
               
                *pedges = max<unsigned char> (edges_LUT[(*redGreenH8s+CHAR_LIMIT/2)*CHAR_LIMIT + *redGreenV8s 
                        +CHAR_LIMIT/2],edges_LUT[(*greenRedH8s+CHAR_LIMIT/2)*CHAR_LIMIT + *greenRedV8s
                        +CHAR_LIMIT/2],edges_LUT[(*blueYellowH8s+CHAR_LIMIT/2)*CHAR_LIMIT + *blueYellowV8s+CHAR_LIMIT/2]);
                
            }
            else
                *pedges = 0;
            
            pedges++;
            redGreenH8s++;redGreenV8s++;
            greenRedH8s++;greenRedV8s++;
            blueYellowH8s++;blueYellowV8s++;
        }
        // padding
        pedges += pad_edges;
        redGreenH8s+=pad_16s;redGreenV8s+=pad_16s;
        greenRedH8s+=pad_16s;greenRedV8s+=pad_16s;
        blueYellowH8s+=pad_16s;blueYellowV8s+=pad_16s;
    }
    int r=height*(psb16s/sizeof(Ipp8s))+maxKernelSize;
    redGreenH8s-=r;redGreenV8s-=r;
    greenRedH8s-=r;greenRedV8s-=r;
    blueYellowH8s-=r;blueYellowV8s-=r;
    
    
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

