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
 * @brief Implementation of the visual filter thread (see earlyVisionThread.h).
 */

#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/earlyVisionThread.h>
#include <cstring>

#define ONE_BY_ROOT_TWO 0.707106781
#define ONE_BY_ROOT_THREE 0.577350269

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

    coRG                = new ImageOf<PixelMono>;
    coGR                = new ImageOf<PixelMono>;
    coBY                = new ImageOf<PixelMono>;

    tmpMonoLPImage      = new ImageOf<PixelMono>;
    tmpMono16LPImage    = new ImageOf<PixelMono16>;
    tmpMono16LPImage1   = new ImageOf<PixelMono16>;
    tmpMono16LPImage2   = new ImageOf<PixelMono16>;
    tmpMonoLPImageSobelHorz      = new ImageOf<PixelMono>;
    tmpMonoLPImageSobelVert      = new ImageOf<PixelMono>;

    orientationImage    = new ImageOf<PixelMono>;
    
    

    edges               = new ImageOf<PixelMono>;
    
    YofYUV              = new ImageOf<PixelMono>;    
    intensImg           = new ImageOf<PixelMono>;
    cartIntensImg       = new ImageOf<PixelMono>;
    
    redPlane            = new ImageOf<PixelMono>;
    greenPlane          = new ImageOf<PixelMono>;
    bluePlane           = new ImageOf<PixelMono>;
    yellowPlane         = new ImageOf<PixelMono>;

    Yplane            = new ImageOf<PixelMono>;
    Uplane            = new ImageOf<PixelMono>;
    Vplane            = new ImageOf<PixelMono>;

    sobelHorXConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono16>,uchar>(3,Sobel3XHorz,0,250.0,0);
    sobelVerXConvolution =  new convolve<ImageOf<PixelMono16>,uchar,ImageOf<PixelMono16>,uchar>(3,Sobel3XVert,1,250.0,0);
    sobelHorYConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono16>,uchar>(3,Sobel3YHorz,0,250.0,0);
    sobelVerYConvolution =  new convolve<ImageOf<PixelMono16>,uchar,ImageOf<PixelMono16>,uchar>(3,Sobel3YVert,1,250.0,0);
    gaborPosHorConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,0,.5,0);
    gaborPosVerConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(5,G5,1,.5,0);
    gaborNegHorConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,0,.5,0);
    gaborNegVerConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(7,GN7,1,.5,0);

    kirschConvolution =  new convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono>,uchar>(3,3,K1,.005,0);
    
    
    
    
    // Let us initialize IplImage pointers to NULL

    //hRG =vRG=hGR=vGR=hBY=vBY= NULL;

    //16 bit image to avoid overflow in Sobel operator
    //tempHRG=tempVRG=tempHGR=tempVGR=tempHBY=tempVBY=NULL;    
          
    
    lambda = 0.3f;
    resized = false;

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
    delete coRG;
    delete coGR;
    delete coBY;
    delete tmpMonoLPImage;
    delete tmpMono16LPImage;
    delete tmpMono16LPImage1;
    delete tmpMono16LPImage2;
    delete tmpMonoLPImageSobelHorz;
    delete tmpMonoLPImageSobelVert;
    delete orientationImage;
    delete sobelHorXConvolution;
    delete sobelVerXConvolution;
    delete sobelHorYConvolution;
    delete sobelVerYConvolution;
    delete gaborPosHorConvolution;    
    delete gaborPosVerConvolution;    
    delete gaborNegHorConvolution;    
    delete gaborNegVerConvolution;
    delete kirschConvolution;    
    delete edges;
    delete YofYUV;
    delete intensImg;
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
    delete img_out_Y;
    delete img_out_UV;
    delete img_out_V;
    delete img_Y;
    delete img_UV;
    delete img_V;
    ippiFree( colour ); 
    ippiFree( tmp ); 
    free ( pyuva );
    ippiFree( yuva_orig );
    ippiFree( first_plane );
    ippiFree( second_plane );
    ippiFree( third_plane );
    ippiFree( cs_tot_32f );
    ippiFree( colcs_out );
    ippiFree( ycs_out );
    ippiFree( scs_out );
    ippiFree( vcs_out );

    printf("Called destructor \n");
    
    
}

bool earlyVisionThread::threadInit() {
    printf("opening ports \n");
    /* open ports */ 
    
    
    if (!imagePortIn.open("/imageRGB:i")) {
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

    if (!orientPort.open(getName("/orientation:o").c_str())) {
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
        
            
            // orientation port
            
            if((orientationImage!=0)&&(orientPort.getOutputCount())) {
                orientPort.prepare() = *(orientationImage);
                orientPort.write();
                }

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

            if((img_out_Y!=0)&&(colorOpp1Port.getOutputCount())) {
                colorOpp1Port.prepare() = *(img_out_Y);
                colorOpp1Port.write();
                }
            if((img_out_UV!=0)&&(colorOpp2Port.getOutputCount())) {
                colorOpp2Port.prepare() = *(img_out_UV);
                colorOpp2Port.write();
                }
            if((img_out_V!=0)&&(colorOpp3Port.getOutputCount())) {
                colorOpp3Port.prepare() = *(img_out_V);
                colorOpp3Port.write();
            }

            

            
            
        }
   }
}

/*void earlyVisionThread::resizeCartesian(int width,int height) {
    cartImage->resize(width, height);
    width_cart = width;
    height_cart = height;
}*/


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

    //LATER: change to relevant size only
    coRG->resize(width, height);
    coGR->resize(width, height);
    coBY->resize(width, height);

    tmpMonoLPImage->resize(width, height);
    tmpMono16LPImage->resize(width, height);
    tmpMono16LPImage1->resize(width, height);
    tmpMono16LPImage2->resize(width, height);
    tmpMonoLPImageSobelHorz->resize(width, height);
    tmpMonoLPImageSobelVert->resize(width, height);
    

    edges->resize(width, height);
    intensImg->resize(width, height);

    cartIntensImg->resize(ROW_SIZE, COL_SIZE);

    redPlane->resize(width, height);
    greenPlane->resize(width, height);
    bluePlane->resize(width, height);
    yellowPlane->resize(width, height);
    Yplane->resize(width, height);
    Uplane->resize(width, height);
    Vplane->resize(width, height);
    
    // allocating for CS

    origsize.width = width_orig;
    origsize.height = height_orig;

    srcsize.width = this->width;
    srcsize.height = this->height;

    colour  = ippiMalloc_8u_C4( this->width, this->height, &psb4);

    yuva_orig = ippiMalloc_8u_C1( this->width *4, this->height, &psb4);
    first_plane    = ippiMalloc_8u_C1( this->width, this->height, &f_psb);
    second_plane    = ippiMalloc_8u_C1( this->width, this->height, &s_psb);
    third_plane   = ippiMalloc_8u_C1( this->width, this->height, &t_psb);
    
    tmp     = ippiMalloc_8u_C1( this->width, this->height, &psb );// to separate alpha channel
    pyuva = (Ipp8u**) malloc(4*sizeof(Ipp8u*));

    cs_tot_32f  = ippiMalloc_32f_C1( this->width, this->height, &psb_32f );
    colcs_out   = ippiMalloc_8u_C1( this->width, this->height,  &col_psb );
    ycs_out     = ippiMalloc_8u_C1( this->width, this->height,  &ycs_psb );
    scs_out     = ippiMalloc_8u_C1( this->width, this->height,  &ycs_psb );
    vcs_out     = ippiMalloc_8u_C1( this->width, this->height,  &ycs_psb );

    ncsscale = 4;
    centerSurr  = new CentSur( srcsize , ncsscale );

    //inputExtImage = new ImageOf<PixelRgb>;
    //inputExtImage->resize( this->width, this->height );
    isYUV = true;
	img_Y = new ImageOf<PixelMono>;
	img_Y->resize( this->width, this->height );

    img_out_Y = new ImageOf<PixelMono>;
	img_out_Y->resize( width_orig, height_orig );

    img_UV = new ImageOf<PixelMono>;
	img_UV->resize( this->width, this->height );

    img_out_UV = new ImageOf<PixelMono>;
	img_out_UV->resize( width_orig, height_orig );

    img_V = new ImageOf<PixelMono>;
	img_V->resize( this->width, this->height );

    img_out_V = new ImageOf<PixelMono>;
	img_out_V->resize( width_orig, height_orig ); 
   
    
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
    int padMono;
    uchar* tmpIntensityImage;
    uchar* ptrIntensityImg;
    uchar* inputPointer;

    // for CS, these planes are of extended size
    Ipp8u* CSPlane[3];
    CSPlane[0] = first_plane;
    CSPlane[1] = second_plane;
    CSPlane[2] = third_plane;
    int padPlane1 = f_psb - this->width;
    int padPlane2 = s_psb - this->width;
    int padPlane3 = t_psb - this->width;
    
    // Pointers to raw plane image
    shift[0] = (uchar*) redPlane->getRawImage(); 
    shift[1] = (uchar*) greenPlane->getRawImage(); 
    shift[2] = (uchar*) bluePlane->getRawImage(); 
    shift[3] = (uchar*) yellowPlane->getRawImage();

    YUV[0] = (uchar*) Yplane->getRawImage(); 
    YUV[1] = (uchar*) Uplane->getRawImage(); 
    YUV[2] = (uchar*) Vplane->getRawImage(); 
    ptrIntensityImg = (uchar*) intensImg->getRawImage();
    inputPointer = (uchar*)extendedInputImage->getRawImage();
    padInput =  extendedInputImage->getPadding();
    padMono = redPlane->getPadding();

    

    const int h = extendedInputImage->height();
    const int w = extendedInputImage->width();
    for(int r = 0; r < h; r++) {       
        
        for(int c = 0; c < w; c++) {
            *shift[0] = *inputPointer++;
            *shift[1] = *inputPointer++;
            *shift[2] = *inputPointer++;

            *shift[3]++ = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));
            *ptrIntensityImg++ = ONE_BY_ROOT_THREE * sqrt(*shift[0] * *shift[0] +*shift[1] * *shift[1] +*shift[2] * *shift[2]);

            // RGB to Y'UV conversion
            *YUV[0] = .299* (*shift[0]) + .587 * (*shift[1]) + .114 * (*shift[2]);
            *YUV[1] = -.14713* (*shift[0]) + -.28886 * (*shift[1]) + .436 * (*shift[2]);
            *YUV[2] = .615* (*shift[0]) + -.51499 * (*shift[1]) + -.10001 * (*shift[2]);

            // CS, take in IPP planes
            *CSPlane[0]++ = *YUV[0];
            *CSPlane[1]++ = *YUV[1];
            *CSPlane[2]++ = *YUV[2];
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
        shift[0] += padMono;
        shift[1] += padMono;
        shift[2] += padMono;
        shift[3] += padMono;
        YUV[0] += padMono;
        YUV[1] += padMono;
        YUV[2] += padMono;
        CSPlane[0] += padPlane1;
        CSPlane[1] += padPlane2;
        CSPlane[2] += padPlane3;
                
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

void earlyVisionThread::centerSurrounding(){

        //performs centre-surround uniqueness analysis on first plane
        centerSurr->proc_im_8u( first_plane , f_psb );
        ippiCopy_8u_C1R( centerSurr->get_centsur_norm8u(), centerSurr->get_psb_8u(), ycs_out, ycs_psb , srcsize );

        ippiSet_32f_C1R( 0.0, cs_tot_32f, psb_32f, srcsize );

        //performs centre-surround uniqueness analysis on second plane:
        centerSurr->proc_im_8u( second_plane , s_psb );
        if ( isYUV ){
            ippiAdd_32f_C1IR( centerSurr->get_centsur_32f(), centerSurr->get_psb_32f(), cs_tot_32f, psb_32f, srcsize );
        }
        else
            ippiCopy_8u_C1R( centerSurr->get_centsur_norm8u(), centerSurr->get_psb_8u(), scs_out, ycs_psb , srcsize ); 

        //Colour process V:performs centre-surround uniqueness analysis:
        centerSurr->proc_im_8u( third_plane , t_psb );

        if ( isYUV ){
            ippiAdd_32f_C1IR( centerSurr->get_centsur_32f(), centerSurr->get_psb_32f(), cs_tot_32f, psb_32f, srcsize );
        }
        else
            ippiCopy_8u_C1R( centerSurr->get_centsur_norm8u(), centerSurr->get_psb_8u(), vcs_out, ycs_psb , srcsize ); 

        if ( isYUV ){
            //get min max   
            Ipp32f valueMin,valueMax;
            valueMin = 0.0f;
            valueMax = 0.0f;
            ippiMinMax_32f_C1R( cs_tot_32f, psb_32f, srcsize, &valueMin, &valueMax );
            if ( valueMax == valueMin ){ valueMax = 255.0f; valueMin = 0.0f;}
            ippiScale_32f8u_C1R( cs_tot_32f, psb_32f, colcs_out, col_psb, srcsize, valueMin, valueMax );
        }
  
        //revert to yarp images
        ippiCopy_8u_C1R( ycs_out, ycs_psb, img_Y->getRawImage(), img_Y->getRowSize(), srcsize );
        
        if ( isYUV ){
            ippiCopy_8u_C1R( colcs_out, col_psb, img_UV->getRawImage(), img_UV->getRowSize(), srcsize );
        }else{
            ippiCopy_8u_C1R( scs_out, ycs_psb, img_UV->getRawImage(), img_UV->getRowSize(), srcsize );
            ippiCopy_8u_C1R( vcs_out, ycs_psb, img_V->getRawImage(), img_V->getRowSize(), srcsize );
        }

        //this is nasty, resizes the images...
        unsigned char* imgY = img_Y->getPixelAddress( maxKernelSize, maxKernelSize );
        unsigned char* imgUV = img_UV->getPixelAddress( maxKernelSize, maxKernelSize );
        unsigned char* imgV;
        unsigned char* imgVo;

        if (!isYUV){
           imgV = img_V->getPixelAddress( maxKernelSize, maxKernelSize );
           imgVo = img_out_V->getRawImage();
        }
        
        unsigned char* imgYo = img_out_Y->getRawImage();
        unsigned char* imgUVo = img_out_UV->getRawImage();
        int rowsize= img_out_Y->getRowSize();
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
            CSPort1.prepare() = *img_out_Y;	
            CSPort1.write();
        }

        //output UV centre-surround results to ports
        if ( CSPort2.getOutputCount()>0 ){
             CSPort2.prepare() = *img_out_UV;	
            CSPort2.write();
        }
        //output UV centre-surround results to ports
        if ( !isYUV && CSPort3.getOutputCount()>0 ){
            CSPort3.prepare() = *img_out_V;	
            CSPort3.write();
        }

}

void earlyVisionThread::colorOpponency(){

    // get opponent colors for eg R+G- from R+ and G- channels
    // Finding color opponency now
    uchar* pRG = coRG->getRawImage();
    uchar* pGR = coGR->getRawImage();
    uchar* pBY = coBY->getRawImage();

    uchar* rPlus = Rplus->getRawImage();
    uchar* rMinus = Rminus->getRawImage();
    uchar* gPlus = Gplus->getRawImage();
    uchar* gMinus = Gminus->getRawImage();
    uchar* bPlus = Bplus->getRawImage();
    uchar* yMinus = Yminus->getRawImage();

    int padChannel = Rplus->getPadding();
    int padOpponents = coRG->getPadding();

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
      

}

void earlyVisionThread::orientation() {


    // Using Kirsch matrix
    orientationImage->zero();
    kirschConvolution->convolve2D(Yplane,orientationImage);
     

    
  
}



void earlyVisionThread::edgesExtract() {

    tmpMono16LPImage->zero();
    tmpMono16LPImage1->zero();
    
    IplImage* tmpI = (IplImage*)tmpMono16LPImage->getIplImage();
    IplImage* tmpIn = (IplImage*)intensImg->getIplImage();
    

    // X derivative 
    sobelHorXConvolution->convolve1D(intensImg,tmpMono16LPImage);
    sobelVerXConvolution->convolve1D(tmpMono16LPImage,tmpMono16LPImage1);

    tmpMono16LPImage->zero();
    tmpMono16LPImage2->zero();

    // Y derivative 
    sobelHorYConvolution->convolve1D(intensImg,tmpMono16LPImage);
    sobelVerYConvolution->convolve1D(tmpMono16LPImage,tmpMono16LPImage2);
    

    //clearing up the previous value
    edges->zero();
    uchar* pedges=edges->getRawImage();
    uchar* ptrHorz = tmpMono16LPImage1->getRawImage();
    uchar* ptrVert = tmpMono16LPImage2->getRawImage();
     
    const int pad_edges = edges->getPadding();
    int padHorz = tmpMono16LPImage1->getPadding();
    int padVert = tmpMono16LPImage2->getPadding();

    // LATER: Do not consider extended portion
    
    for (int row = 0; row < height; row++) {
        for (int col = 0; col < width; col++) {

            if (row < height_orig) {
                double rg = (*ptrHorz ) * (*ptrHorz ) + (*ptrVert ) * (*ptrVert );
                *pedges = (unsigned char)(sqrt(rg)); 
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
    //cvReleaseImage(&retImage);
    //retImage = cvCreateImage(cvSize(imgWidth,imgHeight),IPL_DEPTH_8U, 1 );
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

    

    
    
    printf("Release complete!\n");
    resized = false;    
    
}

void earlyVisionThread::onStop() {

    printf("calling on-stop\n");

    imagePortIn.interrupt();
    intenPort.interrupt();
    chromPort.interrupt();
    edgesPort.interrupt();
    orientPort.interrupt();
    colorOpp1Port.interrupt();
    colorOpp2Port.interrupt();
    colorOpp3Port.interrupt();
    
    
    imagePortIn.close();
    intenPort.close();
    chromPort.close();
    edgesPort.close();
    orientPort.close();
    colorOpp1Port.close();
    colorOpp2Port.close();
    colorOpp3Port.close();
    
    printf("done with on-stop\n");
    
}


