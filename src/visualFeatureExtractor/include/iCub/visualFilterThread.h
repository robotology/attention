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
 * @file visualFilterThread.h
 * @brief Definition of a thread that receives images and does the computation for the
 * visual filter module (see visualFilterModule.h).
 */

#ifndef _VISUAL_FILTER_THREAD_H_
#define _VISUAL_FILTER_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Stamp.h>


#include <cv.h>
#include <cvaux.h>
#include <highgui.h>


#define CHAR_LIMIT 256
#define PI 3.14159265358979

class visualFilterThread : public yarp::os::Thread
{
private:
    

    int psb;
    int width_orig, height_orig;        // dimension of the input image (original)
    int width, height;                  // dimension of the extended input image (extending)
    int size1;                          // size of the buffer
    int psb16s;                         // step size of the Ipp16s vectors
    float lambda;                       // costant for the temporal filter
   
    double sigma, gLambda,psi, gamma, dwnSam,whichScale;
    int kernelUsed;
    int kernelSize[2];
    CvMat* gabKer[4];

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImage;            // input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImageFiltered;    // time filtered input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputExtImage;         // extended input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *logPolarImage;
        
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane;             // image of the red channel
     IplImage *cvRedPlane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane2;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane3;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane;           // image of the green channel
     IplImage *cvGreenPlane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane2;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane3;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane;            // image of the blue channel
     IplImage *cvBluePlane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane2;           // image of the blue channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane3;           // image of the blue channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane;          // image of the yellow channel
     IplImage *cvYellowPlane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane2;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlus;              // positive gaussian-convolved red image 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redMinus;             // negative gaussian-convolved red image
     IplImage *cvRedPlus;
     IplImage *cvRedMinus;
     CvMat *kernel;
    float gK[4][5][5];


    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlus;            // positive gaussian-convolved green image 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenMinus;           // negative gaussian-convolved green image
     IplImage *cvGreenPlus;
     IplImage *cvGreenMinus;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlus;             // positive gaussian-convolved red image 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowMinus;          // negative gaussian-convolved red image
     IplImage *cvBluePlus;
     IplImage *cvYellowMinus;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *redGreen;             // colour opponency map (R+G-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenRed;             // colour opponency map (G+R-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *blueYellow;           // colour opponency map (B+Y-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *edges;                // edges of colour opponency maps 

    IplImage* redG;
    IplImage* greenR;
    IplImage* blueY;
    IplImage* totImg;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;       // input port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imagePortOut;     // output port   
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortExt;      // extended image port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > rgPort;           // Colour opponency map R+G-
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > grPort;           // Colour opponency map G+R-
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > byPort;           // Colour opponency map B+Y-

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > pyImgPort;        // image after pyramid approach

    //IplImage for horizontal and vertical components after Sobel operator on color opponents
    IplImage* hRG;
    IplImage* vRG;
    IplImage* hGR;
    IplImage* vGR;
    IplImage* hBY;
    IplImage* vBY;

    //16 bit image to avoid overflow in Sobel operator
    IplImage* tempHRG;
    IplImage* tempVRG;
    IplImage* tempHGR;
    IplImage* tempVGR;
    IplImage* tempHBY;
    IplImage* tempVBY;

    //down-sampled and up-sampled images for applying Gabor filter 
    IplImage* dwnSample2;
    IplImage* dwnSample4;
    IplImage* dwnSample8;
    IplImage* dwnSample2Fil;
    IplImage* dwnSample4Fil;
    IplImage* dwnSample8Fil;
    IplImage* upSample2;
    IplImage* upSample4;
    IplImage* upSample8;

    /******************/
    //down-sampled and up-sampled images for applying Gabor filter 
    IplImage* dwnSample2a;
    IplImage* dwnSample4a;
    IplImage* dwnSample8a;
    IplImage* dwnSample2Fila;
    IplImage* dwnSample4Fila;
    IplImage* dwnSample8Fila;
    IplImage* upSample2a;
    IplImage* upSample4a;
    IplImage* upSample8a;
    //down-sampled and up-sampled images for applying Gabor filter 
    IplImage* dwnSample2b;
    IplImage* dwnSample4b;
    IplImage* dwnSample8b;
    IplImage* dwnSample2Filb;
    IplImage* dwnSample4Filb;
    IplImage* dwnSample8Filb;
    IplImage* upSample2b;
    IplImage* upSample4b;
    IplImage* upSample8b;

    //yarp image of final added image after pyramid approach
    yarp::sig::ImageOf<yarp::sig::PixelMono>* pyImage;
    

    

    IplImage* intensityImage;
    IplImage* filteredIntensityImage;
    IplImage* filteredIntensityImage1;
    IplImage* filteredIntensityImage2;
    IplImage* totImage;
    IplImage* dwnImage;

    yarp::os::Stamp St;

    
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized

    int minVal;

public:
    /**
    * constructor
    */
    visualFilterThread();

    /**
     * destructor
     */
    ~visualFilterThread();

    bool threadInit();     
    void threadRelease();
    void run(); 
    void onStop();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that resizes the necessary and already allocated images
    * @param width width of the input image
    * @param height height of the input image
    */
    void resize(int width, int height);

    /**
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param origImage originalImage
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* extender(yarp::sig::ImageOf<yarp::sig::PixelRgb>* origImage,int extDimension); 

    /**
    * extracting RGB and Y planes
    */
    void extractPlanes();

    /**
    * function that filters the input image in time 
    */
    void filterInputImage();

    /**
    * gaussing filtering of the of RGBY
    */
    void filtering();

    /**
    * function which constructs colourOpponency maps
    */
    void colourOpponency();

    /**
    * applying sobel operators on the colourOpponency maps and combining via maximisation of the 3 edges
    */
    void edgesExtract();

    void getKernels();

    void setPar(int ,double);

    void downSampleImage(IplImage*, IplImage* ,int);

    void upSampleImage(IplImage*, IplImage* ,int);

    void downSampleMultiScales(IplImage* );

    void upSampleMultiScales(IplImage* );

    void addImages(IplImage**, int ,IplImage*, float*);

    void maxImages(IplImage**, int ,IplImage*, float*);
    
    void openCVtoYARP(IplImage* ,yarp::sig::ImageOf<yarp::sig::PixelMono>*, int); 
};

#endif  //_VISUAL_FILTER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

