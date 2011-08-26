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
 * @file earlyVisionThread.h
 * @brief Definition of a thread that receives images and does the computation for the
 * early vision module via two other threads (see earlyVisionModule.h).
 */

#ifndef _VISUAL_FEATURE_THREAD_H_
#define _VISUAL_FEATURE_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Stamp.h>
/* Log-Polar includes */
#include <iCub/RC_DIST_FB_logpolar_mapper.h>
//#include <yarp/sig/IplImage.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <gsl/gsl_math.h>
#include <gsl/gsl_fft_complex.h>
#include <gsl/gsl_sort_double.h>
#include <gsl/gsl_statistics.h>


#include <iCub/logPolar.h>

#include <iCub/convolve.h>
#include <iCub/config.h>
#include <iCub/centerSurround.h>
#include <iCub/chrominanceThread.h>
#include <iCub/edgesThread.h>


//#include <Eigen/Dense>



#define MONO_PIXEL_SIZE 1

// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif
 

class earlyVisionThread : public yarp::os::RateThread 
{
private:
    

    //int psb;
    int width_orig, height_orig;        // dimension of the input image (original)
    int width, height;                  // dimension of the extended input image (extending)
    int width_cart, height_cart;        // dimension of the cartesian width and height    
    float lambda;                       // costant for the temporal filter

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* filteredInputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* extendedInputImage;
    
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Rplus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Rminus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Gplus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Gminus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Bplus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Bminus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Yminus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *YofYUV;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *tmpMonoLPImage;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> *tmpMono16LPImage;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> *tmpMono16LPImage1;
    yarp::sig::ImageOf<yarp::sig::PixelMono16> *tmpMono16LPImage2;
    
    
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborPosHorConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborPosVerConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborNegHorConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborNegVerConvolution;
        
    yarp::sig::ImageOf<yarp::sig::PixelMono>* intensImg;              //yarp intensity image
    yarp::sig::ImageOf<yarp::sig::PixelMono>* unXtnIntensImg;              //yarp intensity image
    
  
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane;             // image of the red channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane;           // image of the green channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane;            // image of the blue channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane;          // image of the yellow channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Yplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Uplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Vplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *unXtnYplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *unXtnUplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *unXtnVplane;
    

    double gK[4][KERNEL_ROW][KERNEL_COL];


    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > intenPort;  
        
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > colorOpp1Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > colorOpp2Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > colorOpp3Port;   
        
    bool isYUV;   
    
    yarp::os::Stamp St;

    
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized   
    
public:
    /**
    * constructor
    */
    earlyVisionThread();

    /**
     * destructor
     */
    ~earlyVisionThread();

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
    * function that resizes the cartesian image
    * @param width width of the input image
    * @param height height of the input image
    */
    void resizeCartesian(int width, int height);

    /**
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param origImage originalImage
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    void extender(int extDimension); 

    void extender(yarp::sig::ImageOf<yarp::sig::PixelMono>* origImage,int extDimension);

     /**
    * function that maps logpolar image to cartesian
    * @param cartesianImage cartesian image to remap
    * @param logpolarImage  result of the remapping
    */
    void cartremap(yarp::sig::ImageOf<yarp::sig::PixelRgb>* cartesianImage,yarp::sig::ImageOf<yarp::sig::PixelRgb>* logpolarImage);

    /**
    * function that filters the input image in time 
    */
    void filterInputImage();

    /**
    * extracting RGB and Y planes
    */
    void extractPlanes();


    /**
    * gaussing filtering of the of image planes extracted
    */
    void filtering();

   
    /**
    * Creating color opponency maps
    */
    void colorOpponency();   

    edgesThread *edThread;
    chrominanceThread *chromeThread;    
    
};

#endif  //_VISUAL_FEATURE_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

