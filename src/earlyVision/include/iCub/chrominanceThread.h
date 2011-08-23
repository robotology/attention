// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
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
 * @file chrominanceThread.h
 * @brief Definition of a thread that computes chrominance and orientation (see earlyVisionModule.h).
 */

#ifndef _CHROME_THREAD_H_
#define _CHROME_THREAD_H_

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


//#include <Eigen/Dense>


#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#define MONO_PIXEL_SIZE 1


#define ROW_SIZE 252
#define COL_SIZE 152
#define CART_ROW_SIZE 320
#define CART_COL_SIZE 240

#define POS_GAUSSIAN 5
#define NEG_GAUSSIAN 7

#define RATE_OF_CHROME_THREAD 50    // 20 Hz

// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif


class chrominanceThread : public yarp::os::RateThread{ 

private:
    

    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschSalPos0;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschSalPos45;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschSalPos90;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschSalPosM45;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschSalNeg0;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschSalNeg45;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschSalNeg90;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschSalNegM45;

    // pointers to planes defined in earlyVision
    yarp::sig::ImageOf<yarp::sig::PixelMono> *chromYplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *chromUplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *chromVplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *chromUnXtnIntensImg;              //yarp intensity image
   
    KirschOutputImage *o0;
    KirschOutputImage *o45;
    KirschOutputImage *o90;
    KirschOutputImage *oM45;
    KirschOutputImage *tmpKirschCartImage1;
    KirschOutputImage *tmpKirschCartImage2;
    KirschOutputImage *tmpKirschCartImage3;
    KirschOutputImage *tmpKirschCartImage4;
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > intensityCSPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > chromPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > VofHSVPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPort0;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPort45;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPort90;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPortM45;
    

    yarp::sig::ImageOf<yarp::sig::PixelMono>* cartIntensImg;          //yarp cartesian intensity image for orientation

    

    //iCub::logpolar::logpolarTransform trsf; //reference to the converter for logpolar transform
    
    logpolarTransformVisual lpMono;
    int xSizeValue;         // x dimension of the remapped cartesian image
    int ySizeValue;         // y dimension of the remapped cartesian image
    double overlap;         // overlap in the remapping
    int numberOfRings;      // number of rings in the remapping
    int numberOfAngles;     // number of angles in the remapping

    int widthLP;         // original width of logpolar image
    int widthCr;
    int heightLP;
    int heightCr;
    
    //IppiSize srcsize, origsize;
    CenterSurround *centerSurr; 
   
    
    //Ipp8u *tmp;         //extended tmp containing alpha
    IplImage *cs_tot_32f; //extended
    IplImage *cs_tot_8u; 
    IplImage *ycs_out;     //final extended intensity center surround image
    IplImage *scs_out;     //final extended intensity center surround image
    IplImage *vcs_out;     //final extended intensity center surround image
    IplImage *colcs_out;   //final extended coulour center surround image
    int img_psb, psb4, psb, ycs_psb, col_psb, psb_32f, f_psb, s_psb, t_psb; //images rowsizes
    int ncsscale;
    bool isYUV;
    bool kirschIsNormalized;
    bool chromIsInitialized;
    bool dataReadyForChromeThread;

    yarp::sig::ImageOf<yarp::sig::PixelMono>  *img_Y;          // extended output image, also reused for hsv
	yarp::sig::ImageOf<yarp::sig::PixelMono>  *img_UV;         // extended output image, also reused for hsv
    yarp::sig::ImageOf<yarp::sig::PixelMono>  *img_V;         // extended output image, also reused for hsv
	
    
    
    
    yarp::os::Stamp St;
    

    
    std::string name;       // rootname of all the ports opened by this thread
    

    float kirschLimits[4][2];   //maximum and minimum of Kirsch operator for each orientation

public:
    /**
    * constructor
    */
    chrominanceThread();

    /**
     * destructor
     */
    ~chrominanceThread();

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

    

    void copyRelevantPlanes(yarp::sig::ImageOf<yarp::sig::PixelMono> *I,yarp::sig::ImageOf<yarp::sig::PixelMono> *Y,yarp::sig::ImageOf<yarp::sig::PixelMono> *U, yarp::sig::ImageOf<yarp::sig::PixelMono> *V);

    

   
    

    

    



    

    /**
    * Center-surrounding
    */
    void centerSurrounding();
    

    
    void orientation();

    

   


    /**
    * function which crops the image
    * @param corners int array defining the crop boundaries in (left-top,right-bottom) fashion
    * @param imageToBeCropped source image that needs to be cropped
    * @param retImage The final cropped image
    */
    void cropImage(int* corners, IplImage* imageToBeCropped, IplImage* retImage);


    /**
    * function that crops in-place a given circle (center, radius form) from source image
    * @param center center of circle in (int,int) array
    * @param radius radius of circle
    * @param srcImage source image that will be changed after cropping
    */
    void cropCircleImage(int* center, float radius, IplImage* srcImg);


    inline yarp::sig::ImageOf<yarp::sig::PixelMono>* getCartesianImage(){
        return this->cartIntensImg;
    }

    inline bool getFlagForDataReady(){
        
         return this->dataReadyForChromeThread;
         
    }

    inline void setFlagForDataReady(bool v){
         //atomic operation
         dataReadyForChromeThread = v;         
    }

    
    
    
};




#endif  //_CHROME_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

