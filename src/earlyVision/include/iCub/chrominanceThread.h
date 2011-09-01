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
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <iCub/logPolar.h>
#include <iCub/convolve.h>
#include <iCub/config.h>
#include <iCub/centerSurround.h>



#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#define MONO_PIXEL_SIZE 1


#define ROW_SIZE 252
#define COL_SIZE 152
#define CART_ROW_SIZE 320
#define CART_COL_SIZE 240


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
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschListOfPosKernels;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,KirschOutputImage,KirschOutputImagePtr >*
kirschListOfNegKernels;

    // pointers to planes defined in earlyVision
    yarp::sig::ImageOf<yarp::sig::PixelMono> *chromYplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *chromUplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *chromVplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *chromUnXtnIntensImg;              //yarp intensity image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *logPolarOrientImg;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ori0;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ori45;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ori90;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *oriM45;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *oriAll;
   
    KirschOutputImage *o0;
    KirschOutputImage *o45;
    KirschOutputImage *o90;
    KirschOutputImage *oM45;
    KirschOutputImage *tmpKirschCartImage1;
    KirschOutputImage *tmpKirschCartImage2;
    KirschOutputImage *tmpKirschCartImage3;
    KirschOutputImage *tmpKirschCartImage4;
    KirschOutputImage *totalKirsch;
    KirschOutputImage *listOfPosKir[4];
    KirschOutputImage *listOfNegKir[4];

    float wtForEachOrientation[4];
    float brightness;                                   // this adjusts overall brightness 
    
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPort0;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPort45;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPort90;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPortM45;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > totalOrientImagePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > totalOrientCartImgPort;
    

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
    
    bool kirschIsNormalized;
    bool resized;
    bool dataReadyForChromeThread;
    bool chromeThreadProcessing;

    
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
    
    /**
    * function that copies the images from the main thread
    * @param I intensity image
    */
    void copyRelevantPlanes(yarp::sig::ImageOf<yarp::sig::PixelMono> *I);    

      

    /**
    * Orientation using Kirsch kernel on Gaussian smoothened intensity. (Refer config.h)
    */
    void orientation();

    void maxImages(IplImage** ImagesTobeAdded, int numberOfImages,IplImage* resultantImage); 

    /* to suspend and resume the thread processing
    */
    void suspend(){
        printf("suspending chrome thread\n");
        RateThread::suspend();      // LATER: some sanity checks
    }

    void resume(){
        printf("resuming chrome thread\n");
        RateThread::resume();
    }   
   

    /* Some handy get-set methods
    */
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

    inline bool getFlagForThreadProcessing(){
        
         return this->chromeThreadProcessing;
         
    }

    inline void setFlagForThreadProcessing(bool v){
         //atomic operation
         chromeThreadProcessing = v;         
    } 

    inline void setWeightForOrientation(int orientNbr, float val){
        assert(orientNbr>=0 && orientNbr<=3);
        wtForEachOrientation[orientNbr] = val;
    } 
    inline float getWeightForOrientation(int orientNbr){
        assert(orientNbr>=0 && orientNbr<=3);
        return wtForEachOrientation[orientNbr];
    }

    inline void setBrightness(float val){
        this->brightness = val;
    }

    inline float getBrightness(){
        return this->brightness;
    }
    
};

#endif  //_CHROME_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

