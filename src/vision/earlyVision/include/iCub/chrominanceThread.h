// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea, Shashank Pathak
  * email: francesco.rea@iit.it, shashank.pathak@iit.it
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
#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>

/* openCV includes */
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>
#include <opencv2/highgui/highgui_c.h>
/*  iCub includes */
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

class chrominanceThread : public yarp::os::PeriodicThread{

private:
   
    yarp::sig::ImageOf<yarp::sig::PixelMono> *chromUnXtnIntensImg;              //yarp intensity image
    
    
    //convolve<yarp::sig::ImageOf<yarp::sig::PixelFloat>,float,yarp::sig::ImageOf<yarp::sig::PixelFloat> ,float >* gaborFiveByFive[4];
    yarp::sig::ImageOf<yarp::sig::PixelFloat>* imageAtScale[GABOR_SCALES];
    yarp::sig::ImageOf<yarp::sig::PixelFloat>* imageForAScale[GABOR_SCALES];
    yarp::sig::ImageOf<yarp::sig::PixelFloat>* gaussUpScaled[GABOR_SCALES];
    yarp::sig::ImageOf<yarp::sig::PixelFloat>* tempCSScaleOne;
    yarp::sig::ImageOf<yarp::sig::PixelMono>* imageInCartMono;
    int weightIntensityAtScale[GABOR_SCALES];
    int weightGaborAtScale[GABOR_SCALES];
        
    CvMat* gaborKernels[GABOR_ORIS];
    CvPoint anchor;
   
    // Ports to output different orientation images    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPort0;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPort45;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPort90;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > orientPortM45;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > totalOrientImagePort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > totalOrientCartImgPort;
    

    yarp::sig::ImageOf<yarp::sig::PixelMono>* cartIntensImg;          //yarp cartesian intensity image for orientation
    yarp::sig::ImageOf<yarp::sig::PixelMono>* cartOri0;               //yarp cartesian for orientation 0
    yarp::sig::ImageOf<yarp::sig::PixelMono>* cartOri45;              //yarp cartesian for orientation 45
    yarp::sig::ImageOf<yarp::sig::PixelMono>* cartOri90;              //yarp cartesian for orientation 90
    yarp::sig::ImageOf<yarp::sig::PixelMono>* cartOriM45;             //yarp cartesian for orientation M45
        
    //iCub::logpolar::logpolarTransform trsf; //reference to the converter for logpolar transform
    
    logpolarTransformVisual lpMono;
    int xSizeValue;         // x dimension of the remapped cartesian image
    int ySizeValue;         // y dimension of the remapped cartesian image
    double overlap;         // overlap in the remapping
    int numberOfRings;      // number of rings in the remapping
    int numberOfAngles;     // number of angles in the remapping       

    int widthLP;            // original width of logpolar image
    int widthCr;
    int heightLP;
    int heightCr;
    
    //IppiSize srcsize, origsize;   
    
    bool resized;
    bool dataReadyForChromeThread;
    bool chromeThreadProcessing;
    
    std::string name;           // rootname of all the ports opened by this thread
    
    //Deprecated
    double wtForEachOrientation[GABOR_ORIS];
    float brightness;
    
   
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
    * function that copies the images from the main thread
    * @param I intensity image
    */
    void copyScalesOfImages(yarp::sig::ImageOf<yarp::sig::PixelMono> *I, CvMat **toBeCopiedGauss);

    /**
    * function that set the value for the weight horizontal orientation in linear combination
    * @param value double value of the weight
    */
    void setWHorizontal(double value) { wtForEachOrientation[0] = value; };

    /**
    * function that set the value for the weight vertical orientation in linear combination
    * @param value double value of the weight
    */
    void setWVertical(double value) { wtForEachOrientation[2] = value; };

    /**
    * function that set the value for the weight 45 degrees orientation in linear combination
    * @param value double value of the weight
    */
    void setW45Degrees(double value) { wtForEachOrientation[1] = value; };

    /**
    * function that set the value for the weight minus 45 degrees orientation in linear combination
    * @param value double value of the weight
    */
    void setWM45Degrees(double value) { wtForEachOrientation[3] = value; };
    
    /**
    * Orientation using Gabor kernel on Gaussian smoothened intensity. (Refer config.h)
    */
    void orientation();    

    /* to suspend and resume the thread processing
    */
    void suspend(){
        printf("suspending chrome thread\n");
        PeriodicThread::suspend();      // LATER: some sanity checks
    }

    void resume(){
        printf("resuming chrome thread\n");
        PeriodicThread::resume();
    }   
   
    
    /**
     *   A handy get-set method
     */
    inline yarp::sig::ImageOf<yarp::sig::PixelMono>* getCartesianImage(){
        return this->cartIntensImg;
    }

    /**
     *   A handy get-set method
     */
    inline bool getFlagForDataReady(){
        
         return this->dataReadyForChromeThread;
         
    }

    /**
     *   A handy get-set method
     */
    inline void setFlagForDataReady(bool v){
         //atomic operation
         dataReadyForChromeThread = v;         
    }
    
    /**
     *   A handy get-set method
     */
    inline bool getFlagForThreadProcessing(){
        
         return this->chromeThreadProcessing;
         
    }

    /**
     *   A handy get-set method
     */
    inline void setFlagForThreadProcessing(bool v){
         //atomic operation
         chromeThreadProcessing = v;         
    } 

    /**
     *   A handy get-set method
     */
    inline void setWeightForOrientation(int orientNbr, float val){
        assert(orientNbr>=0 && orientNbr< GABOR_ORIS);
        wtForEachOrientation[orientNbr] = val;
    }
    
    /**
     *   A handy get-set method
     */
    inline float getWeightForOrientation(int orientNbr){
        assert(orientNbr>=0 && orientNbr< GABOR_ORIS);
        return wtForEachOrientation[orientNbr];
    }

    /**
     *   A handy get-set method
     */
    inline void setBrightness(float val){
        this->brightness = val;
        printf("received brightness val%lf and set to%f\n",val,this->brightness);        
    }

    /**
     *   A handy get-set method
     */
    inline float getBrightness(){
        return this->brightness;
    }

    /**
     * @brief function that returns the value in fovea of a particular orientation map
     * @param angle angle that indicates with feature map is going to be exploited
     */
    double getFoveaOri(int angle);
    
    
};

#endif  //_CHROME_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

