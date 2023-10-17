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
 * @file edgesThread.h
 * @brief Definition of a thread that extracts edges based on Sobel filter (see earlyVisionModule.h).
 */

#ifndef _EDGES_THREAD_H_
#define _EDGES_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Stamp.h>
/* Log-Polar includes */
#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>
//#include <yarp/sig/IplImage.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>


//#include <iCub/logPolar.h>

#include <iCub/convolve.h>
#include <iCub/config.h>



//#include <Eigen/Dense>


#ifndef PI
#define PI 3.1415926535897932384626433832795
#endif

#define MONO_PIXEL_SIZE 1

// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif


class edgesThread : public yarp::os::PeriodicThread{

private:
    

    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,SobelOutputImage ,SobelOutputImagePtr >*
sobel2DXConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,SobelOutputImage ,SobelOutputImagePtr >*
sobel2DYConvolution;    
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > edges;    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > debugOutPort;    
    
    int widthLP;         // original width of logpolar image 
    int heightLP;
    int widthUnXnt;
    int heightUnXnt;   
    
    bool edgesThreadIsProcessing;
    bool dataReadyForEdgesThread;
    bool resized;
    int sobelIsNormalized;

    yarp::sig::ImageOf<yarp::sig::PixelMono>  *intensityImage;  // unextended intensity image 
    SobelOutputImage *tmpMonoSobelImage1;
    SobelOutputImage *tmpMonoSobelImage2;

    float sobelLimits[2];                          // maximum and minimum of Sobel operator results    
    float *dImgIn, *dImgBuff, *dImgOut;            // variables for cuda processing
    
    std::string name;       // rootname of all the ports opened by this thread    

public:
    /**
    * constructor
    */
    edgesThread();

    /**
     * destructor
     */
    ~edgesThread();

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
    * To copy relevant planes, in this case it is intensity image, which is used to extract edges via Sobel operator
    * @param I intensity image
    */
    void copyRelevantPlanes(yarp::sig::ImageOf<yarp::sig::PixelMono> *I); 

    /**
    * applying Sobel operators on the colourOpponency maps and combining via maximisation of the 3 edges
    */
    void edgesExtract();

     /* to suspend and resume the thread processing
    */
    void suspend(){
        printf("suspending edges thread\n");
        PeriodicThread::suspend();      // LATER: some sanity checks
    }

    void resume(){
        printf("resuming edges thread\n");
        PeriodicThread::resume();
    } 

    
    inline bool getFlagForDataReady(){
        
         return this->dataReadyForEdgesThread;
         
    }

    inline void setFlagForDataReady(bool v){
         //atomic operation
         dataReadyForEdgesThread = v;         
    }

    inline bool getFlagForThreadProcessing(){        
         return this->edgesThreadIsProcessing;         
    }

    inline void setFlagForThreadProcessing(bool v){
         //atomic operation
         edgesThreadIsProcessing = v;         
    }
    
    
};




#endif  //_EDGES_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

