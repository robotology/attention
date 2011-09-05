// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file earlyMotionThread.h
 * @brief Definition of a thread that receives images and does the computation for the
 * visual filter module (see earlyMotionModule.h).
 */

#ifndef _EARLY_MOTION_THREAD_H_
#define _EARLY_MOTION_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>

#include <iCub/convolve.h>


static float G5[5] = { -0.0545f,
                       -0.2442f,
                       -0.4026f,
                       -0.2442f,
                       -0.0545f
};


//gaussian kernel 9 pixels  with sigma 3
static float G9[9] = {    -0.0630,
                          -0.0929,
                          -0.1226,
                          -0.1449,
                          -0.1532,
                          -0.1449,
                          -0.1226,
                          -0.0929,
                          -0.0630
};


#define CHAR_LIMIT 256
#define KIRSCH_FLICKER 0
#define KIRSCH_SHIFT 50
#define KIRSCH_FACTOR .05

// Kirsch matrix is non-separable, these directions are 8 namely N, NE, E, ...
static float K1[9] = {+3,  	+3,  	+3,
                        +3, 	0, 	+3,
                        -5, 	-5, 	-5};
static float K2[9] = {+3,  	+3,  	+3,
                        -5, 	0, 	+3,
                        -5, 	-5, 	+3};
static float K3[9] = {  5,  	+3,  	+3,
                        -5, 	0, 	+3,
                        -5, 	+3, 	+3};
static float K4[9] = {-5,  	-5,  	+3,
                        -5, 	0, 	+3,
                        +3, 	+3, 	+3};
static float K5[9] = {-5,  	-5,  	-5,
                        +3, 	0, 	+3,
                        +3, 	+3, 	+3};
static float K6[9] = {+3,  	-5,  	-5,
                        +3, 	0, 	-5,
                        +3, 	+3, 	+3};
static float K7[9] = {+3,  	+3,  	-5,
                        +3, 	0, 	-5,
                        +3, 	+3, 	-5};
static float K8[9] = {+3,  	+3,  	+3,
                        +3, 	0, 	-5,
                        +3, 	-5, 	-5};


class earlyMotionThread : public yarp::os::Thread
{
private:

    int count;                          // iteration counter for time division
    int width_orig, height_orig;        // dimension of the input image (original)
    int width, height;                  // dimension of the extended input image (extending)
    int size1;                          // size of the buffer
    int psb16s;                         // step size of the Ipp16s vectors
    float lambda;                       // costant for the temporal filter
   
    yarp::sig::ImageOf<yarp::sig::PixelMono> *inputImage;           // input image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *inputImageFiltered;   // time filtered input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb>  *inputExtImage;        // extended input image
        
    yarp::sig::ImageOf<yarp::sig::PixelMono> *imageT1;              // image of t-1 temporal space
    yarp::sig::ImageOf<yarp::sig::PixelMono> *imageT2;              // image of t-2 temporal space
    yarp::sig::ImageOf<yarp::sig::PixelMono> *imageT3;              // image of t-3 temporal space
    yarp::sig::ImageOf<yarp::sig::PixelMono> *imageT4;              // image of t-3 temporal space
    

    yarp::sig::ImageOf<yarp::sig::PixelMono> *motion;                // edges of colour opponency maps 

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imagePortIn;    // input port
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > motionPort;     // output port   

    //convolve conv;
    
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized

public:
    /**
    * constructor
    */
    earlyMotionThread();

    /**
     * destructor
     */
    ~earlyMotionThread();

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
     * function that extracts temporal variancy informatin
     */
    void temporalSubtraction(yarp::sig::ImageOf<yarp::sig::PixelMono>* out);

    /**
     * function that stores the image in a temporal array
     */
    void temporalStore();

    
};

#endif  //_EARLY_MOTION_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

