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
 * @file mosaicThread.h
 * @brief Definition of a thread that receives images from two camera and shifts one image and overlaps on another
 * mosaic mosaic (see mosaicModule.h).
 */

#ifndef _MOSAIC_THREAD_H_
#define _MOSAIC_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/RateThread.h>

#include <iostream>

#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480
#define MAX_WIDTH 640
#define MAX_HEIGHT 480



class mosaicThread : public yarp::os::Thread {
private:
    int width_orig, height_orig;    // dimension of the input image (original)
    int width, height;              // dimension of the mosaic image 
    int xcoord, ycoord;             // position of input image's center in mosaic reference frame
    

    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImage;            // input image from camera
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *outputImageMosaic;     // output image (mosaic)
    

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;       // input port for camera 1
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortOut;      // output port for overlapped monochromised image
    
    
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized
    
public:
    /**
    * constructor
    */
    mosaicThread();

    /**
     * destructor
     */
    ~mosaicThread();

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
    * function that places the input image on mosaic
    * @param xcoord x coordinate of input image's center in mosaic image reference frame
    * @param ycoord y coordinate of input image's center in mosaic image reference frame
    * @return bool flag to confirm or deny the placing
    */
    bool placeInpImage(int xCoord, int yCoord);

    /**
    * function that sets the size of mosaic image
    * @param width width of the mosaic image
    * @param height height of the mosaic image
    * @return bool flag to confirm or deny the setting
    */
    bool setMosaicSize(int width, int height);


    /**
    * function to create mosaic image from a given input image
    * @param inputImage the input image
    */
    void makeMosaic(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inpImage);

    
};

#endif  //_MOSAIC_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

