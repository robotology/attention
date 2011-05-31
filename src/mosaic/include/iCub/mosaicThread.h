// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak, Francesco Rea
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
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iCub/iKin/iKinInv.h>
//#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/pids.h>

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
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    float* memory;                  // memory of plotted location in the space
    int countMemory;                // number of saved location
    double shiftx;                  // shift of the mosaic picture
    double shifty;                  // shift of the mosaic picture
    float azimuth, elevation;          // parameters necessary to fetch the portion of the mosaic
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImage;            // input image from camera
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *outputImageMosaic;     // output image (mosaic)
    yarp::dev::IGazeControl *igaze;         //Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;  //polydriver for the gaze controller
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;       // input port for camera 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortOut;      // output port for overlapped monochromised image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > portionPort;       // port used to send the portion of the mosaic requested
    yarp::dev::PolyDriver *polyTorso, *drvHead;             // polydriver for the control of the torso and head
    iCub::iKin::iCubEye *eyeL;
    iCub::iKin::iCubEye *eyeR;
    yarp::dev::IEncoders   *encTorso,*encHead;              // encoders of the torso and head
    yarp::sig::Matrix *invPrjL, *invPrjR;                   // inverse of prjection matrix
    yarp::sig::Matrix *PrjL, *PrjR;                         // projection matrix
    yarp::sig::Matrix *cyclopicPrj;                         // projection on the cyclopic plane  
    yarp::sig::Matrix *eyeHL, *eyeHR;                                // rototranslation matrix for the considered eye
    yarp::sig::Matrix *eyeH0;
    yarp::sig::Matrix *inveyeH0;
    float cxl, cyl, fxl, fyl;
    int count;
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized
    
public:
    /**
    * constructor default
    */
    mosaicThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    mosaicThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~mosaicThread();

    /**
    *  initialises the thread
    */
    bool threadInit();     

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run(); 

    /**
    *  on stopping of the thread
    */
    void onStop();

    /**
    *  set the dimension of the mosaic image
    */
    void setMosaicDim(int width, int height);

    /**
    *  set the dimension of the input image
    */
    void setInputDim(int width, int height);

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


    /**
     * function that adds a novel location to the list of represented locations
     * @param x x coordinate of the object
     * @param y y coordinate of the object
     * @param z z coordinate of the object
     */
    void plotObject(float x,float y, float z);

    /**
     * function that fetches a portion of the mosaic 
     * @param image portion extracted and ready to be used
     */
    void fetchPortion(yarp::sig::ImageOf<yarp::sig::PixelRgb>* image);

    /**
     * function that set a portion of the mosaic to fetch  
     * @param azimuth angle of gaze interested in
     * @param elevation angle of gaze interested in
     */    
    void setFetchPortion(float azimuth, float elevation);
    
};

#endif  //_MOSAIC_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

