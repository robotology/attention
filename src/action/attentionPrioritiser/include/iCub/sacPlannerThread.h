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
 * @file sacPlannerThread.h
 * @brief RateThread which collects gaze request from the lower level as commands and foward those to the arbiter
 * 
 */

#ifndef _SAC_PLANNER_THREAD_H_
#define _SAC_PLANNER_THREAD_H_

// standard includes
#include <iostream>
#include <string>

// opencv includes
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>  

// yarp includes
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/all.h>

// Log-Polar includes 
#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>

//within project includes
#include <iCub/observer.h>
#include <iCub/observable.h>

class sacPlannerThread : public yarp::os::Thread, public observable{
private:
    
    std::string name;       // rootname of all the ports opened by this thread
    yarp::os::BufferedPort<yarp::os::Bottle> inCommandPort;     // port where all the low level commands are sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inImagePort;
    iCub::logpolar::logpolarTransform trsfC2L;                  // reference to the converter for logpolar transform frmo cartesian to logpolar
    iCub::logpolar::logpolarTransform trsfL2C;                  // reference to the converter for logpolar transform frmo cartesian to logpolar
    bool sleep;                                                 // flag set after the prediction is memorised
    bool compare;                                               // flag that indicates when it is checking the difference between predicted saccadic image with the current saccadic image
    bool idle;                                                  // flag that inhibith the computation when a saccade process is performed
    bool correctionReady;                                        // flag that indicates when an alternative direction is ready
    
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* inputImage;                                     // reference to the input image for saccadic adaptation    
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* predictedImage;                                 // reference to the predicted image for saccadic adaptation    
    
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* outputImageUp;                           // image of the alternative saccadic event (up)      
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* outputImageDown;                         // image of the alternative saccadic event (down) 
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* outputImageLeft;                         // image of the alternative saccadic event (left) 
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* outputImageRight;                        // image of the alternative saccadic event (right) 
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* intermImage;
    yarp::sig::ImageOf <yarp::sig::PixelRgb>* intermImage2;
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > corrPort;                //output port for representing the correlation measure in the adaptation
    yarp::os::Semaphore mutex;
    yarp::os::Semaphore correctionMutex;                                               // mutex for the correctionFlag

    double corrValue;                                           // correlation value between the predicted image and the current 
    double direction;                                              // direction of the more likely increment in correlation
    int countDirection;                                         // counter of the selected directions
    int rho, theta;                                             // position in the log-polar space of the desired saccadic goal

public:
    /**
    * default constructor
    */
    sacPlannerThread();

    /**
     * destructor
     */
    ~sacPlannerThread();
    
    /**
    * default constructor
    */
    sacPlannerThread(std::string moduleName);
    
    /**
    * function that initialise the thread
    */
    bool threadInit();

    /**
    * function called when the thread is stopped
    */
    void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    void run(); 

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();

    /**
    * function which is automatically executed when the stop function of the thread is called
    */
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
     * function that wakes the planner up changing the sleep flage from true to false
     */
    void wakeup(){mutex.wait();sleep = false;mutex.post();printf("waking up \n"); };

    /**
    * function that allocate the reference to the input magic 
    * @param ref pointer to the retina input image
    */
    void referenceRetina( yarp::sig::ImageOf<yarp::sig::PixelRgb>* ref);

    /** 
     * function that set the saccadic target for correction and fast cuncurrent planning in logpolar space
     * @param rho coordinate in the logpolar image
     * @param theta coordinate in the logpolar image
     */
    void setSaccadicTarget(int rho, int theta);

    /**
     * @brief: function that set the compare flag.
     * The compare flag set by this function enable/disable the control of the saccadic position reached
     * @param value value of the boolean flag that has be to assigned
     */
    void setCompare(bool value) {mutex.wait(); compare = value; mutex.post(); };    
    

    /**
     *   function that shifts the Region of Interest
     *  @param inImg input image to shift
     *  @param output of the shift operation
     *  @param x position on columns
     *  @param y position on rows
     */
    void shiftROI(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inImg,yarp::sig::ImageOf<yarp::sig::PixelRgb>* outImg, int x, int y);
    
    
    /**
     * function that calculate the corr.value of two logpolar images
     * providing as well step and shift of the image
     * @param imgA first image
     * @param imgB second image
     * @param step jumping of the pixel to reduce the computation demand (default value 1)
     * @return the pointer to the list of correlation function
     */
    void logCorrRgbSum(yarp::sig::ImageOf<yarp::sig::PixelRgb>* imgA, yarp::sig::ImageOf<yarp::sig::PixelRgb>* imgB,double* pointerCorr, int step = 1 );

    /**
     * function that resize the images once the dimension of the input image are known 
     * @param logwidth width dimension of the log-polar input image
     * @param logheight height dimension of the log-polar input image
     */ 
    void resizeImages(int logwidth, int logheight);


    /**
     * function that return the direction that is more likely to increase the correlation
     * @return angle in degree of the direction
     */
    double getDirection(){return direction;};

   
     /**
     * function that return the direction that is more likely to increase the correlation
     * @return angle in degree of the direction
     */
    double getCorrValue(){return corrValue;};
    
    /**
     * function that returns the most likely direction that is more likely to increase the correlation
     * @return angle in degree of the direction
     */
    double getCorrection(){if(correctionReady ) return direction; else return -1.0; };

    
};

#endif  //_SAC_PLANNER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

