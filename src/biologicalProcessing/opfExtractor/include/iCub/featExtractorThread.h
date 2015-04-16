// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file featExtractorThread.h
 * @brief Definition of a thread that sends frame-based representation 
 * (see featExtractorThread.h).
 */

#ifndef _FEATEXTRACTOR_THREAD_H_
#define _FEATEXTRACTOR_THREAD_H_

#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <yarp/os/Semaphore.h>
#include <yarp/os/all.h>


#include <fstream>
#include <time.h>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <cstring>

#define TH1__ 0.5
#define TH2__ 0.5
#define PTH__ 0.5


class featExtractorThread : public yarp::os::RateThread {
private:    
    int count;                            // loop counter of the thread
    int width, height;                    // dimension of the squared retina

    yarp::os::BufferedPort<yarp::os::Bottle>  outputPortDescr;      // name of output port for the descriptor vector
    yarp::os::Bottle descrBottle;                                   // name of the boottle with float
    yarp::os::Bottle contentBottle;                                 // name of the bottle with list of float

    cv::Mat Ut;
    cv::Mat Vt;
    cv::Mat Maskt;

    bool featDataready;

	yarp::os::Semaphore semColor;
	yarp::os::Semaphore semMono;
    
    yarp::os::Semaphore sem;

    std::string name;                           // rootname of all the ports opened by this thread
    bool synchronised;                          // flag to check whether the microsecond counter has been synchronised
    bool stereo;                                // flag indicating the stereo characteristic of the synchronization


    std::vector<float> descr;
    bool firstProcessing;
    cv::Mat Ut_1;
    cv::Mat Vt_1;

    cv::Mat MAGt_1;
    cv::Mat THETAt_1;
    bool computed;
    float counter;
public:
    /**
    * default constructor
    */
    featExtractorThread();
                                                                  
    /**
     * destructor
     */
    ~featExtractorThread();

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
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);

    /**
    * function that set the stereo mode for event synchronization
    * @param value boolean to be assigned
    */
    void setStereo(bool value){ stereo = value;};    
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);

     /**
     * function that copies the image in the left output
     * @param img passed input of the image to be copied
     */
    void copyImage(yarp::sig::ImageOf<yarp::sig::PixelMono>* img);

	 /**
     * function that copies the image in the left output
     * @param img passed input of the image to be copied
     */
    void copyImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>* img);

     /**
     * function that convert a Mat into a ImageOf
     * @param a passed input of the image to be converted
     * @return image which is of type ImageOf
     */
    void convertMat2ImageOf(cv::Mat a,yarp::sig::ImageOf<yarp::sig::PixelMono>* image);

     /**
     * function that copies the images
     * @param U, V, M passed input of the images to be copied
     */
    void copyAll(cv::Mat U, cv::Mat V, cv::Mat M)  ;

     /**
     * function that copies the image in the left output
     * @param img passed input of the image to be copied
     */
    void setFlag();

     /**
     * function that copies the image in the left output
     * @param img passed input of the image to be copied
     */
    void setFlagPointer(bool* flagPointer);
    /**
     * function that copies the RGB image in the left output 
     * @param img passed input of the image to be copied
     */
    //void copyLeft(yarp::sig::ImageOf<yarp::sig::PixelRgb>* img);

    /**
     * function that copies the RGB image in the right output
     * @param img passed input of the image to be copied
     */
    //void copyRight(yarp::sig::ImageOf<yarp::sig::PixelMono>* img);

    /**
     * function that copies the RGB image in the right output
     * @param img passed input of the image to be copied
     */
    //void copyRight(yarp::sig::ImageOf<yarp::sig::PixelRgb>* img);

    
    /**
     * @brief function thatset the dimension of the output image
     * @param value the dimension in pixels of the retina device
     */
    void setRetinalSize(int value) {
    }


    /**
     * @brief function that test the network and feature of the thread
     * @return the result of the analysis true/false for success/unsuccess in the test
     */
    bool test();
    

};

#endif  //_PLOTTER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

