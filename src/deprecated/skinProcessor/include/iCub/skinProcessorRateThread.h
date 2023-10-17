// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2016  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Fabio Vannucci
  * email: fabio.vannucci@iit.it
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
 * @file skinProcessorThread.h
 * @brief Definition of a thread that receives input from skin and outputs message strings on a port
 */


#ifndef _SKINPROCESSOR_RATETHREAD_H_
#define _SKINPROCESSOR_RATETHREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/PeriodicThread.h>
#include <iCub/skinDynLib/skinContactList.h>
#include <iCub/skinDynLib/skinContact.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <stdio.h>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp> 



class skinProcessorRateThread : public yarp::os::RateThread {
private:
    bool idle;                      // flag that interrupts the processing
    int c;                          // character read from keyboard
    int count[20];

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    int useLeftArm;
    int useRightArm;
    int useTorso;

    yarp::os::Bottle* inputValue;
    yarp::os::Bottle* contactList;
    int bodyPart;
    int touchedPart;
    std::vector<int> taxellList;
    std::vector<int> touchInfo;
    std::vector<double> geometricCenter;
    double avgPressure;

    cv::Mat image;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* outputImage;                    

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputImagePort;     // output port , result of the processing
    yarp::os::BufferedPort<yarp::os::Bottle> outputDataPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inputPort;
    std::string name;                                        // rootname of all the ports opened by this thread

    
public:
    /**
    * constructor default
    */
    skinProcessorRateThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    skinProcessorRateThread(std::string robotname, std::string configFile);

    /**
     * destructor
     */
    ~skinProcessorRateThread();

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

    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);

    /**
     * @brief function to set the body parts to consider in the thread
     */
    void setBodyParts(int left, int right, int torso);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);


    /**
     * @brief function of main processing in the thread
     */
    int processing();


     /**
    *  draw the rectangles for contact
    */
    void drawContact(); 
};

#endif  //_SKINPROCESSOR_RATETHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

