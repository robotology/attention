// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2015  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file handProfilerThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _HAND_PROFILER_THREAD_H_
#define _HAND_PROFILER_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/math/Math.h>

#include <gsl/gsl_math.h>

#include <iostream>
#include <fstream>
#include <time.h>
#include <cv.h>
#include <stdio.h>

class handProfilerThread : public yarp::os::RateThread,  public yarp::dev::CartesianEvent{
protected:
    int inputWidth;                // width of the input image
    int inputHeight;               // height of the input image
    int outputWidth;               // width of the output image
    int outputHeight;              // height of the output image
    int startup_context_id;        // memorizing the context
    short widthRatio;              // ratio between the input and output image
    short heightRatio;             // ratio between the input and output image
    double timeEnd;                //
    double timeStart;              //
    double t;
    double t0;
    double t1;  

    yarp::sig::Vector xd;          // vector representating the desired position for the hand
    yarp::sig::Vector od;          // vector representating the desired orientation for the hand

    yarp::dev::PolyDriver client;
    yarp::dev::ICartesianControl *icart;
    yarp::dev::CartesianEvent *ce; 

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* outputImage;

    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPort;     // output port to plot event
    std::string name;                                                                // rootname of all the ports opened by this thread

    // the event callback attached to the "motion-ongoing"
    virtual void cartesianEventCallback() {
        fprintf(stdout,"20%% of trajectory attained\n");
    }
    
public:
    /**
    * constructor default
    */
    handProfilerThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    handProfilerThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~handProfilerThread();

    /**
    *  initialises the thread
    */
    virtual bool threadInit();
    
    
    virtual void afterStart(bool s) {
        if (s)
              fprintf(stdout,"Thread started successfully\n");
          else
              fprintf(stdout,"Thread did not start\n");

          t=t0=t1=yarp::os::Time::now();
    }

    /**
    *  correctly releases the thread
    */
    virtual void threadRelease();

    /**
    *  active part of the thread
    */
    virtual void run(); 

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
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /**
     * function that sets the dimension of the output image
     */
    void setOutputDimension(int width, int height) {
        outputWidth  = width;
        outputHeight = height;
    }

    /**
     * function that perfoms downsampling (if necessary)
     */
    void processing();

    /**
    * limiting the torso pitch
    */
    void limitTorsoPitch();

    /**
    * generate the target position in space   
    */
    void generateTarget();

    /**
    * function that prints out the status of the performer.
    */
    void printStatus();
};

#endif  //_HAND_PROFILER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

