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

#include <iCub/MotionProfile.h>

class handProfilerThread : public yarp::os::RateThread,  public yarp::dev::CartesianEvent{
protected:
    
    static yarp::sig::Vector xZero;  

    int count;                     // counter for execution cycles
    int inputWidth;                // width of the input image
    int inputHeight;               // height of the input image
    int outputWidth;               // width of the output image
    int outputHeight;              // height of the output image
    int startup_context_id;        // memorizing the context
    int yawDof;
    int rollDof;
    int pitchDof;                  // 
    int originalContext;           // original context for the gaze Controller
    int blockNeckPitchValue;
    short widthRatio;              // ratio between the input and output image
    short heightRatio;             // ratio between the input and output image
    double timeEnd;                //
    double timeStart;              //
    double t;
    double t0;
    double t1;
    
    bool firstIteration;           // flag indicating the first iteration  
    bool idle;                     // flag indicating if the thread is in idle
    bool simulation;               // flag indicating whether the movement is simulation or executed
    bool gazetracking;             // flag indicating whether the gaze should follow hand move
    
    yarp::sig::Vector x;           // vector representating the desired position for the hand
    yarp::sig::Vector o;           // vector representating the desired position for the hand
    yarp::sig::Vector xd;          // vector representating the desired position for the hand
    yarp::sig::Vector od;          // vector representating the desired orientation for the hand
    yarp::sig::Vector xdhat;       // vector representating the desired orientation for the hand
    yarp::sig::Vector odhat;       // vector representating the desired orientation for the hand
    yarp::sig::Vector qdhat;       // vector representating the desired orientation for the hand
    
    yarp::dev::PolyDriver client;
    yarp::dev::ICartesianControl *icart;
    yarp::dev::CartesianEvent *ce;
    yarp::dev::IGazeControl *igaze;                 // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;          // polydriver for the gaze controller
    profileFactory::MotionProfile *mp;               

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* outputImage;

    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    yarp::os::BufferedPort<yarp::os::Bottle>  guiPort;                                  // output port to plot event
    yarp::os::BufferedPort<yarp::os::Bottle>  xdPort;                                   // output port to plot event
    yarp::os::BufferedPort<yarp::os::Bottle>  velPort;                                   // output port to plot event

    std::string name;                                                                   // rootname of all the ports opened by this thread

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
    
    /**
    * function executed after start
    */
    virtual void afterStart(bool s);

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

    /**
    * function that returns sets the gazeTracking either ON or OFF
    */
    void setGazeTracking(bool value){ gazetracking = value; };

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

        /*
    * function that sets the inputPort name
    */
    void setTorsoDof(const int _yawDof, const int _rollDof, const int _pitchDof ) {
        yawDof = _yawDof; rollDof = _rollDof; pitchDof = _pitchDof;
    }

    /**
     * function that sets the dimension of the output image
     */
    void setOutputDimension(int width, int height) {
        outputWidth  = width;
        outputHeight = height;
    }
  
    /**
    * function that generates the motionProfile
    * @param b bottle containing the necessary information for the profile
    */
    bool factory(const std::string type, const yarp::os::Bottle b);

    /**
    *   function that reset the execution
    */
    bool resetExecution();

    /**
    *  function that start the execution from A->B->C
    *  @param reverse indicates whether ther execution is reverse C->B->A
    */
    bool startExecution(const bool reverse);

    /**
    *  function that start the execution from A->B->C
    *  @param reverse indicates whether the simulation is reverse C->B->A
    */
    bool startSimulation(const bool reverse);

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
    * @return true/false if newtarget/notarget
    */
    bool generateTarget();

    /**
     * function thatrotates the OAB triple around x
     */
    void rotAxisX(const double& angle);

    /**
     * function thatrotates the OAB triple around y
     */
    void rotAxisY(const double& angle); 

    /**
     * function thatrotates the OAB triple around z
     */
    void rotAxisZ(const double& angle);

    /**
    * function that shows the target in time in the iCubGui
    */
    void displayTarget();

    /**
    * function that display the important via points of the profile
    */
    void displayProfile();

    /**
    * function that prints out the status of the performer.
    */
    void printStatus();

    /**
    * function that prints out the desired location to track
    */
    void printXd();
    
    /**
    * function that prints out the linearVelocity of the end-effector
    */
    void printVel();
};

#endif  //_HAND_PROFILER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

