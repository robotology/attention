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
#include <iCub/FingerProfile.h>

class handProfilerThread : public yarp::os::RateThread,  public yarp::dev::CartesianEvent{
protected:

    static yarp::sig::Vector xZero;

    int icartContext;
    int count;                     // counter for execution cycles
    int fileCounter;               // counter for file naming
    int inputWidth;                // width of the input image
    int inputHeight;               // height of the input image
    int outputWidth;               // width of the output image
    int outputHeight;              // height of the output image
    int startup_context_id;        // memorizing the context
    int yawDof;
    int rollDof;
    int pitchDof;
    int originalContext;           // original context for the gaze Controller
    int blockNeckPitchValue;
    int njoints;                   // number of joints for saving to file (left_arm)s
    int sampleNumber;              // number of samples read from file
    int infoSamples;               // numbero of samples save to file
    int repsNumber;                // number of repetitions to perform from file
    bool graspDone;
    short widthRatio;              // ratio between the input and output image
    short heightRatio;             // ratio between the input and output image
    double t;
    double t0;
    double t1;
    double speedFactor;           // factor to regulate speed in movement from file
    double partnerStart;          // variables to set the duration from MARK and SYNC commands
    double partnerStop;
    double partnerTime;
    double movementDuration;      // duration of movement read from file
    double firstDuration;         // time of the first sample for saving to file

    bool verbosity;                // flag indicating verbosity
    bool firstIteration;           // flag indicating the first iteration
    bool idle;                     // flag indicating if the thread is in idle
    bool gazetracking;             // flag indicating whether the gaze should follow hand move
    bool saveOn;                   // flag indicating if saving is enabled or not
    bool graspOn;                  // flag indicating if grasping is enabled or not

    int graspNumber;
    yarp::sig::Vector fingerJoints;
    yarp::sig::Vector fd;

    yarp::sig::Vector x;           // vector representating the desired position for the hand
    yarp::sig::Vector o;           // vector representating the desired position for the hand
    yarp::sig::Vector xd;          // vector representating the desired position for the hand
    yarp::sig::Vector xdHome;      // vector representating the desired position for the hand (HOME)
    yarp::sig::Vector xdGazeHome;  // vector representating the desired position for the hand (GAZE HOME)
    yarp::sig::Vector od;          // vector representating the desired orientation for the hand
    yarp::sig::Vector odHome;      // vector representating the desired orientation for the hand (HOME)
    yarp::sig::Vector xdhat;       // vector representating the desired orientation for the hand
    yarp::sig::Vector odhat;       // vector representating the desired orientation for the hand
    yarp::sig::Vector qdhat;       // vector representating the desired orientation for the hand
    yarp::sig::Vector startPos;           // vector with first position of hand for play from file
    yarp::sig::Vector startOri;           // vector with first position of hand for play from file
    yarp::sig::Vector firstPos;           // vector with first position of hand for save to file
    yarp::sig::Vector firstOri;           // vector with first position of hand for save to file
    yarp::sig::Vector jointsToSave;       // vector with all joints values for save to file
    yarp::sig::Vector graspHome;          // vector with home joint position for the hand
    yarp::sig::Vector graspFinal;         // vector with final joint position for the hand
    yarp::sig::Vector graspCurrent;

    yarp::dev::PolyDriver client;
    yarp::dev::PolyDriver robotDevice;
    yarp::dev::IPositionDirect *idir;               // Position DIrect for movement from file
    yarp::dev::IEncoders *encs;                     // to read encoders values
    yarp::dev::IControlMode2 *ictrl;                // to set control mode of the joints
    yarp::dev::ICartesianControl *icart;            // cartesian controller to generate and move
    yarp::dev::CartesianEvent *ce;
    yarp::dev::IGazeControl *igaze;                 // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;          // polydriver for the gaze controller
    profileFactory::MotionProfile *mp;
    fingerFactory::FingerProfile fp;

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber
    std::string part;               // robot part to move

    yarp::os::ConstString filePath;                               // path of the file found by the resource finder
    yarp::os::ResourceFinder rf;                                  // resource finder
    yarp::os::BufferedPort<yarp::os::Bottle>  guiPort;            // output port to plot event
    yarp::os::BufferedPort<yarp::os::Bottle>  xdPort;             // output port to plot event
    yarp::os::BufferedPort<yarp::os::Bottle>  velPort;            // output port to plot event
    yarp::os::BufferedPort<yarp::os::Bottle>  errPort;            // output port to plot event
    std::string name;                                             // rootname of all the ports opened by this thread
    std::string fileName;                                         // name of the file to load for movement

    std::ofstream outputFile;                                     // file in which to save joints values
    std::ofstream infoOutputFile;                                 // file in which to save info about the movement
    std::ifstream inputFile;                                      // file to read joint positions from
    std::ifstream infoInputFile;                                  // file to read with info about the movement
    enum States {none, simulation, execution, file, home};              // states for the controller
    States state;
    yarp::os::Stamp* timestamp;                                   // timestamp for saving to file

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
    handProfilerThread(std::string robotname,std::string configFile, yarp::os::ResourceFinder rf);

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
    * @param str rootname
    */
    void setName(std::string str);

    /*
    * function that loads a file for movement performing
    * @param str name of the file without extension
    */
    void loadFile(std::string str);

    /**
     * function that sets the robot arm to move
     */
    void setPart(std::string str);

    /**
     * function that sets the orientation of the endeffector
     */
    bool setOrientation(const yarp::sig::Vector vectorOrientaion);

    /**
     * function that sets the orientation of the endeffector
     */
    bool setCurrentOrientation();

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
    * function that sets the number of repetitions to perform
    */
    void setRepsNumber(int n);

    /**
    * function that sets the grasp on or off
    */
    void setGrasp(bool grasp);

     /**
     * function that sets the variable partnerStart
     */
    void setPartnerStart();

    /**
     * function that sets the variable partnerStop
     */
    void setPartnerStop();

    /**
     * function that sets the variable partnerTime
     */
    void setPartnerTime(double time);

    /**
    * function that generates the motionProfile
    * @param b bottle containing the necessary information for the profile
    */
    bool factory(const std::string type, const yarp::os::Bottle b);

    /**
    *function that resets the fingers position
    */
    void graspReset();

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
    *  function that set parameters for saving in a file the value of joints
    */
    bool saveJoints();

    /**
    *  function that saves in an array vaues of joints
    */
    void saveToArray();

    /**
    *  function that saves in a file the value of joints
    */
    void saveToFile();

    /**
    *  function that saves in a file the info of a movement
    */
    void saveInfo();

    /**
    * @brief function that set parameters for starting movement from a file with joint values
    * @param factor to divide by execution time to speed up or slow down a movement from file
    */
    bool startJoints(double factor);

    /**
    *  function that starts movement from a file with joint values
    */
    void startFromFile();

    /**
    * @brief function that starts movement from a file with joint values
    * @return boolean indicating if the resetting was successful
    */
    bool startResetting();

     /**
    *  function that plays movement from a file with joint values
    */
    void playFromFile();

    /**
     * function that perfoms downsampling (if necessary)
     */
    void processing();

    /**
    * limiting the torso pitch
    */
    void limitTorsoPitch();

    /**
    * limiting the torso pitch
    */
    void limitTorsoYaw();

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
    * function that prints out the status of the performer.
    */
    void printErr();

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
