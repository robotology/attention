// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file trajectoryPredictor.h
 * @brief thread that given as input the image of the region to be track, extract the centroid and determines velocity components and stopping position
 */

#ifndef _TRAJECTORY_PREDICTOR_H_
#define _TRAJECTORY_PREDICTOR_H_


#include <iCub/attention/predModels.h>
#include <iCub/attention/evalThread.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Semaphore.h>

#include <yarp/sig/all.h>
#include <iostream>
#include <string>

//within project includes
#include <iCub/trackerThread.h>
#include <iCub/observer.h>
#include <iCub/observable.h>



class trajectoryPredictor : public yarp::os::Thread, public observable{
private:
    static const int numEvalVel;  // number of evaluator based on const velocity 
    static const int numEvalAcc;  // number of evaluator based on const acceleration
    static const int numEvalMj ;  // number of evaluator based on minimum jerk

    double Vx, Vy;                // components of velocity
    std::string name;             // rootname of all the ports opened by this thread

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > inImagePort;     //port where all the low level commands are sent
    yarp::os::Semaphore mutex;    // semaphore for the variable of the prediction accomplished
    
    bool predictionAccompl;       // flag that indicates when the prediction was carried on correctly
    yarp::os::ResourceFinder* rf; // resource finder for initialisation of the tracker
    trackerThread*    tracker;    // reference to the object in charge of tracking a tamplete surrounding a point
    
    attention::predictor::evalThread evalVel1;          // evaluation thread velocity 1
    attention::predictor::evalThread evalVel2;          // evaluation thread velocity 2
    attention::predictor::evalThread evalAcc1;          // evaluation thread acceleration 1
    attention::predictor::evalThread evalAcc2;          // evaluation thread accelaration 2
    attention::predictor::evalThread evalMJ1_T1;        // evaluation thread minJerk distance 1 - period 1
    attention::predictor::evalThread evalMJ2_T1;        // evaluation thread minJerk distance 2 - period 1
    attention::predictor::evalThread evalMJ1_T2;        // evaluation thread minJerk distance 1 - period 2
    attention::predictor::evalThread evalMJ2_T2;        // evaluation thread minJerk distance 2 - period 2

public:
    /**
    * default constructor
    */
    trajectoryPredictor();

    /**
     * destructor
     */
    ~trajectoryPredictor();

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
     * @brief function that passes the resource finder to the class for further use of files
     * @param resourceFinder reference to the object
     */
    void setResourceFinder(yarp::os::ResourceFinder* resourceFinder) {rf = resourceFinder; }; 

    /**
     * function that sets the reference to the tracker
     */
    void setTracker(trackerThread* tt) { tracker = tt; };
    
    /**
     * @brief function that returns if the prediction has been performed
     * @param b boolean flag updated by the function
     */
    void isPredict(bool& b);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
     * function that extract the centroid coordinates of the blob in the image
     */
    void extractCentroid(yarp::sig::ImageOf<yarp::sig::PixelMono>* image, int& x, int& y);

    /**
     * @brief function that estimate the velocity of the centroid in time
     * @param Vx estimated velocity along x axis
     * @param Vy estimated velocity along y axis
     * @param xPos estimated landing location along x axis
     * @param yPos estimated landing location along x axis
     * @param time estimated time length of movement
     * @param distance of the stimulus from the fovea
     */
    bool estimateVelocity(int x, int y, double& Vx, double& Vy, double& xPos, double& yPos, double& time, double& distance);
};

#endif  //_TRAJECTORY_PREDICTOR_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

