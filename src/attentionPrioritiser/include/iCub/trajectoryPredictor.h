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
#include <yarp/dev/all.h>
#include <iostream>
#include <string>

//within project includes
#include <iCub/trackerThread.h>
#include <iCub/observer.h>
#include <iCub/observable.h>



class trajectoryPredictor : public yarp::os::Thread, public observable{
private:
    static const int numEvalVel = 1;  // number of evaluator based on const velocity 
    static const int numEvalAcc = 1;  // number of evaluator based on const acceleration
    static const int numEvalMj  = 1;  // number of evaluator based on minimum jerk
    static const int numIter    = 10; // number of iteractions

    int originalContext;              // original context for the gaze Controller
    double Vx, Vy;                    // components of velocity
    double blockNeckPitchValue;       // value for blocking the pitch of the neck
    std::string name;                 // rootname of all the ports opened by this thread

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > inImagePort;     //port where all the low level commands are sent
    yarp::os::Semaphore mutex;       // semaphore for the variable of the prediction accomplished
    yarp::os::Semaphore mutexP0;     // semaphore for resource p0
    yarp::os::Semaphore mutexN;      // semaphore for resource n
    
    bool predictionAccompl;                         // flag that indicates when the prediction was carried on correctly
    yarp::os::ResourceFinder* rf;                   // resource finder for initialisation of the tracker
    trackerThread*    tracker;                      // reference to the object in charge of tracking a tamplete surrounding a point
    yarp::dev::IGazeControl *igaze;                 // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;          // polydriver for the gaze controller
    
    //attention::evaluator::evalThread evalVel1;      // evaluation thread velocity 1
    //attention::evaluator::evalThread evalVel2;      // evaluation thread velocity 2
    //attention::evaluator::evalThread evalAcc1;      // evaluation thread acceleration 1
    //attention::evaluator::evalThread evalAcc2;      // evaluation thread accelaration 2
    //attention::evaluator::evalThread evalMJ1_T1;    // evaluation thread minJerk distance 1 - period 1
    //attention::evaluator::evalThread evalMJ2_T1;    // evaluation thread minJerk distance 2 - period 1
    //attention::evaluator::evalThread evalMJ1_T2;    // evaluation thread minJerk distance 1 - period 2
    //attention::evaluator::evalThread evalMJ2_T2;    // evaluation thread minJerk distance 2 - period 2

    attention::evaluator::evalQueue* eQueue;        // queue of evaluation threads
    attention::evaluator::evalThread* eval; 
    
    yarp::sig::Matrix zMeasure;                     // vector of measurements
    yarp::sig::Matrix uMeasure;                     // vector of the input values
    yarp::sig::Vector n;                            // vector normal to the plane
    yarp::sig::Vector p0;                           // point belonging to the plane

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
     * function that adds the evaluation thread to the list
     */
    void addEvalThread(attention::evaluator::evalThread* et){ 
        printf("trajectoryPredictor::addEvalThread %08X \n", et);
        Vector x = et->getX();
        printf("trajectoryPredictor::addEvalThread: x = \n %s \n", x.toString().c_str());
        eQueue->push_back(et); 
    };

    /**
     * @brief function that estimate the velocity of the centroid in time
     * @param Vx estimated velocity along x axis
     * @param Vy estimated velocity along y axis
     * @param xPos estimated landing location along x axis (return)
     * @param yPos estimated landing location along x axis (return)
     * @param time estimated time length of movement       (return)
     * @param distance of the stimulus from the fovea      (return)
     */
    bool estimateVelocity(int x, int y, double& Vx, double& Vy, double& xPos, double& yPos, double& time, double& distance);

    
    /**
     * @brief project on plane (a, b, c, d) the point (u, v) belonging to the image plan
     * @param a plane param
     * @param b plane param
     * @param c plane param
     * @param d plane param
     * @param u coordinate on the image plane
     * @param v coordinate on the image plane
     * @return Vector of the cordinate of the plane
     */
    yarp::sig::Vector projectOnPlane(int a, int b, int c , int d, int u, int v);
};

#endif  //_TRAJECTORY_PREDICTOR_H_

//----- end-of-file --- ( next line intentionally left blank ) --make// -
// ---------------

