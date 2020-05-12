// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2014  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file repeaterThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _IKART_FOLLOWER_H_
#define _IKART_FOLLOWER_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <iostream>
#include <fstream>
#include <time.h>



class inputCBPort : public yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > {

public:
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;
    bool hasNewImage;

    inputCBPort() {
        inputImage = new  yarp::sig::ImageOf<yarp::sig::PixelRgb>; 
    }

    ~inputCBPort() {
        delete inputImage;
    }

    virtual void onRead(yarp::sig::ImageOf<yarp::sig::PixelRgb>& inputImage1) {
        //printf("Callback for the new image received! \n");
        inputImage = &inputImage1;
        hasNewImage = true;             // to be set to false once done with the events
    }
};


class iKartFollowerThread : public yarp::os::Thread {
private:
    int counter;                    // cycle counter
    int forward;                    // direction of the forward drive
    int countVal;                   // counter of the double values acquired
    int targetInfoPointer;          //

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    double* val;                    // value read from the targetFile
    
    double heading;                 //the commanded linear direction of the iKart, expressed in degrees
    double linSpeed;                //the commanded linear speed, expressed in m/s.
    double angSpeed;                //the commanded angular speed, expressed in deg/s.
    double velocityTarget;          // velocity of the target with respect to the iKart
    double omegaTarget;             // angular velocity of the target with respect to iKart
    double positionX;               // position of the target along the x axis in the ikart frame of reference in meters
    double positionY;               // position of the target along the y axis in the ikart frame of reference in meters

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;

    inputCBPort inputCbPort;  // buffered port listening to images through callback
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    yarp::os::BufferedPort<yarp::os::Bottle>  outputPort;     // output port to plot event
    std::string name;                                                                // rootname of all the ports opened by this thread
    
    FILE* targetFile;                                         // file cointaoms the relevant characteristic of the target movement
    
public:
    /**
    * constructor default
    */
    iKartFollowerThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    iKartFollowerThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~iKartFollowerThread();

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
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);

    /**
    * function that sets the targetFile path
    */
    void setTargetFilePath(std::string tf) {
        targetFile = fopen(tf.c_str() ,"r+");
    }

    /**
     * function that computes the control according to the desired law
     */
    void computeControl();

    /**
     * function that sends the command to the iKart
     */
    void commandControl();
    
    /**
     * acquire the target receiving its velocity and speed
     */
    void acquireTarget();

    /**
     * control law based on the technique that avoids the jack knife effect
     * @param position1
     * @param position2
     * @param velocityTarget
     * @param omegaTarget
     * @param velocityIKART
     * @param omegaIKART
     */
    void jackKnifeControl(double position1, double position2, double velocityTarget, double omegaTarget, double& velocityIKART, double& omegaIKART);


    void readTargetInfo(FILE* tf);
    
};

#endif  //_IKART_FOLLOWER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

