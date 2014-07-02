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


#ifndef _EYE_INTERPRETER_H_
#define _EYE_INTERPRETER_H_

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


class eyeInterpreterThread : public yarp::os::Thread {
private:
    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;

    inputCBPort inputCbPort;  // buffered port listening to images through callback
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputCallbackPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPort;     // output port to plot event
    std::string name;                                                                // rootname of all the ports opened by this thread
    
public:
    /**
    * constructor default
    */
    eyeInterpreterThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    eyeInterpreterThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~eyeInterpreterThread();

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

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);



};

#endif  //_EYE_INTERPRETER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

