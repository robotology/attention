// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Carlo Mazzola
  * email: carlo.mazzola@iit.it
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
 * @file tutorialPeriodThread.h
 * @brief Definition of a thread that receives an RGN image from input port and sends it to the output port.
 */


#ifndef _TUTORIAL_PERIODTHREAD_H_
#define _TUTORIAL_PERIODTHREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <fstream>
#include <time.h>


class tutorialPeriodThread : public yarp::os::PeriodicThread {
private:
    bool result;                    //result of the processing

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* tmpImage;                              // tmpImage generated by the processing
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;                            // input image from the inputPort
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* outputImage;                           // output image

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPort;     // output port to plot event
    std::string name;                                                                // rootname of all the ports opened by this thread
    
public:
    /**
    * constructor default
    */
    tutorialPeriodThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    tutorialPeriodThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~tutorialPeriodThread();

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
     * method for the processing in the ratethread
     **/
    bool processing();
};

#endif  //_TUTORIAL_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

