// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file producer.h
 * @brief Definition of a thread that receives an RGB image from input port and sends it to the output port.
 */


#ifndef _producerConsumer_THREAD_H_
#define _producerConsumer_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <yarp/os/Semaphore.h>

#include <iCub/consumer.h>


#define WIDTH  320
#define HEIGHT 240


class producer : public yarp::os::Thread {
private:
	int sharedResource;									  //tutti gli attributi sono privati, e tu ne dai  accesso A TUTTO CIO CHE � Esterno devo usare i metodi pubblici (per dare protezione all'attributo)
	int* pSharedResource;
	yarp::os::Semaphore sem;
	yarp::os::Semaphore* pSem;

	double  timenow_producer;
	double  timepred_producer;
	double  timeDiff_producer;

    int width, height;
    bool idle;                      // flag that interrupts the processing 
    yarp::os::Semaphore idleLock;   // semaphore that checks access to the resource

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;     // image generated in the processing

    consumer* pt;                                       // rateThread responsible for the visualization of the generated images

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort;  //inputport for processing
    yarp::os::BufferedPort<yarp::os::Bottle> outputPort;     // output port , result of the processing
    yarp::os::Semaphore checkImage;                          // semaphore responsible for the access to the inputImage     
    std::string name;                                                                // rootname of all the ports opened by this thread
    
public:
    /**
    * constructor default
    */
    producer();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    producer(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~producer();

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

    /**
     * @brief function of main processing in the thread
     */
    int processing();

    /**
     * @brief suspend the processing of the module
     */
    void suspend(){idleLock.wait(); idle=true; idleLock.post();};

    /**
     * @brief resume the processing of the module
     */
    void resume(){idleLock.wait(); idle=false; idleLock.post();};

    /**
     * @brief visualization suspend method
     */
    void visualizationSuspend();

    /**
     * @brief visualization resume method
     */
    void visualizationResume();

    /**
     * @brief function that test the network and feature of the thread
     * @return the result of the analysis true/false for success/unsuccess in the test
     */
    bool test();

};

#endif  //_producerConsumer_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

