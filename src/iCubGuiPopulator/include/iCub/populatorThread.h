// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file populatorThread.h
 * @brief Definition of a thread that sends queries, parses the reply and send commands to iCubGui
 * (see populatorModule.h).
 */

#ifndef _POPULATOR_THREAD_H_
#define _POPULATOR_THREAD_H_

#include <yarp/os/Port.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include <iostream>

#define MAXBUFFERDIMENSION 10

class populatorThread : public yarp::os::RateThread {
private:
    std::string name;                                                       // name of this module
    int numNames;
    int count;                                                              // counter of the cycle
    short listNames[MAXBUFFERDIMENSION];
    short cName[MAXBUFFERDIMENSION];
    yarp::os::Port databasePort;                                            // rpc the remote procedure call port used to send requests to the database and receive replies
    yarp::os::BufferedPort< yarp::os::Bottle> guiPort;                      // port of the gui that receives commands for the object populating process
    yarp::os::BufferedPort<yarp::sig::VectorOf<unsigned char> > texPort;    // port for sending the texture
public:
    /**
    * default constructor
    */
    populatorThread();

    /**
     * destructor
     */
    ~populatorThread();

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
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);

    /**
     * function that searches in the vector the presence of the name
     */
    bool checkNames(short str);

    /**
     * function that searches in the vector the presence of the name
     */
    void cleanNames();
};

#endif  //POPULATOR_THREAD_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------

