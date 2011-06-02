// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
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
 * @file mosaicModule.cpp
 * @brief Implementation of the mosaicModule (see header file).
 */

#include "iCub/mosaicModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool mosaicModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/mosaic"), 
                           "module name (string)").asString();
    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";
    
    /*
    * get the original width of the input image
    */
    width_orig            = rf.check("widthOrig", 
                           Value(320), 
                           "width original (int)").asInt();
    /*
    * get the original height of the input image
    */
    height_orig            = rf.check("heightOrig", 
                           Value(240), 
                           "height original (int)").asInt();
    /*
    * get the width of the mosaic image
    */
    width            = rf.check("width", 
                           Value(640), 
                           "width mosaic (int)").asInt();
    /*
    * get the original height of the mosaic image
    */
    height            = rf.check("height", 
                           Value(480), 
                           "height mosaic (int)").asInt();
    /*
    * attach a port of the same name as the module (prefixed with a /) to the module
    * so that messages received from the port are redirected to the respond method
    */
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port
    if (rf.check("config")) {
        configFile=rf.findFile(rf.find("config").asString().c_str());
        if (configFile=="") {
            return false;
        }
    }
    else {
        configFile.clear();
    }

    /* create the thread and pass pointers to the module parameters */
    mThread = new mosaicThread(robotName, configFile);
    mThread->setName(getName().c_str());
    
    /* now start the thread to do the work */
    mThread->start(); // this calls threadInit() and it if returns true, it then calls run()
    
    // setting the dimension after threadinit to use some correct variables    
    mThread->setInputDim(width_orig, height_orig);
    mThread->setMosaicSize(width, height);       
    mThread->placeInpImage(width / 2, height / 2);  

    if (rf.check("rectify")) {
        mThread->setRectification(true);
        printf("rectification required \n");
    }
    else {
        printf("rectification not required \n");
    }
    

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool mosaicModule::interruptModule()
{
    handlerPort.interrupt();
    return true;
}

bool mosaicModule::close()
{
    handlerPort.close();
    /* stop the thread */
    printf("stopping the thread \n");
    mThread->stop();
    return true;
}

bool mosaicModule::respond(const Bottle& command, Bottle& reply) 
{
    string helpMessage =  string(getName().c_str()) + 
        " commands are: \n" +  
        "help \n" +
        "size <int> <int> -to change, if allowed, size (width, height) of mosaic to <int> <int> \n" + 
        "place <int> -to place, if allowed, input image's center (horz, vert) in mosaic refernce frame \n" +
        "fetch <float> <float> -to placefetch a portion of th emosaic \n" +
        "quit \n";

    reply.clear(); 


    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        cout << helpMessage;
        reply.addString("ok");
    }
    else if (command.get(0).asString() == "size") {
        bool set = mThread->setMosaicSize(command.get(1).asInt(), command.get(2).asInt());
        if (set) reply.addString("Changed size of the mosaic.");
        else reply.addString("Could NOT change size of the mosaic.");
    }
    else if (command.get(0).asString() == "place") {
        bool set = mThread->placeInpImage(command.get(1).asInt(), command.get(2).asInt());
        if(set) reply.addString("Input image placed successfully");
        else reply.addString("Input image can NOT be placed there!");
    }
    else if (command.get(0).asString() == "plot") {
      double x = command.get(1).asDouble();
      double y = command.get(2).asDouble();
      double z = command.get(3).asDouble();
      
      printf("x %f y %f z %f \n",x,y,z);
      mThread->plotObject(x,y,z);
    }
    else if (command.get(0).asString() == "fetch") {
      double azimuth = command.get(1).asDouble();
      double elevation = command.get(2).asDouble();
     
      printf("azimuth %f elevation %f  \n",azimuth,elevation);
      mThread->setFetchPortion(azimuth, elevation);
    }
    

    return true;
}

/* Called periodically every getPeriod() seconds */
bool mosaicModule::updateModule()
{
    return true;
}

double mosaicModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.1;
}

