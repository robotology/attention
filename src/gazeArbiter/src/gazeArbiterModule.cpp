// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
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
 * @file gazeArbiterModule.cpp
 * @brief Implementation of the gazeArbiterModule (see header file).
 */

#include <iCub/gazeArbiterModule.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool gazeArbiterModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName             = rf.check("name", 
                           Value("/gazeArbiter"), 
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

    if (rf.check("config")) {
        configFile=rf.findFile(rf.find("config").asString().c_str());
        if (configFile=="") {
            return false;
        }
    }
    else {
        configFile.clear();
    }

    collector=new gazeCollectorThread();
    collector->setName(getName().c_str());
    
    arbiter=new gazeArbiterThread(configFile);
    arbiter->setName(getName().c_str());
    arbiter->setRobotName(robotName);

    /* get the dimension of the image for the thread parametric control */
    width                  = rf.check("width", 
                           Value(320), 
                           "width of the image (int)").asInt();

    height                 = rf.check("height", 
                           Value(240), 
                           "height of the image (int)").asInt();

    printf("\n width: %d  height:%d \n", width, height);
    arbiter->setDimension(width,height);

    /* offset for 3d position along x axis */
    this->xoffset       = rf.check("xoffset", 
                           Value(0), 
                           "offset for 3D fixation point x").asInt();
    arbiter->setXOffset(xoffset);

    /* offset for 3d position along y axis */
    this->yoffset       = rf.check("yoffset", 
                           Value(0), 
                           "offset for 3D fixation point y").asInt();
    arbiter->setYOffset(yoffset);

    /* offset for 3d position along z axis */
    this->zoffset       = rf.check("zoffset", 
                           Value(0), 
                           "offset for 3D fixation point z").asInt();
    arbiter->setZOffset(zoffset);

    
    // limits for 3d position along x axis 
    xmax       = rf.check("xmax", 
                           Value(-0.2), 
                          "limit max for 3D fixation point x").asDouble();
    printf("xmax:%f \n", xmax);
    xmin       = rf.check("xmin", 
                           Value(-10.0), 
                           "limit min for 3D fixation point x").asDouble();;
    printf("xmin:%f \n", xmin);
    arbiter->setXLimits(xmax,xmin);
    
    // limits for 3d position along y axis 
    ymax       = rf.check("ymax", 
                           Value(0.3), 
                           "limit max for 3D fixation point y").asDouble();
    printf("ymax:%f \n", ymax);
    ymin       = rf.check("ymin", 
                           Value(-0.3), 
                           "limit max for 3D fixation point y").asDouble();
    printf("ymin:%f \n", ymin);
    arbiter->setYLimits(ymax,ymin);
    
    // limits for 3d position along z axis 
    zmax       = rf.check("zmax", 
                           Value(0.9), 
                           "limit max for 3D fixation point z").asDouble();
    printf("zmax:%f \n", zmax);
    zmin       = rf.check("zmin", 
                           Value(-0.3), 
                           "limit min for 3D fixation point z").asDouble();
    printf("zmin:%f \n", zmin);
    arbiter->setZLimits(zmax,zmin);

    // fixating pitch
    pitch       = rf.check("blockPitch", 
                           Value(-1), 
                           "fixing the pitch to a desired angle").asDouble();
    printf("pitch:%f \n", pitch);
    arbiter->setBlockPitch(pitch);

    collector->addObserver(*arbiter);
    arbiter->start();
    collector->start();

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
    //attach(Port);                       // attach to port

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool gazeArbiterModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool gazeArbiterModule::close() {
    handlerPort.close();
    //stopping threads
    collector->stop();
    arbiter->stop();
    delete collector;
    delete arbiter;
    return true;
}

bool gazeArbiterModule::respond(const Bottle& command, Bottle& reply) {
    string helpMessage =  string(getName().c_str()) + 
                        " commands are: \n" +  
                        "help \n" + 
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
    else if ((command.get(0).asString()=="sus") || (command.get(0).asString()=="\"sus\"")) {
        //arbiter->waitMotionDone();
        arbiter->suspend();
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="res" || command.get(0).asString()=="\"res\"" ) {
        arbiter->resume();
        reply.addString("ok");
    }
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool gazeArbiterModule::updateModule() {
    return true;
}

double gazeArbiterModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 0.0;
}

