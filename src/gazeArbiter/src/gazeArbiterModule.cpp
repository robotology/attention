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
    if(rf.check("help")) {
        printf("HELP \n");
        printf("====== \n");
        printf("--name  : changes the rootname of the module ports \n");
        printf("--robot : changes the name of the robot where the module interfaces to  \n");
        printf("--visualFeedback : ");
        printf("--name : rootname for all the connection of the module \n");
        printf("--xmax, xmin, ymax, ymin, zmax, zmin : outOfReach limits \n");
        printf("--onWings : if the camera is mounted on the wings\n ");
        printf("--onDvs   : if the camera is DVS camera \n");
        printf(" \n");
        printf("press CTRL-C to continue.. \n");
        return true;
    }
    

    /* get the module name which will form the stem of all module port names */
    moduleName             = rf.check("name", 
                           Value("/gazeArbiter"), 
                           "module name (string)").asString();

    /*
    * before continuing, set the module name before getting any other parameters, 
    * specifically the port names which are dependent on the module name
    */
    setName(moduleName.c_str());
    printf("module will be activated with the name: %s \n", getName().c_str());

    /*
    * get the robot name which will form the stem of the robot ports names
    * and append the specific part and device required
    */
    robotName             = rf.check("robot", 
                           Value("icub"), 
                           "Robot name (string)").asString();
    robotPortName         = "/" + robotName + "/head";
    printf("robotName: %s \n", robotName.c_str());
    
    configName             = rf.check("config", 
                           Value("icubEyes.ini"), 
                           "Config file for intrinsic parameters (string)").asString();
    printf("configFile: %s \n", configName.c_str());

    if (strcmp(configName.c_str(),"")) {
        printf("looking for the config file \n");
        configFile=rf.findFile(configName.c_str());
        printf("config file %s \n", configFile.c_str());
        if (configFile=="") {
            printf("ERROR: file not found");
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
    
    if (rf.check("visualFeedback")) {
        arbiter->setVisualFeedback(true);
        printf("visualFeedback required \n");
    }
    else {
        printf("visualFeedback  not required \n");
        arbiter->setVisualFeedback(false);
    }

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
                           "offset for 3D fixation point x").asDouble();
    printf("xoffset:%f \n", xoffset);
    arbiter->setXOffset(xoffset);

    /* offset for 3d position along y axis */
    this->yoffset       = rf.check("yoffset", 
                           Value(0), 
                           "offset for 3D fixation point y").asDouble();
    printf("yoffset:%f \n", yoffset);
    arbiter->setYOffset(yoffset);

    /* offset for 3d position along z axis */
    this->zoffset       = rf.check("zoffset", 
                           Value(0), 
                           "offset for 3D fixation point z").asDouble();
    printf("zoffset:%f \n", zoffset);
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
    
    // specifies whether the camera is mounted on the head
    //onWings       = rf.check("onWings", 
    //                       Value(0), 
    //                       "indicates whether the camera is mounted on the head").asInt();
    //printf("onWings %d \n", onWings);
    //arbiter->setOnWings(onWings);
    
    // specifies whether the camera is mounted on the head
    mode       = rf.check("mode", 
                           Value("standard"), 
                           "indicates mapping with which the image plane is moved").asString();
    printf("mode seleected: %s \n", mode.c_str());
    if(!strcmp("onWings", mode.c_str())) {
        printf("onWings %d true \n", onWings);
        arbiter->setOnWings(true);
    }
    else if(!strcmp("onDvs", mode.c_str())) {
        printf("onDvs true  \n");
        arbiter->setOnDvs(true);
    } 
       
   

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
    * attach a port of the same name as the module (not prefixed with a /) to the module
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

//double gazeArbiterModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
//    return 1.0;
//}

