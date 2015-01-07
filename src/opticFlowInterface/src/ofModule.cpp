// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file ofModule.cpp
 * @brief Implementation of the ofModule (see header file).
 */

#include <iCub/ofModule.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool ofModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    if (rf.check("help")) {
        printf("Help \n");
        printf("==== \n");
        printf("--name   : specifies the name of the module \n");
        printf("--robot  : specifies the name of the robot\n");
        printf("--config : specifies the name of the config file of the camera\n");
        printf("--width  : specifies the dimension of the input space \n");
        printf("--height : specifies the dimension of the input space\n");
        printf("\n");
        printf("waiting for CTRL-C command to close the module \n");
        return true;
    }

    /* get the module name which will form the stem of all module port names */
    moduleName             = rf.check("name", 
                           Value("/ofInterface"), 
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

    collector=new ofCollector();
    collector->setName(getName().c_str());
    
    arbiter=new ofThread(configFile);
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

    
    
    // specifies whether the camera is mounted on the head
    //onWings       = rf.check("onWings", 
    //                       Value(0), 
    //                       "indicates whether the camera is mounted on the head").asInt();
    //printf("onWings %d \n", onWings);
    //arbiter->setOnWings(onWings);
    
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

bool ofModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool ofModule::close() {
    handlerPort.close();
    //stopping threads
    collector->stop();
    arbiter->stop();
    delete collector;
    delete arbiter;
    return true;
}

bool ofModule::respond(const Bottle& command, Bottle& reply) {
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
    
    reply.clear();

    bool ok = false;
    bool rec = false; // is the command recognized?

    respondLock.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            //reply.addString("many");    // what used to work
            reply.addString("help");
            reply.addString("commands are:");
            reply.addString(" help  : to get help");
            reply.addString(" quit  : to quit the module");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" sus   : suspending the thread   ");
            reply.addString(" res   : resuming the thread     ");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" ");
            //reply.addString(helpMessage.c_str());
            ok = true;
        }
        break;
    case COMMAND_VOCAB_QUIT:
        rec = true;
        {
            reply.addString("quitting");
            ok = false;
        }
        break;
    case COMMAND_VOCAB_GET:
        {            
            ok = true;rec = true;
            printf("responding to the get command \n");
            int lefttopx = command.get(1).asInt() ;
            int lefttopy = command.get(2).asInt() ;
            int bottomrightx = command.get(3).asInt() ;
            int bottomrighty = command.get(4).asInt() ;
            double u, v;
            arbiter->getVelocity(lefttopx, lefttopy, bottomrightx, bottomrighty, u, v);
            reply.addDouble(u);
            reply.addDouble(v);
        }
        break;
    
    case COMMAND_VOCAB_SUSPEND:
        {            
            reply.addString("suspending");
            ok = true;
            arbiter->suspend();
        }
        break;
    case COMMAND_VOCAB_RESUME:
        {
            reply.addString("resuming");
            ok = true;
            arbiter->resume();
        }
        break;
    default:
        rec = false;
        ok  = false;
    }    
    
    respondLock.post();
    if (!rec){
        ok = RFModule::respond(command,reply);
    }
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);

    return ok;

}

/* Called periodically every getPeriod() seconds */
bool ofModule::updateModule() {
    return true;
}

double ofModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule (never 0 value!!) */
    return 1.0;
}

