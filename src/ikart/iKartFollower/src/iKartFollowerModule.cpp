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
 * @file iKartFollowerModule.cpp
 * @brief Implementation of the iKartFollowerModule (see header file).
 */

#include "iCub/iKartFollowerModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define COMMAND_VOCAB_OK  VOCAB2('o', 'k')
#define COMMAND_VOCAB_RESUME  VOCAB3('r', 'e', 's')
#define COMMAND_VOCAB_FIX     VOCAB3('f', 'i', 'x')
#define COMMAND_VOCAB_STOP    VOCAB4('s', 't', 'o', 'p')
#define COMMAND_VOCAB_SUSPEND VOCAB4('s', 'u', 's', 'p')
#define COMMAND_VOCAB_FAILED  VOCAB4('f', 'a', 'i', 'l')
#define COMMAND_VOCAB_HELP    VOCAB4('h', 'e', 'l', 'p')



/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool iKartFollowerModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */
    if(rf.check("help")) {
        printf("HELP \n");
        printf("====== \n");
        printf("--name           : changes the rootname of the module ports \n");
        printf("--robot          : changes the name of the robot where the module interfaces to  \n");
        printf("--visualFeedback : indicates whether the visual feedback is active \n");
        printf("--name           : rootname for all the connection of the module \n");
        printf("--camerasContext : context where camera parameters are stored \n");
        printf("--camerasFile    : file of parameters of the camera in the context \n");
        printf("--drive          : left/right indicates the drive eye");
        printf("--config         : camera parameters");
        printf("--blockPitch     : blocking the head during motions \n");
        printf("--xmax, xmin, ymax, ymin, zmax, zmin : outOfReach limits \n");
        printf("--onWings        : if the camera is mounted on the wings\n ");
        printf("--onDvs          : if the camera is DVS camera \n");
        printf(" \n");
        printf("press CTRL-C to stop... \n");
        return true;
    }
    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/iKartFollower"), 
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

    inputPortName           = rf.check("inputPortName",
			                Value(":i"),
                            "Input port name (string)").asString();
    
    
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

    if (rf.check("targetFile")) {
        targetFilePath=rf.findFile(rf.find("targetFile").asString().c_str());
        if (targetFilePath=="") {
            return false;
        }
    }
    else {
        targetFilePath.clear();
    }
    yInfo("targetFilePath %s", targetFilePath.c_str());

    /* create the thread and pass pointers to the module parameters */
    rThread = new iKartFollowerThread(robotName, configFile);
    rThread->setName(getName().c_str());
    rThread->setTargetFilePath(targetFilePath);
    //rThread->setInputPortName(inputPortName.c_str());

    
    /* now start the thread to do the work */
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool iKartFollowerModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool iKartFollowerModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the thread \n");
    rThread->stop();
    return true;
}

bool iKartFollowerModule::respond(const Bottle& command, Bottle& reply) 
{    
    bool ok = false;
    bool rec = false; // is the command recognized?

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

    mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addString("many");
            reply.addString("help");

            //reply.addString();
            reply.addString("set fn \t: general set command ");
            reply.addString("get fn \t: general get command ");
            //reply.addString();

            
            //reply.addString();
            reply.addString("seek red \t : looking for a red color object");
            reply.addString("seek rgb \t : looking for a general color object");
            reply.addString("sus  \t : suspending");
            reply.addString("res  \t : resuming");
            //reply.addString();


            ok = true;
        }
        break;
    case COMMAND_VOCAB_SUSPEND:
        rec = true;
        {
  
            ok = true;
        }
        break;
    case COMMAND_VOCAB_STOP:
        rec = true;
        {

            ok = true;
        }
        break;
    case COMMAND_VOCAB_RESUME:
    rec = true;
        {
            
            ok = true;
        }
        break;

    default: {
                
    }
        break;    
    }
    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool iKartFollowerModule::updateModule()
{
    return true;
}

double iKartFollowerModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly */
    return 1;
}

