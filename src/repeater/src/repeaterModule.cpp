// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Shashank Pathak
  * email: shashank.pathak@iit.it
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
 * @file repeaterModule.cpp
 * @brief Implementation of the repeaterModule (see header file).
 */

#include "iCub/repeaterModule.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

// general command vocab's
#define COMMAND_VOCAB_HELP               VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SET                VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET                VOCAB3('g','e','t')
#define COMMAND_VOCAB_RUN                VOCAB3('r','u','n')
#define COMMAND_VOCAB_SUSPEND            VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME             VOCAB3('r','e','s')
#define COMMAND_VOCAB_FIX                VOCAB3('f','i','x')
#define COMMAND_VOCAB_IS                 VOCAB2('i','s')
#define COMMAND_VOCAB_OK                 VOCAB2('o','k')
#define COMMAND_VOCAB_FAILED             VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_SEEK               VOCAB4('s','e','e','k')
#define COMMAND_VOCAB_CENT               VOCAB4('c','e','n','t')
#define COMMAND_VOCAB_STOP               VOCAB4('s','t','o','p')
#define COMMAND_VOCAB_PUSH               VOCAB4('p','u','s','h')
#define COMMAND_VOCAB_SAT                VOCAB3('s','a','t')
#define COMMAND_VOCAB_HUE                VOCAB3('h','u','e')
#define COMMAND_VOCAB_BRI                VOCAB3('b','r','i')

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool repeaterModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/repeater"), 
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

    /**
     * get the dimension of the output image
     */   
    int  outputWidth       = rf.check("outputWidth", 
                           Value(320), 
                           "output image width (int)").asInt();
    int  outputHeight      = rf.check("outputHeight", 
                           Value(240), 
                           "output image height (int)").asInt();

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
    rThread = new repeaterThread(robotName, configFile);
    rThread->setName(getName().c_str());
    rThread->setOutputDimension(outputWidth, outputHeight);
    //rThread->setInputPortName(inputPortName.c_str());
    
    /* now start the thread to do the work */
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool repeaterModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool repeaterModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the thread \n");
    rThread->stop();
    return true;
}

bool repeaterModule::respond(const Bottle& command, Bottle& reply) 
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
    else if ((command.get(0).asString()=="sus") || (command.get(0).asString()=="\"sus\"")) {
        //prioritiser->waitMotionDone();
        //prioritiser->suspend();
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="res" || command.get(0).asString()=="\"res\"" ) {
        //prioritiser->resume();
        reply.addString("ok");
    }
    
    mutex.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addVocab(Vocab::encode("many"));
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
            //prioritiser->suspend();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_STOP:
        rec = true;
        {
            //prioritiser->suspend();
            //prioritiser->resume();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_RESUME:
    rec = true;
        {
            //prioritiser->resume();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_SEEK:
        rec = true;
        {
            //prioritiser->suspend();
            //prioritiser->seek(command);
            //prioritiser->resume();
            ok = true;
        }
        break;
    case COMMAND_VOCAB_FIX:
        rec = true;
        {
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_CENT:
                {
                    printf("Fixating in Center \n");
                    //prioritiser->fixCenter(1000);
                }
                break;
            }
            ok = true;
        }
        break;
    case COMMAND_VOCAB_PUSH:
        rec = true;
        {
            switch (command.get(1).asVocab()) {
            case COMMAND_VOCAB_SAT:
                {   int delta = command.get(2).asInt();
                    printf("Pushing Saturation %d \n", delta);
                    rThread->setSatPush(delta);
                }
                break;
            
            case COMMAND_VOCAB_BRI:
                {
                    int delta = command.get(2).asInt();
                    printf("Pushing Brightness %d \n", delta);
                    rThread->setBriPush(delta);
                }
                break;
            
            case COMMAND_VOCAB_HUE:
                {
                    int delta = command.get(2).asInt();
                    printf("Pushing Hue %d \n", delta);
                    rThread->setHuePush(delta);
                }
                break;
                
            }
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
    else {
        reply.addVocab(COMMAND_VOCAB_OK);
    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool repeaterModule::updateModule()
{
    return true;
}

double repeaterModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}

