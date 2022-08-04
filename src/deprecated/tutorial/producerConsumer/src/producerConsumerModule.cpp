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
 * @file producerConsumerModule.cpp
 * @brief Implementation of the producerConsumerModule (see header file).
 */

#include "iCub/producerConsumerModule.h"		 //in the file .cpp of the module you  have to inculde just this one


using namespace yarp::os;
using namespace yarp::sig;
using namespace attention::dictionary;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool producerConsumerModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/producerConsumer"), 
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
    rThread = new producer(robotName, configFile);
    rThread->setName(getName().c_str());
    //rThread->setInputPortName(inputPortName.c_str());
    
    /* now start the thread to do the work */
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool producerConsumerModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool producerConsumerModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the thread \n");
    rThread->stop();
    return true;
}

bool producerConsumerModule::respond(const Bottle& command, Bottle& reply) 
{
    bool ok = false;
    bool rec = false; // is the command recognized?

    string helpMessage =  string(getName().c_str()) + 
                " commands are: \n" +  
                "help \n" +
                "quit \n";
    reply.clear(); 

    //if (command.get(0).asString()=="quit") {
    //    reply.addString("quitting");
    //    return false;     
    //}
    //else if (command.get(0).asString()=="help") {
    //    cout << helpMessage;
    //    reply.addString("ok");
    //}

    respondLock.wait();
    switch (command.get(0).asVocab32()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addVocab32(Vocab32::encode("many"));
            reply.addString("help");
            reply.addString("commands are:");
            reply.addString(" help    : to get help");
            reply.addString(" quit    : to quit the module");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" sus     : to suspend the processing");
            reply.addString(" res     : to resume  the processing");
            reply.addString(" ");
            reply.addString(" ");
            reply.addString(" vis on  : to enable  the visualization");
            reply.addString(" vis off : to disable the visualization");
            reply.addString("    ");
            reply.addString(" test    : automatic test of the features of the module");
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
            
            ok = true;
        }
        break;
    case COMMAND_VOCAB_TEST:
        rec = true;
        {
            reply.addString("testing");
            bool res = rThread->test();
            if(res){ 
                reply.addString("test:success");
                ok = true;
            }
            else{
                reply.addString("test:insuccess");
                ok = true;
            }
        }
        break;
    case COMMAND_VOCAB_VIS:
        {
            rec = true;
            switch(command.get(1).asVocab32()){
            case COMMAND_VOCAB_ON:
                {
                    reply.addString("visualization ON");
                    rThread->visualizationResume();
                    ok = true;   
                }
            break;
            case COMMAND_VOCAB_OFF:
                {
                    reply.addString("visualization OFF");
                    rThread->visualizationSuspend();
                    ok = true;   
                }
            break;
            }
        }
        break;

    case COMMAND_VOCAB_GET:
        {
            rec = true;
            switch(command.get(1).asVocab32()){
            case COMMAND_VOCAB_WEIGHT:
                {
                    switch(command.get(2).asVocab32()){
                    case COMMAND_VOCAB_HOR:
                        
                        reply.clear();
                        reply.addVocab32(COMMAND_VOCAB_HOR); // ?? Needed
                        //reply.addFloat32(wt);
                        rec = true;
                        ok = true;
                        break;
                                                   
                    default:
                        rec = false;
                        ok  = false;
                    
                    }

                }
            break;
            
            }
        }
        break;

    case COMMAND_VOCAB_SUSPEND:
        rec = true;
        {
            reply.addString("suspending processing");
            rThread->suspend();
            ok = true;   
        }
        break;
    case COMMAND_VOCAB_RESUME:
        rec = true;
        {
            reply.addString("resuming processing");
            rThread->resume();
            ok = true;
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
        reply.addVocab32(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab32(COMMAND_VOCAB_OK);
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool producerConsumerModule::updateModule()
{
    return true;
}

double producerConsumerModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}

