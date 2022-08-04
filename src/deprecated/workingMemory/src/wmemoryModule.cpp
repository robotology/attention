// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
 * email:   francesco.rea@iit.it
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
 * @file wmemoryModule.cpp
 * @brief Implementation of the wmemoryModule (see header file).
 */

#include <iCub/wmemoryModule.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool wmemoryModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/workingMemory"), 
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

    wmThread = new wmemoryThread();
    wmThread->setName(getName().c_str());
    wmThread->start();

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool wmemoryModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool wmemoryModule::close() {
    handlerPort.close();
    /* stop the thread */
    wmThread->stop();
    delete wmThread;
    return true;
}

bool wmemoryModule::respond(const Bottle& command, Bottle& reply) {
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
    switch (command.get(0).asVocab32()) {
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
    case COMMAND_VOCAB_ADD:
        rec = true;
        {
            Bottle* listAttr = command.get(1).asList();
            printf("got the list of attribute %s \n", listAttr->toString().c_str());
            wmThread->setTarget(*listAttr);
    
            /*
            Bottle& sublistX = listAttr.addList();
    
            sublistX.addString("x");
            sublistX.addFloat32(x1[0] * 1000);
            listAttr.append(sublistX);
            
            Bottle& sublistY = listAttr.addList();
            sublistY.addString("y");
            sublistY.addFloat32(x1[1] * 1000);
            listAttr.append(sublistY);
            
            Bottle& sublistZ = listAttr.addList();            
            sublistZ.addString("z");
            sublistZ.addFloat32(x1[2] * 1000);
            listAttr.append(sublistZ);
            
            Bottle& sublistR = listAttr.addList();
            sublistR.addString("r");
            sublistR.addFloat32(255.0);
            listAttr.append(sublistR);
            
            Bottle& sublistG = listAttr.addList();
            sublistG.addString("g");
            sublistG.addFloat32(255.0);
            listAttr.append(sublistG);
            
            Bottle& sublistB = listAttr.addList();
            sublistB.addString("b");
            sublistB.addFloat32(255.0);
            listAttr.append(sublistB);
            
            Bottle& sublistLife = listAttr.addList();
            sublistLife.addString("lifeTimer");
            sublistLife.addFloat32(1.0);
            listAttr.append(sublistLife)
            */
            ok = true;
        }
    default: {
                
    }
        break;    
    }
    mutex.post();

    if (!rec)
        ok = RFModule::respond(command,reply);
    
    if (!ok) {
        reply.clear();
        reply.addVocab32(COMMAND_VOCAB_FAILED);
    }
    else{
        reply.addVocab32(COMMAND_VOCAB_OK);
    }

    RFModule::respond(command,reply); 

    return ok;
}

/* Called periodically every getPeriod() seconds */
bool wmemoryModule::updateModule() {
    return true;
}

double wmemoryModule::getPeriod() {
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}

