// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C)2019  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Omar Eldardeer
  * email: omar.eldardeer@iit.it
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

#include <iCub/helperFunctions.h>
#include "iCub/topDownAttentionModule.h"

bool topDownAttentionModule::configure(ResourceFinder &rf) {


    //initialize the name of the module from the parameter (name) or use the default if name isn't supported (/randomTopDownAttention)
    moduleName = rf.check("name",
                            Value("/randomTopDownAttention"),
                            "module name (string)").asString();
    setName(moduleName.c_str());

    //set the handlerPortName to module name. any msg to this port will be redirected to respond method
    handlerPortName =  "";
    handlerPortName += getName();

    //open handler port. fail the process if handle port failed to be opened
    if (!handlerPort.open(handlerPortName.c_str())) {
        yError("%s : Unable to open port %s \n ", getName().c_str() ,handlerPortName.c_str());
        return false;
    }

    //attach the handler port to the module
    attach(handlerPort);

    //initialize the thread with X seconds and pass the module name.
    //the module name is used to initialize the name of the ports opened in the thread.
    pThread = new topDownAttentionPeriodic(5,moduleName);

    //start thr thread. fail the process if the thread failed to be started
    bool ok = pThread->start();
    if(!ok)
    {
       cout << getName() << ": Problem in starting the thread "  << endl;
       return false;
    }
    cout << getName() << ": starting the thread "  << endl;
    return true ;
}

bool topDownAttentionModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool topDownAttentionModule::close() {
    //close the handlerPort and stop the thread
    handlerPort.close();
    yDebug("stopping the thread \n");
    pThread->stop();
    return true;
}

bool topDownAttentionModule::respond(const Bottle& command, Bottle& reply)
{
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
        yInfo(helpMessage.c_str());
        reply.addString("ok");
    }




    else if (command.get(0).asString()=="Orientation") {
        pThread->setOrientationMode();
        reply.addString("Orientation");
    }
    else if (command.get(0).asString()=="Motion") {
        pThread->setMotionMode();
        reply.addString("Motion");
    }
    else if (command.get(0).asString()=="Edges") {
        pThread->setEdgesMode();
        reply.addString("Edges");
    }
    else if (command.get(0).asString()=="Chrominance") {
        pThread->setChrominanceMode();
        reply.addString("Chrominance");
    }
    else if (command.get(0).asString()=="Intensity") {
        pThread->setIntensityMode();
        reply.addString("Intensity");
    }
    else if (command.get(0).asString()=="Blob") {
        pThread->setBlobMode();
        reply.addString("Blob");
    }
    else if (command.get(0).asString()=="Random") {
        pThread->setRandomMode();
        reply.addString("Random");
    }





    switch (command.get(0).asVocab()){
        case COMMAND_VOCAB_MODE_BLOB:
            pThread->setBlobMode();
            reply.addString("Blob");
            break;
        case COMMAND_VOCAB_MODE_MOTION:
            break;
        case COMMAND_VOCAB_MODE_CHROMINANCE:
            pThread->setChrominanceMode();
            reply.addString("Chrominance");
            break;
        case COMMAND_VOCAB_MODE_INTENSITY:
            pThread->setIntensityMode();
            reply.addString("Intensity");
            break;
        case COMMAND_VOCAB_MODE_EDGES:
            pThread->setEdgesMode();
            reply.addString("Edges");
            break;
        case COMMAND_VOCAB_MODE_ORIENTATION:
            pThread->setOrientationMode();
            reply.addString("Orientation");
            break;
        case COMMAND_VOCAB_MODE_RANDOM:
            pThread->setRandomMode();
            reply.addString("Random");
            break;

    }

    return true;
}

/* Called periodically every getPeriod() seconds */
bool topDownAttentionModule::updateModule()
{
    return true;
}

double topDownAttentionModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 5;
}
