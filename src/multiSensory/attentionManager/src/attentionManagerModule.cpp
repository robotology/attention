// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C)2020 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
#include "iCub/attentionManagerModule.h"

using namespace attention::dictionary;

bool attentionManagerModule::configure(ResourceFinder &rf) {


    //initialize the name of the module from the parameter (name) or use the default if name isn't supported (/randomTopDownAttention)
    moduleName = rf.check("name",
                          Value("/attentionManager"),
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
    pThread = new attentionManagerThread(moduleName);
    if (!pThread->configure(rf)){
        yInfo("Problem in configuring the thread ");
        return false;
    }

    //start thr thread. fail the process if the thread failed to be started
    if(!pThread->start())
    {
        yInfo("Problem in starting the thread ");
        return false;
    }
    yInfo(" starting the thread ");
    return true ;
}

bool attentionManagerModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool attentionManagerModule::close() {
    //close the handlerPort and stop the thread
    handlerPort.close();
    yDebug("stopping the thread \n");
    pThread->stop();
    delete pThread;
    return true;
}

bool attentionManagerModule::respond(const Bottle& command, Bottle& reply)
{
    string helpMessage =  string(getName().c_str()) +
                          " commands are: \n " +
                          "help \n " +
                          "[res] to resume process  \n " +
                          "[sus] to suspend process \n " +
                          "quit \n ";
    reply.clear();

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;
    }
    else if (command.get(0).asString()=="help") {
        reply.addString(helpMessage);
        return true;
    }

    bool ok = false;

    switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_SUSPEND:
            if(command.size()==2 && pThread->suspendAttentionState(command.get(1).asInt())){
                reply.addVocab(COMMAND_VOCAB_OK);
                ok = true;
                break;
            }  else{
                reply.addVocab(COMMAND_VOCAB_ERROR);
                ok = true;
                break;
            }
        case COMMAND_VOCAB_RESUME:
            if(command.size()==2 && pThread->resetAttentionState(command.get(1).asInt())){
                reply.addVocab(COMMAND_VOCAB_OK);
                ok = true;
                break;
            }  else{
                reply.addVocab(COMMAND_VOCAB_ERROR);
                ok = true;
                break;
            }
        case COMMAND_VOCAB_SET:
            if(command.size()==5){
                switch (command.get(1).asVocab()){
                    case COMMAND_VOCAB_THRESHOLD:
                        if(pThread->setThreshold(command.get(2).asVocab(),command.get(3).asVocab(),command.get(4).asFloat64())){
                            reply.addVocab(COMMAND_VOCAB_OK);
                            ok = true;
                            break;
                        }
                        else{
                            reply.addVocab(COMMAND_VOCAB_ERROR);
                            ok = true;
                            break;
                        }
                }
            }
            break;
        case COMMAND_VOCAB_GET:
            if(command.size()==4){
                switch (command.get(1).asVocab()){
                    case COMMAND_VOCAB_THRESHOLD:
                        reply.addFloat64(pThread->getThreshold(command.get(2).asVocab(),command.get(3).asVocab()));
                        ok = true;
                        break;
                }
            }
            break;
        case COMMAND_VOCAB_RESET:
            if(command.size()==3){
                switch (command.get(1).asVocab()){
                    case COMMAND_VOCAB_THRESHOLD:
                        ok = pThread->resetThreshold(command.get(2).asVocab());
                        if( ok)
                            reply.addVocab(COMMAND_VOCAB_OK);
                        break;
                }
            }
            break;
    }

    if(!ok)
        reply.addString("undefined Command " + helpMessage );

    return true;
}

/* Called periodically every getPeriod() seconds */
bool attentionManagerModule::updateModule()
{
    return true;
}

double attentionManagerModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}
