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

#include "iCub/topDownAttentionModule.h"

bool topDownAttentionModule::configure(ResourceFinder &rf) {

    serverName=rf.find("server").asString();
    clientName=rf.find("client").asString();

    setName("periodicAttentionModule");

    //
    // handlerPortName =  "";
    // handlerPortName += getName();         // use getName() rather than a literal
    //
    // if (!handlerPort.open(handlerPortName.c_str())) {
    //     cout << getName() << ": Unable to open port " << handlerPortName << endl;
    //     return false;
    // }
    //
    // attach(handlerPort);                  // attach to port

    pThread = new topDownAttentionPeriodic(5,clientName, serverName);

    bool ok = pThread->start();
    if(!ok)
    {
       cout << getName() << ": Problem in starting the thread "  << endl;
    }
    cout << getName() << ": starting the thread "  << endl;
    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool topDownAttentionModule::interruptModule() {
    //handlerPort.interrupt();
    return true;
}

bool topDownAttentionModule::close() {
    //handlerPort.close();
    /* stop the thread */
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
        cout << helpMessage;
        reply.addString("ok");
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
    return 1;
}
