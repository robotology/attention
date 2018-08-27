/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file earlyMotionModule.cpp
 * @brief Implementation of the earlyMotionModule (see header file).
 */

#include "iCub/earlyMotionModule.h"

// general command vocab's
#define COMMAND_VOCAB_OK     VOCAB2('o','k')

#define COMMAND_VOCAB_SET    VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET    VOCAB3('g','e','t')
#define COMMAND_VOCAB_SUS    VOCAB3('s','u','s')
#define COMMAND_VOCAB_RES    VOCAB3('r','e','s')
#define COMMAND_VOCAB_THRESSHOLD VOCAB3('t','h', 'r')

#define COMMAND_VOCAB_HELP   VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool earlyMotionModule::configure(yarp::os::ResourceFinder &rf) {
  /* Process all parameters from both command-line and .ini file */

  /* get the module name which will form the stem of all module port names */
  moduleName = rf.check("name",
                        Value("/earlyMotion"),
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
  robotName = rf.check("robot",
                       Value("icub"),
                       "Robot name (string)").asString();
  robotPortName = "/" + robotName + "/head";

  /*
  * attach a port of the same name as the module (prefixed with a /) to the module
  * so that messages received from the port are redirected to the respond method
  */
  handlerPortName = "";
  handlerPortName += getName();         // use getName() rather than a literal

  if (!handlerPort.open(handlerPortName.c_str())) {
    cout << getName() << ": Unable to open port " << handlerPortName << endl;
    return false;
  }

  attach(handlerPort);                  // attach to port

  /* create the thread and pass pointers to the module parameters */
  emThread = new earlyMotionThread();
  emThread->setName(getName().c_str());

  /* now start the thread to do the work */
  emThread->start(); // this calls threadInit() and it if returns true, it then calls run()

  threshold = (unsigned char) rf.check("threshold",
                                       Value(12),
                                       "Threshold value for white value(int)").asInt();

  return true;       // let the RFModule know everything went well
  // so that it will then run the module
}

bool earlyMotionModule::interruptModule() {
  handlerPort.interrupt();
  return true;
}

bool earlyMotionModule::close() {
  handlerPort.close();
  /* stop the thread */
  emThread->stop();
  delete emThread;
  return true;
}

bool earlyMotionModule::respond(const Bottle &command, Bottle &reply) {

  vector<string> replyScript;
  reply.clear();

  if (command.get(0).asString() == "quit") {
    reply.addString("quitting");
    return false;
  }

  bool ok = false;
  bool rec = false; // is the command recognized?

  mutex.wait();

  string helpMessage = string(getName().c_str()) +
      " commands are: \n" +
      "help \n" +
      "quit \n" +
      "set thr <n> ... set the threshold \n" +
      "(where <n> is an integer number) \n"
      "get thr get the threshold \n" ;

  switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP: {
      rec = true;
      reply.addVocab(Vocab::encode("many"));
      reply.addString(helpMessage);
      ok = true;
      break;
    }

    case COMMAND_VOCAB_SET: {
      rec = true;
      if( command.get(1).asVocab() == COMMAND_VOCAB_THRESSHOLD){
        ok =true;
        unsigned char tmpThreshold = (unsigned char) command.get(2).asInt();
        emThread->setThreshold(tmpThreshold);
        reply.addString("ok");
      }

      break;
    }
    case COMMAND_VOCAB_GET: {
      rec = true;

      if( command.get(1).asVocab() == COMMAND_VOCAB_THRESSHOLD){
        reply.addInt(emThread->getThreshold());
        ok = true;
      }

    }

    case COMMAND_VOCAB_SUS: {
      rec = true;

      
      emThread->setSuspend(true);
      ok = true;
      

    }

    case COMMAND_VOCAB_RES: {
      rec = true;

     
      emThread->setSuspend(false);
      ok = true;
      

    }

    default: {
      break;
    }
  }

  mutex.post();

  if (!rec)
    ok = RFModule::respond(command, reply);

  if (!ok) {
    reply.clear();
    reply.addVocab(COMMAND_VOCAB_FAILED);
  } else
    reply.addVocab(COMMAND_VOCAB_OK);

  return ok;

  return true;
}

/* Called periodically every getPeriod() seconds */
bool earlyMotionModule::updateModule() {
  return true;
}


