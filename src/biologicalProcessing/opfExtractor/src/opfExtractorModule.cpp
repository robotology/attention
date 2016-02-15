// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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
 * @file opfExtractorModule.cpp
 * @brief Implementation of the opfExtractorModule (see header file).
 */

#include "iCub/opfExtractorModule.h"

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

bool opfExtractorModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/oneBlobFeatExtractor"),//Value("/opfExtractor"),
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
    //Rea
    handlerPortName =  "";
    handlerPortName += getName();         // use getName() rather than a literal 

    //Carlo
    //handlerPortName = "";
    //handlerPortName += getName(rf.check("CommandPort",Value("/rpc"),"Output image port (string)").asString().c_str());

    if (!handlerPort.open(handlerPortName.c_str())) {           
        cout << getName() << ": Unable to open port " << handlerPortName << endl;  
        return false;
    }

    attach(handlerPort);                  // attach to port


    /**
	*  reading the threshold1 value
	*/ 
	double th1Level = rf.check("threshold1Level", 
                           Value(0.5),
                           "threshold 1 level for the segmentation (float)").asDouble();

    /**
	*  reading the threshold2 value
	*/ 
	double th2Level = rf.check("threshold2Level", 
                           Value(0.5), //0.7
                           "threshold 2 level for the segmentation (float)").asDouble();

    /**
	*  reading the threshold3 value
	*/ 
	double th3Level = rf.check("threshold3Level", 
                           Value(0.5),  //0.6
                           "threshold 3 level for the segmentation (float)").asDouble();

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
    rThread = new opfExtractorThread(robotName, configFile);
    rThread->setName(getName().c_str());

    rThread->setThreshold1Segmentation(th1Level);
    rThread->setThreshold2Segmentation(th2Level);
    rThread->setThreshold3Segmentation(th3Level);
    //rThread->setInputPortName(inputPortName.c_str());
    
    /* now start the thread to do the work */
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool opfExtractorModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool opfExtractorModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the thread \n");
    rThread->stop();
    return true;
}

bool opfExtractorModule::respond(const Bottle& command, Bottle& reply) 
{
    bool ok = false;
    bool rec = false; // is the command recognized?

    string helpMessage =  string(getName().c_str()) + 
                " commands are: \n" +  
                "help \n" +
                "quit \n";
    reply.clear(); 

    if(command.get(0).asString()=="newseq")
    {
        reply.addString("yeah");
        //pensare a fare qlcosa come rThread->  e da  li  passare cose a fet
        return true;
    }

    //if (command.get(0).asString()=="quit") {
    //    reply.addString("quitting");
    //    return false;     
    //}
    //else if (command.get(0).asString()=="help") {
    //    cout << helpMessage;
    //    reply.addString("ok");
    //}

    respondLock.wait();
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addVocab(Vocab::encode("many"));
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
            switch(command.get(1).asVocab()){
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
            switch(command.get(1).asVocab()){
            case COMMAND_VOCAB_WEIGHT:
                {
                    switch(command.get(2).asVocab()){
                    case COMMAND_VOCAB_HOR:
                        
                        reply.clear();
                        reply.addVocab(COMMAND_VOCAB_HOR); // ?? Needed
                        //reply.addDouble(wt);
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

    case COMMAND_VOCAB_SET:
        {
            rec = true;
            switch(command.get(1).asVocab()){
            case COMMAND_VOCAB_ALGO:
                {
                    switch(command.get(2).asVocab()){
                    case COMMAND_VOCAB_FB:
                        
                        reply.clear();
                        reply.addVocab(COMMAND_VOCAB_FB); 

                        rThread->setAlgorithm(ALGO_FB);

                        //reply.addDouble(wt);
                        rec = true;
                        ok = true;
                        break;
                        case COMMAND_VOCAB_TV:
                        
                        reply.clear();
                        reply.addVocab(COMMAND_VOCAB_TV); 

                        rThread->setAlgorithm(ALGO_TV);

                        //reply.addDouble(wt);
                        rec = true;
                        ok = true;
                        break;
                        case COMMAND_VOCAB_LK:
                        
                        reply.clear();
                        reply.addVocab(COMMAND_VOCAB_LK); 

                        rThread->setAlgorithm(ALGO_LK);

                        //reply.addDouble(wt);
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
            //rThread->suspend();
            rThread->resetFlagVisualization();
            ok = true;   
        }
        break;
    case COMMAND_VOCAB_RESUME:
        rec = true;
        {
            reply.addString("resuming processing");
            Time::delay(2.0);
            rThread->resume();
            rThread->setFlagVisualization();
            //rThread->visualizationResume();
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
        reply.addVocab(COMMAND_VOCAB_FAILED);
    }
    else
        reply.addVocab(COMMAND_VOCAB_OK);
    
    return true;
}

/* Called periodically every getPeriod() seconds */
bool opfExtractorModule::updateModule()
{
    return true;
}

double opfExtractorModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1;
}

