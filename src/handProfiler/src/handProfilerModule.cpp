// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2015  Department of Robotics Brain and Cognitive Sciences (RBCS) - Istituto Italiano di Tecnologia (IIT)
  * Author:Rea Francesco
  * email: francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file handProfilerModule.cpp
 * @brief Implementation of the handProfilerModule (see header file).
 */

#include "iCub/handProfilerModule.h"

// general command vocab's
#define COMMAND_VOCAB_IS     VOCAB2('i','s')
#define COMMAND_VOCAB_OK     VOCAB2('o','k')

#define COMMAND_VOCAB_HELP   VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_TRED   VOCAB4('t','r','e','d')
#define COMMAND_VOCAB_TGRE   VOCAB4('t','g','r','e')
#define COMMAND_VOCAB_TBLU   VOCAB4('t','b','l','u')
#define COMMAND_VOCAB_FRED   VOCAB4('f','r','e','d')       // request of fovea blob color (red)
#define COMMAND_VOCAB_FBLU   VOCAB4('f','b','l','u')       // request of fovea blob color (red)
#define COMMAND_VOCAB_MINJ   VOCAB4('m','i','n','j')       
#define COMMAND_VOCAB_TTPL   VOCAB4('T','T','P','L')      
#define COMMAND_VOCAB_MANY   VOCAB4('m','a','n','y') 
#define COMMAND_VOCAB_STAR   VOCAB4('S','T','A','R')

#define COMMAND_VOCAB_MAXDB  VOCAB3('M','d','b')           // maximum dimension of the blob drawn
#define COMMAND_VOCAB_MINDB  VOCAB3('m','d','b')           // minimum dimension of the blob drawn
#define COMMAND_VOCAB_MBA    VOCAB3('m','B','A')           // minimum dimension of the bounding area
#define COMMAND_VOCAB_SET    VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET    VOCAB3('g','e','t')
#define COMMAND_VOCAB_GEN    VOCAB3('G','E','N')
#define COMMAND_VOCAB_CON    VOCAB3('C','V','P')
#define COMMAND_VOCAB_SIM    VOCAB3('S','I','M')
#define COMMAND_VOCAB_EXE    VOCAB3('E','X','E')


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

/* 
 * Configure method. Receive a previously initialized
 * resource finder object. Use it to configure your module.
 * If you are migrating from the old Module, this is the 
 *  equivalent of the "open" method.
 */

bool handProfilerModule::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName            = rf.check("name", 
                           Value("/handProfiler"), 
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
    rThread = new handProfilerThread(robotName, configFile);
    rThread->setName(getName().c_str());
    rThread->setOutputDimension(outputWidth, outputHeight);
    //rThread->setInputPortName(inputPortName.c_str());
    
    /* now start the thread to do the work */
    rThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true ;       // let the RFModule know everything went well
                        // so that it will then run the module
}

bool handProfilerModule::interruptModule() {
    handlerPort.interrupt();
    return true;
}

bool handProfilerModule::close() {
    handlerPort.close();
    /* stop the thread */
    printf("stopping the thread \n");
    rThread->stop();
    delete rThread;
    return true;
}

bool handProfilerModule::respond(const Bottle& command, Bottle& reply) 
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

    bool ok = false;
    bool rec = false; // is the command recognized?

    mutex.wait();
    
    switch (command.get(0).asVocab()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addVocab(COMMAND_VOCAB_MANY);            
            reply.addString("help");         
            reply.addString("get fn \t: general get command");          
            reply.addString("set s1 <s> \t: general set command");
            
            reply.addString("NOTE: capitalization of command name is mandatory");
            reply.addString("set Mdb : set maximum dimension allowed for blobs");
            reply.addString("set mdb : set minimum dimension allowed for blobs");
            reply.addString("set mBA : set the minimum bounding area");

            reply.addString("get Mdb : get maximum dimension allowed for blobs");
            reply.addString("get mdb : get minimum dimension allowed for blobs");
            reply.addString("get mBA : get the minimum bounding area");

            reply.addString("GENERATE PROFILES");
            reply.addString("GEN CVP  : generate constant velocity profile");
            reply.addString("GEN MJP  : generate minimum jerk profile");
            reply.addString("GEN TTPL : generate two-third power law profile");

            reply.addString("START simulation and execute");
            reply.addString("STAR SIM : start simulation (yellow)");
            reply.addString("STAR EXE : start execution (green)");

            ok = true;
        }
        break;

    case COMMAND_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab()) {
            case COMMAND_VOCAB_MBA:
                {
                    double w = command.get(2).asDouble();
                    cout << "set mBA: " << w << endl;
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_MAXDB:
                {
                    int w = command.get(2).asInt();
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_MINDB:
                {
                    int w = command.get(2).asInt();
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_TRED:
                {
                    int t = command.get(2).asInt();
                    reply.addString("trg:");
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_TGRE:
                {
                    int t = command.get(2).asInt();
                    reply.addString("trg:");
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_TBLU:
                {
                    int t = command.get(2).asInt();
                    reply.addString("trg:");
                    ok=true;
                }
                break;
            default:
                cout << "received an unknown request " << endl;
                break;
            }
        }
        break;

    case COMMAND_VOCAB_GET:
        rec = true;
        {
            //reply.addVocab(COMMAND_VOCAB_IS);
            //reply.add(command.get(1));
            switch(command.get(1).asVocab()) {

            case COMMAND_VOCAB_MAXDB:
                {

                    reply.addInt(0);
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_MINDB:
                {

                    reply.addInt(0);
                    ok = true;
                }
                break;
               
            /* LATER: implement case COMMAND_VOCAB_MBA */

            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
        }
        break;
    case COMMAND_VOCAB_STAR:
        rec = true;
        {
            switch(command.get(1).asVocab()) {

            case COMMAND_VOCAB_SIM:
                {
                    if(0!=rThread) {
                        reply.addString("OK");
                        rThread->startSimulation(false);    
                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_EXE:
                {
                    if(0!=rThread) {
                        reply.addString("OK");
                        rThread->startExecution(false);    
                    }
                    ok = true;
                }
                break;
               

            default:
                cout << "received an unknown request after a STAR" << endl;
                break;
            }
            
            ok = true;
        }
        break;

    case COMMAND_VOCAB_GEN:
        rec = true;
        {
            //reply.addVocab(COMMAND_VOCAB_IS);
            //reply.add(command.get(1));
            switch(command.get(1).asVocab()) {

            case COMMAND_VOCAB_CON:
                {
                    rec = true;                
                    reply.addString("constant"); 
                    if(0!=rThread){
                        /*Bottle b;
                        Bottle bA;
                        bA.addDouble(-0.3);bA.addDouble(0.0);bA.addDouble(0.1);
                        b.addList() = bA;
                        Bottle bB;
                        bB.addDouble(-0.3);bB.addDouble(-0.1);bB.addDouble(0.2);
                        b.addList() = bB;
                        Bottle bC;
                        bC.addDouble(-0.3);bC.addDouble(-0.1);bC.addDouble(0.0);
                        b.addList() = bC;
                        Bottle bAngles;
                        bAngles.addDouble(0.0);bAngles.addDouble(1.5708);bAngles.addDouble(4.7128);
                        b.addList() = bAngles;
                        Bottle bParam;
                        bParam.addDouble(0.1);bParam.addDouble(0.1);;
                        b.addList() = bParam;   
                        Bottle finalB;
                        finalB.addList() = b;
                        */
    
                        //GEN CVP (((-0.3 -0.0 0.1) (-0.3 -0.1 0.2) (-0.3 -0.1 0.0) (0.0 1.57 4.71) (0.1 0.1)))

                        if(command.size() == 3) {                        
                            Bottle* finalB = command.get(2).asList();                        
                            yDebug("bottle in threadInit %s", finalB->toString().c_str());                                     
                            if(rThread->factory("CVP", *finalB)) {
                                yInfo("factory:constant");                           
                                //reply.addString("OK");
                                ok = true;
                            }
                        }
                    }
                }
            break;
            case COMMAND_VOCAB_MINJ:
                {
                    rec = true;                
                    reply.addString("minJerk"); 
                    if(0!=rThread){
                        Bottle finalB;                  
                        if(rThread->factory("MJP",finalB)) {
                            yInfo("factory:minJerk");
                            reply.addInt(1);
                            ok = true;
                        } 
                    }  
                }
                break;
            case COMMAND_VOCAB_TTPL:
                {
                    rec = true;                
                    reply.addString("twoThirdPowerLaw"); 
                    if(0!=rThread){
                        //GEN TTPL (((-0.3 -0.0 0.1) (-0.3 -0.1 0.2) (-0.3 -0.1 0.0) (0.0 1.57 4.71) (0.1 0.1)))
                        if(command.size() == 3) {                        
                            Bottle* finalB = command.get(2).asList();                        
                            yDebug("bottle in threadInit %s", finalB->toString().c_str());                                     
                            if(rThread->factory("TTPL", *finalB)) {
                                yInfo("factory:ttpl");                           
                                ok = true;
                            }
                        }
                    }
                }
                break;
            
            
            /* LATER: implement case COMMAND_VOCAB_MBA */

            default:
                cout << "received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                break;
            }
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

    return ok;
    

}

/* Called periodically every getPeriod() seconds */
bool handProfilerModule::updateModule()
{
    return true;
}

double handProfilerModule::getPeriod()
{
    /* module periodicity (seconds), called implicitly by myModule */
    return 1.0;
}

