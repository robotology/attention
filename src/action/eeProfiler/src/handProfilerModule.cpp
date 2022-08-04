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
const int32_t  COMMAND_VOCAB_IS      = yarp::os::createVocab32('i','s');
const int32_t  COMMAND_VOCAB_OK      = yarp::os::createVocab32('o','k');
const int32_t  COMMAND_VOCAB_UP      = yarp::os::createVocab32('U','P');
const int32_t  COMMAND_VOCAB_ON      = yarp::os::createVocab32('O','N');


const int32_t  COMMAND_VOCAB_HELP    = yarp::os::createVocab32('h','e','l','p');
const int32_t  COMMAND_VOCAB_FAILED  = yarp::os::createVocab32('f','a','i','l');
const int32_t  COMMAND_VOCAB_TRED    = yarp::os::createVocab32('t','r','e','d');
const int32_t  COMMAND_VOCAB_TGRE    = yarp::os::createVocab32('t','g','r','e');
const int32_t  COMMAND_VOCAB_TBLU    = yarp::os::createVocab32('t','b','l','u');
const int32_t  COMMAND_VOCAB_FRED    = yarp::os::createVocab32('f','r','e','d');       // request of fovea blob color (red)
const int32_t  COMMAND_VOCAB_FBLU    = yarp::os::createVocab32('f','b','l','u');       // request of fovea blob color (red)
const int32_t  COMMAND_VOCAB_MINJ    = yarp::os::createVocab32('m','i','n','j');
const int32_t  COMMAND_VOCAB_TTPL    = yarp::os::createVocab32('T','T','P','L');
const int32_t  COMMAND_VOCAB_MANY    = yarp::os::createVocab32('m','a','n','y');
const int32_t  COMMAND_VOCAB_XAXI    = yarp::os::createVocab32('X','A','X','I');
const int32_t  COMMAND_VOCAB_YAXI    = yarp::os::createVocab32('Y','A','X','I');
const int32_t  COMMAND_VOCAB_ZAXI    = yarp::os::createVocab32('Z','A','X','I');
const int32_t  COMMAND_VOCAB_STAR    = yarp::os::createVocab32('S','T','A','R');
const int32_t  COMMAND_VOCAB_PALM    = yarp::os::createVocab32('P','A','L','M');
const int32_t  COMMAND_VOCAB_DOWN    = yarp::os::createVocab32('D','O','W','N');
const int32_t  COMMAND_VOCAB_CURR    = yarp::os::createVocab32('C','U','R','R');
const int32_t  COMMAND_VOCAB_SAVE    = yarp::os::createVocab32('S','A','V','E');        //save action joints in a file
const int32_t  COMMAND_VOCAB_FILE    = yarp::os::createVocab32('F','I','L','E');        //
const int32_t  COMMAND_VOCAB_SPEED   = yarp::os::createVocab32('S','P','E','E');        //start movement from joints file
const int32_t  COMMAND_VOCAB_MARK    = yarp::os::createVocab32('M','A','R','K');        //to generate a time interval with start/stop
const int32_t  COMMAND_VOCAB_STOP    = yarp::os::createVocab32('S','T','O','P');        //
const int32_t  COMMAND_VOCAB_SYNC    = yarp::os::createVocab32('S','Y','N','C');        //set the time interval generated with MARK
const int32_t  COMMAND_VOCAB_REPS    = yarp::os::createVocab32('R','E','P','S');        //set the number of repetitions for file movement
const int32_t  COMMAND_VOCAB_LOAD    = yarp::os::createVocab32('L','O','A','D');        //load a file
const int32_t  COMMAND_VOCAB_TIME    = yarp::os::createVocab32('T','I','M','E');        //
const int32_t  COMMAND_VOCAB_GRAS    = yarp::os::createVocab32('G','R','A','S');        //grasp on
const int32_t  COMMAND_VOCAB_PART    = yarp::os::createVocab32('P','A','R','T');


const int32_t  COMMAND_VOCAB_MAXDB   = yarp::os::createVocab32('M','d','b');           // maximum dimension of the blob drawn
const int32_t  COMMAND_VOCAB_MINDB   = yarp::os::createVocab32('m','d','b');           // minimum dimension of the blob drawn
const int32_t  COMMAND_VOCAB_MBA     = yarp::os::createVocab32('m','B','A');           // minimum dimension of the bounding area
const int32_t  COMMAND_VOCAB_SET     = yarp::os::createVocab32('s','e','t');
const int32_t  COMMAND_VOCAB_GET     = yarp::os::createVocab32('g','e','t');
const int32_t  COMMAND_VOCAB_GEN     = yarp::os::createVocab32('G','E','N');
const int32_t  COMMAND_VOCAB_CON     = yarp::os::createVocab32('C','V','P');
const int32_t  COMMAND_VOCAB_MJP     = yarp::os::createVocab32('M','J','P');
const int32_t  COMMAND_VOCAB_SIM     = yarp::os::createVocab32('S','I','M');
const int32_t  COMMAND_VOCAB_CLR     = yarp::os::createVocab32('C','L','R');
const int32_t  COMMAND_VOCAB_EXE     = yarp::os::createVocab32('E','X','E');
const int32_t  COMMAND_VOCAB_ROT     = yarp::os::createVocab32('R','O','T');
const int32_t  COMMAND_VOCAB_REV     = yarp::os::createVocab32('R','E','V');
const int32_t  COMMAND_VOCAB_RES     = yarp::os::createVocab32('R','E','S');
const int32_t  COMMAND_VOCAB_TTL     = yarp::os::createVocab32('T','T','L');
const int32_t  COMMAND_VOCAB_CUS     = yarp::os::createVocab32('C','U','S');
const int32_t  COMMAND_VOCAB_JOI     = yarp::os::createVocab32('J','O','I');            //save action joints in a file
const int32_t  COMMAND_VOCAB_OFF     = yarp::os::createVocab32('O','F','F');

const int32_t  COMMAND_VOCAB_GRAZ     = yarp::os::createVocab32('G','R','A','Z');
const int32_t  COMMAND_VOCAB_CVV      = yarp::os::createVocab32('C','V','V');


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
    Vector tmp(4);
    tmp[0] = -0.096; tmp[1] = 0.513; tmp[2] = -0.8528; tmp[3] = 2.514;
    vectorDownOrientation = tmp;
    /* Process all parameters from both command-line and .ini file */
    if(rf.check("help")) {
        printf("HELP \n");
        printf("====== \n");
        printf("--name           : changes the rootname of the module ports \n");
        printf("--robot          : changes the name of the robot where the module interfaces to  \n");
        printf("--name           : rootname for all the connection of the module \n");
        printf("--part           : selected arm \n");
        printf("--pitchDof       : 0/1 disable/enable the DoF \n");
        printf("--yawDof         : 0/1 disable/enable the DoF \n");
        printf("--rollDof        : 0/1 disable/enable the DoF \n");
        printf("--gazeTracking   : enable gaze tracking \n");
        printf(" \n");
        printf("press CTRL-C to stop... \n");
        return true;
    }

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
                           "output image width (int)").asInt16();
   string  part       = rf.check("part",
                          Value(""),
                          "selected part (string)").asString();
    int  outputHeight      = rf.check("outputHeight",
                           Value(240),
                           "output image height (int)").asInt16();
    int  yawDof           = rf.check("yawDof",
                                     Value(0),
                                     "value of the yawDofl(int)").asInt16();
    int  rollDof          = rf.check("rollDof",
                                     Value(0),
                                     "value of the rollDof(int)").asInt16();
    int  pitchDof         = rf.check("pitchDof",
                                     Value(0),
                                     "value of the pitchRoll(int)").asInt16();

    bool gazeTracking     = rf.check("gazeTracking");
    if(gazeTracking) {
        yInfo("gazeTracking ON");
    }
    else {
        yInfo("gazeTracking OFF");
    }

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
    rThread = new handProfilerThread(robotName, configFile, rf);
    rThread->setName(getName().c_str());
    rThread->setPart(part);
    rThread->setTorsoDof(yawDof, rollDof, pitchDof);
    rThread->setGazeTracking(gazeTracking);
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
    //delete rThread;
    printf("success in stopping the thread");
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
    //else if (command.get(0).asString()=="help") {
    //    cout << helpMessage;
    //    reply.addString("ok");
    //}

    bool ok = false;
    bool rec = false; // is the command recognized?

    mutex.wait();

    switch (command.get(0).asVocab32()) {
    case COMMAND_VOCAB_HELP:
        rec = true;
        {
            reply.addVocab32(Vocab32::encode("many"));
            reply.addString("help");
            reply.addString("get fn \t: general get command");
            reply.addString("set s1 <s> \t: general set command");

            reply.addString("GENERATE PROFILES (left hand)");
            reply.addString("GEN CVP  : generate constant velocity profile");
	          reply.addString("         : (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.1) (rev) (param 0.1)))");
            reply.addString("GEN MJP  : generate minimum jerk profile");
	          reply.addString("         : (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.2) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.1) (rev) (param 1.57 3.0)))");
            reply.addString("GEN TTPL : generate NON two-third power law profile");
            reply.addString("         : (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.1 0.33)))");
            reply.addString("GEN TTL : generate two-third power law profile");
            reply.addString("         : (((O -0.3 -0.1 0.1) (A -0.3 -0.0 0.1) (B -0.3 -0.1 0.3) (C -0.3 -0.1 0.0) (theta 0.0 1.57 4.71) (axes 0.1 0.2) (rev) (param 0.01 0.33)))");

            reply.addString("PALM CUS : to change the orientation of the palm ");
            reply.addString("PALM CUS (-0.076 -0.974 0.213 3.03) ");

            reply.addString("START simulation and execute");
            reply.addString("STAR SIM : start simulation (yellow)");
            reply.addString("STAR EXE : start execution  (green)");
            reply.addString("STAR RES : start resetting of the posture");
            reply.addString("SIM CLR  : simulator cleaning");
            reply.addString("STAR FILE SPEE #value: start execution from file with speed multiplied for the value, if omitted default value is 1.0");
            reply.addString("SAVE JOI: save joints positions in file");
            reply.addString("MARK START/STOP: start/stop time interval");
            reply.addString("SYNC: set time interval decided with MARK START/STOP");
            reply.addString("REPS #value: set number of repetitions for file execution");
            reply.addString("LOAD FILE: set the name of the file to load");
            reply.addString("GRAS ON: turn on grasping");
            reply.addString("GRAS RES: reset the open hand");
            reply.addString("GRAZ CVV (params): constant velocity grasping");
            reply.addString("GRAZ CVV (((t 0.2 0.2 0.2) (p 0.3 0.3 0.3))) : power grasp");
            reply.addString("GRAZ CVV (((pitch 0.1) (time 0.2 0.2 0.2) (param 0.3 0.3 0.3))) : pitch grasp");
            reply.addString("example movement JULIA (left arm):  GEN TTL (((O -0.3 -0.15 0.05) (A -0.3 -0.3 0.05) (B -0.3 -0.15 0.1) (C -0.3 -0.0 0.05) (theta 0.0 1.57 3.14) (axes 0.15 0.05) (param 0.01 0.33)))");

            ok = true;
        }
        break;

    case COMMAND_VOCAB_SET:
        rec = true;
        {
            switch(command.get(1).asVocab32()) {
            case COMMAND_VOCAB_MBA:
                {
                    double w = command.get(2).asFloat32();
                    cout << "set mBA: " << w << endl;
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_MAXDB:
                {
                    int w = command.get(2).asInt16();
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_MINDB:
                {
                    int w = command.get(2).asInt16();
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_TRED:
                {
                    int t = command.get(2).asInt16();
                    reply.addString("trg:");
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_TGRE:
                {
                    int t = command.get(2).asInt16();
                    reply.addString("trg:");
                    ok=true;
                }
                break;
            case COMMAND_VOCAB_TBLU:
                {
                    int t = command.get(2).asInt16();
                    reply.addString("trg:");
                    ok=true;
                }
                break;
            default:
                cout << "received an unknown request after SET" << endl;
                break;
            }
        }
        break;

    case COMMAND_VOCAB_GET:
        rec = true;
        {
            //reply.addVocab32(COMMAND_VOCAB_IS);
            //reply.add(command.get(1));
            switch(command.get(1).asVocab32()) {

            case COMMAND_VOCAB_MAXDB:
                {

                    reply.addInt16(0);
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_MINDB:
                {

                    reply.addInt16(0);
                    ok = true;
                }
                break;
            case COMMAND_VOCAB_PART:
                {

                    rThread->getPart();
                    ok = true;
                }
                break;

            /* LATER: implement case COMMAND_VOCAB_MBA */

            default:
                cout << "received an unknown request after a GET" << endl;
                break;
            }
        }
        break;
    case COMMAND_VOCAB_PALM:
        rec = true;
        {
            //reply.addVocab32(COMMAND_VOCAB_IS);
            //reply.add(command.get(1));
            switch(command.get(1).asVocab32()) {

            case COMMAND_VOCAB_DOWN:
                {
                    rThread->setOrientation(vectorDownOrientation);
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_UP:
                {
                    rThread->setOrientation(vectorUpOrientation);
                    ok = true;
                }
                break;
            case COMMAND_VOCAB_CURR:
                {
                    rThread->setCurrentOrientation();
                    ok = true;
                }
                break;
             case COMMAND_VOCAB_CUS:
                {
                    Bottle* b = command.get(2).asList();
                    Vector v(4);
                    v(0) = b->get(0).asFloat32();
                    v(1) = b->get(1).asFloat32();
                    v(2) = b->get(2).asFloat32();
                    v(3) = b->get(3).asFloat32();
                    yInfo("PALM CUSTOM setting to vector %s", v.toString().c_str());
                    //rThread->setOrientation(vectorUpOrientation);
                    if(v.size() == 4) {
                        ok = rThread->setOrientation(v);
                    }
                    else {
                        ok = false;
                    }
                }
                break;

            /* LATER: implement case COMMAND_VOCAB_MBA */

            default:
                cout << "received an unknown request after a GET" << endl;
                break;
            }
        }
        break;
    case COMMAND_VOCAB_STAR:
        rec = true;
        {
            switch(command.get(1).asVocab32()) {
            case COMMAND_VOCAB_RES:
                {
                    bool rev = false;
                    if(0!=rThread) {
                        reply.addString("OK");
                        rThread->startResetting();

                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_SIM:
                {
                    bool rev = false;
                    if(0!=rThread) {
                        reply.addString("OK");

                        if(command.size() == 3) {
                            yInfo("Looking for REV COMMNAD");
                            if(command.get(2).asVocab32() == COMMAND_VOCAB_REV) {
                                yInfo("REV COMMAND found");
                                rev = true;
                            }
                        }
                        rThread->startSimulation(rev);

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
            case COMMAND_VOCAB_FILE:
                {
                    if(command.get(2).asVocab32() == COMMAND_VOCAB_SPEED && 0!=rThread) {
                        if(command.get(3).isFloat32() || command.get(3).isInt16()){
                            reply.addString("OK");
                            rThread->startJoints(command.get(3).asFloat32());
                            ok = true;
                        }else{
                            // when we have SPEE command but no value
                            reply.addString("OK");
                            rThread->startJoints(1.0);
                            ok = true;
                        }
                    }else{
                        // when we have no SPEE command
                        if(0!=rThread) {
                            reply.addString("OK");


                            rThread->startJoints(1.0);
                        }
                        ok = true;
                        }
                }
            break;


            default:
                cout << "received an unknown request after a STAR" << endl;
                break;
            }

            ok = true;
        }
        break;
    case COMMAND_VOCAB_SAVE:
        rec = true;
        {
            switch(command.get(1).asVocab32()) {

            case COMMAND_VOCAB_JOI:
                {

                    if(0!=rThread) {
                        reply.addString("OK");

                        rThread->saveJoints();

                    }
                    ok = true;
                }
            break;

            default:
                cout << "received an unknown request" << endl;
                break;
            }

            ok = true;
        }
        break;
    case COMMAND_VOCAB_GRAS:
        rec = true;
        {
            switch(command.get(1).asVocab32()) {
            case COMMAND_VOCAB_ON:
                {
                    if(0!=rThread) {
                        reply.addString("OK");                     
                        rThread->setGrasp(true);
                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_OFF:
                {
                    if(0!=rThread) {
                        reply.addString("OK");
                        rThread->setGrasp(false);
                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_RES:
                {
                    if(0!=rThread) {
                        reply.addString("OK");
                        rThread->graspReset();
                    }
                    ok = true;
                }
            break;
            default:
                cout << "received an unknown request" << endl;
                break;
            }
            ok = true;
        }
        break;
    case COMMAND_VOCAB_GRAZ:
        rec = true;
        {
            switch(command.get(1).asVocab32()) {
            case COMMAND_VOCAB_CVV:
                {
                    yInfo("activating grasp costant velocity with VIAPOINT");
                    if(0!=rThread) {
                        reply.addString("OK");
                        //GRAZ CVV ((power) (0.2 0.2 0.2) (0.3 0.3 0.3))

                        if(command.size() == 3) {
                            Bottle* finalB = command.get(2).asList();
                            yDebug("bottle in threadInit %s", finalB->toString().c_str());
                            if(rThread->fingerfactory("CVV", *finalB)) {
                                yInfo("fingerfactory:constant velocity viaPoint");
                                //reply.addString("OK");
                                ok = true;
                            }
                            else{
                                yError("finger factory error");
                            }
                        }
                        rThread->setGrasp(true);
                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_ON:
                {

                    if(0!=rThread) {
                        reply.addString("OK");                        
                        rThread->setGrasp(true);

                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_OFF:
                {

                    if(0!=rThread) {
                        reply.addString("OK");

                        rThread->setGrasp(false);

                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_RES:
                {

                    if(0!=rThread) {
                        reply.addString("OK");
                        rThread->graspReset();

                    }
                    ok = true;
                }
            break;

            default:
                cout << "received an unknown request" << endl;
                break;
            }

            ok = true;
        }
        break;
    case COMMAND_VOCAB_LOAD:
        rec = true;
        {
            if(command.get(1).asVocab32() == COMMAND_VOCAB_FILE) {
                if(0!=rThread) {
                    reply.addString("OK");

                    rThread->loadFile(command.get(2).asString());

                }
                ok = true;
            }
        }
        break;
    case COMMAND_VOCAB_REPS:
        rec = true;
        {
            if(0!=rThread && command.get(1).asInt16() != 0) {
              reply.addString("OK");

              rThread->setRepsNumber(command.get(1).asInt16());

            }
            ok = true;
        }
        break;
    case COMMAND_VOCAB_MARK:
        {
            rec = true;
            switch(command.get(1).asVocab32()) {

            case COMMAND_VOCAB_STAR:
                {

                    if(0!=rThread) {
                        reply.addString("OK");

                        rThread->setPartnerStart();

                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_STOP:
                {

                    if(0!=rThread) {
                        reply.addString("OK");

                        rThread->setPartnerStop();

                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_TIME:
                {

                    if(0!=rThread && command.get(2).asFloat32() != 0) {
                        reply.addString("OK");

                        rThread->setPartnerTime(command.get(2).asFloat32());

                    }
                    ok = true;
                }
            break;

            default:
                cout << "received an unknown request" << endl;
                break;
            }

            ok = true;
        }
        break;
    case COMMAND_VOCAB_SYNC:
        {
            rec = true;
            if(0!=rThread) {
                reply.addString("OK");

                rThread->setPartnerTime(0.0);

            }
            ok = true;
        }
        break;
    case COMMAND_VOCAB_ROT:
        rec = true;
        {
            switch(command.get(1).asVocab32()) {

            case COMMAND_VOCAB_XAXI:
                {
                    if(0!=rThread) {
                        reply.addString("OK");
                        rThread->rotAxisX(5.0);
                    }
                    ok = true;
                }
            break;
            case COMMAND_VOCAB_YAXI:
                {
                    if(0!=rThread) {
                        reply.addString("OK");
                        rThread->rotAxisY(5.0);
                    }
                    ok = true;
                }
                break;
            case COMMAND_VOCAB_ZAXI:
                {
                    if(0!=rThread) {
                        reply.addString("OK");
                        rThread->rotAxisZ(5.0);
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
    case COMMAND_VOCAB_SIM:
        rec = true;
        {
            switch(command.get(1).asVocab32()) {

            case COMMAND_VOCAB_CLR:
                {
                    if(0!=rThread) {
                        reply.addString("OK");

                    }
                    ok = true;
                }
            break;
            }

        }
        break;
    case COMMAND_VOCAB_GEN:
        rec = true;
        {
            yInfo("GEN %s ", command.get(1).asString().c_str());
            //reply.addVocab32(COMMAND_VOCAB_IS);
            //reply.add(command.get(1));
            switch(command.get(1).asVocab32()) {

            case COMMAND_VOCAB_CON:
                {
                    rec = true;
                    reply.addString("constant");
                    if(0!=rThread){
                        /*Bottle b;
                        Bottle bA;
                        bA.addFloat32(-0.3);bA.addFloat32(0.0);bA.addFloat32(0.1);
                        b.addList() = bA;
                        Bottle bB;
                        bB.addFloat32(-0.3);bB.addFloat32(-0.1);bB.addFloat32(0.2);
                        b.addList() = bB;
                        Bottle bC;
                        bC.addFloat32(-0.3);bC.addFloat32(-0.1);bC.addFloat32(0.0);
                        b.addList() = bC;
                        Bottle bAngles;
                        bAngles.addFloat32(0.0);bAngles.addFloat32(1.5708);bAngles.addFloat32(4.7128);
                        b.addList() = bAngles;
                        Bottle bParam;
                        bParam.addFloat32(0.1);bParam.addFloat32(0.1);;
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
            case COMMAND_VOCAB_MJP:
                {
                    rec = true;
                    reply.addString("minJerk");
                    if(0!=rThread){
                        Bottle* finalB = command.get(2).asList();
                        if(rThread->factory("MJP",*finalB)) {
                            yInfo("factory:minJerk");
                            ok = true;
                        }
                    }
                }
                break;
            case COMMAND_VOCAB_TTPL:
                {
                    rec = true;
                    reply.addString("nonBioLaw");
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
            case COMMAND_VOCAB_TTL:
                {
                    rec = true;
                    reply.addString("twoThirdPowerLaw");
                    if(0!=rThread){
                        //GEN TT (((-0.3 -0.0 0.1) (-0.3 -0.1 0.2) (-0.3 -0.1 0.0) (0.0 1.57 4.71) (0.1 0.1)))
                        if(command.size() == 3) {
                            Bottle* finalB = command.get(2).asList();
                            yDebug("bottle in threadInit %s", finalB->toString().c_str());
                            if(rThread->factory("TwoThird", *finalB)) {
                                yInfo("factory:tt");
                                ok = true;
                            }
                        }
                    }
                }
                break;

            /* LATER: implement case COMMAND_VOCAB_MBA */

            default:
                cout << "received an unknown request after GEN" << endl;
                break;
            }
        }
        break;
    }
    mutex.post();
    if (!rec){
        ok = RFModule::respond(command,reply);
    }
    if (!ok) {
        reply.clear();
        reply.addVocab32(COMMAND_VOCAB_FAILED);
    }
    else{
        reply.addVocab32(COMMAND_VOCAB_OK);
    }

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
