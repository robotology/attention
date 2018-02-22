// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff Andrew Dankers
 * email:   vadim.tikhanoff@iit.it
 * website: www.robotcub.org
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

#include "../include/iCub/zeroDisparityModule.h"


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

bool zeroDisparityFilterMod::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName = rf.check("name",
                          Value("zeroDisparityFilterMod"),
                          "module name (string)").asString();

    /*
    Default Ini file values: #amaroyo 03/03/2016

        max_iteration 1
        randomize_iteration 0
        smoothness_penalty_base 50
        smoothness_penalty 600
        data_penalty 106
        smoothness_3sigmaon2 13
        bland_dog_thresh 1
        radial_penalty 50
        acquire_wait 25
        min_area  5000
        max_area  10000
        max_spread 50
        cog_snap 0.3
        bland_prob 0.3

    */

    setName(moduleName.c_str());
    parameters.iter_max = rf.findGroup("PARAMS").check("max_iteration", Value(3000), "what did the user select?").asInt();
    parameters.randomize_every_iteration = rf.findGroup("PARAMS").check("randomize_iteration", Value(0),
                                                                        "what did the user select?").asInt();
    parameters.smoothness_penalty_base = rf.findGroup("PARAMS").check("smoothness_penalty_base", Value(50),
                                                                      "what did the user select?").asInt();
    parameters.smoothness_penalty = rf.findGroup("PARAMS").check("smoothness_penalty", Value(600),
                                                                 "what did the user select?").asInt();
    parameters.data_penalty = rf.findGroup("PARAMS").check("data_penalty", Value(106),
                                                           "what did the user select?").asInt();
    parameters.smoothness_3sigmaon2 = rf.findGroup("PARAMS").check("smoothness_3sigmaon2", Value(13),
                                                                   "what did the user select?").asInt();
    parameters.bland_dog_thresh = rf.findGroup("PARAMS").check("bland_dog_thresh", Value(250),
                                                               "what did the user select?").asInt();
    parameters.radial_penalty = rf.findGroup("PARAMS").check("radial_penalty", Value(50),
                                                             "what did the user select?").asInt();
    parameters.acquire_wait = rf.findGroup("PARAMS").check("acquire_wait", Value(25),
                                                           "what did the user select?").asInt();
    parameters.min_area = rf.findGroup("PARAMS").check("min_area", Value(5000), "what did the user select?").asInt();
    parameters.max_area = rf.findGroup("PARAMS").check("max_area", Value(10000), "what did the user select?").asInt();
    parameters.max_spread = rf.findGroup("PARAMS").check("max_spread", Value(50), "what did the user select?").asInt();
    parameters.cog_snap = rf.findGroup("PARAMS").check("cog_snap", Value(0.3), "what did the user select?").asDouble();
    parameters.bland_prob = rf.findGroup("PARAMS").check("bland_prob", Value(0.3),
                                                         "what did the user select?").asDouble();

    parameters.fovea_width = rf.findGroup("PARAMS").check("fovea_size", Value(128),
                                                          "what did the user select?").asInt();
    parameters.fovea_height = rf.findGroup("PARAMS").check("fovea_size", Value(128),
                                                           "what did the user select?").asInt();

    parameters.sigma1 = rf.findGroup("PARAMS").check("sigma1", Value(25),
                                                     "what did the user select?").asDouble();
    parameters.sigma2 = rf.findGroup("PARAMS").check("sigma2", Value(2),
                                                     "what did the user select?").asDouble();


    //  Disparity choice of algorithm by default NDT
    parameters.rankOrNDT = rf.findGroup("PARAMS").check("rankOrNDT", Value(1),  "what did the user select?").asBool();


    //NDT parameters
    parameters.ndtX = rf.findGroup("PARAMS").check("ndtX", Value(4),  "what did the user select?").asInt();
    parameters.ndtY = rf.findGroup("PARAMS").check("ndtY", Value(4),  "what did the user select?").asInt();
    parameters.ndtSize = rf.findGroup("PARAMS").check("ndtSize", Value(14),  "what did the user select?").asInt();
    parameters.ndtEQ = rf.findGroup("PARAMS").check("ndtEQ", Value(50),  "what did the user select?").asInt();

    //RANK parameters
    parameters.rankX = rf.findGroup("PARAMS").check("rankX", Value(2),  "what did the user select?").asInt();
    parameters.rankY = rf.findGroup("PARAMS").check("rankY", Value(2),  "what did the user select?").asInt();
    parameters.rankSize = rf.findGroup("PARAMS").check("rankSize", Value(8),  "what did the user select?").asInt();


    /*
     * attach a port of the same name as the module (prefixed with a /) to the module
     * so that messages received from the port are redirected to the respond method
     */

    workWith = rf.check("with", Value("nothing"), "work with arbiter (string)").asString();


    handlerName = "/";
    handlerName += getName();         // use getName() rather than a literal

    if (!handlerPort.open(handlerName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerName << endl;
        return false;
    }

    attach(handlerPort);               // attach to port
    //attachTerminal();                // attach to terminal (maybe not such a good thing...)

    /* create the thread and pass pointers to the module parameters */
    zdfThread = new ZDFThread(&parameters, workWith);

    /*pass the name of the module in order to create ports*/
    zdfThread->setName(moduleName);

    /* now start the thread to do the work */
    zdfThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true;
}

/* Called periodically every getPeriod() seconds */

bool zeroDisparityFilterMod::updateModule() {
    return true;
}

bool zeroDisparityFilterMod::interruptModule() {

    handlerPort.interrupt();
    return true;
}

bool zeroDisparityFilterMod::close() {
    handlerPort.close();
    zdfThread->stop();
    cout << "deleting thread " << endl;
    delete zdfThread;
    return true;
}

double zeroDisparityFilterMod::getPeriod() {
    return 0.1;
}


bool zeroDisparityFilterMod::respond(const Bottle &command, Bottle &reply) {

    //bool ok = false;
    //bool rec = false; // is the command recognized?

    // mutex.wait();
    switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_HELP: {
            //rec = true;
            string helpMessage = string(getName().c_str()) +
                                 " commands are: \n" +
                                 "help \n" +
                                 "quit \n";
            reply.clear();
            //ok = true;
            break;
        }

        case COMMAND_VOCAB_SET: {
            //rec = true;
            switch (command.get(1).asVocab()) {

                case COMMAND_VOCAB_K1: {
                    int w = command.get(2).asInt();
                    zdfThread->params->data_penalty = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K2: {
                    int w = command.get(2).asInt();
                    zdfThread->params->smoothness_penalty_base = w;
                    //ok = true;
                    break;

                }

                case COMMAND_VOCAB_K3: {
                    int w = command.get(2).asInt();
                    zdfThread->params->smoothness_penalty = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K4: {
                    int w = command.get(2).asInt();
                    zdfThread->params->radial_penalty = w;
                    //ok = true;
                    break;

                }

                case COMMAND_VOCAB_K5: {
                    int w = command.get(2).asInt();
                    zdfThread->params->smoothness_3sigmaon2 = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K6: {
                    int w = command.get(2).asInt();
                    zdfThread->params->bland_dog_thresh = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K7: {
                    int w = command.get(2).asInt();
                    zdfThread->params->sigma1 = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K8: {
                    int w = command.get(2).asInt();
                    zdfThread->params->sigma2 = w;
                    //ok = true;
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case COMMAND_VOCAB_GET: {
            //rec = true;

            reply.addVocab(COMMAND_VOCAB_IS);
            reply.add(command.get(1));

            switch (command.get(1).asVocab()) {

                case COMMAND_VOCAB_K1: {
                    double w = zdfThread->params->data_penalty;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K2: {
                    double w = zdfThread->params->smoothness_penalty_base;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K3: {
                    double w = zdfThread->params->smoothness_penalty;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K4: {
                    double w = zdfThread->params->radial_penalty;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K5: {
                    double w = zdfThread->params->smoothness_3sigmaon2;
                    reply.addDouble(w);
                    break;
                    //ok = true;
                }

                case COMMAND_VOCAB_K6: {
                    double w = zdfThread->params->bland_dog_thresh;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K7: {
                    double w = zdfThread->params->sigma1;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K8: {
                    double w = zdfThread->params->sigma2;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                default: {
                    break;
                }
            }
            break;
        }
        default:
            break;
    }

    return true;
}