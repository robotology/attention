/*
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
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

/**
*
* @ingroup icub_logpolarAttention
* \defgroup icub_zeroDisparityFilter zeroDisparityFilter
*
* This module performs zero disparity filtering. Given the object at zero disparity segments the silhouette of the object
*/
#ifndef ZERODISPARITYFILTER_ZERODISPARITYMODULE_H
#define ZERODISPARITYFILTER_ZERODISPARITYMODULE_H

#include <iCub/zdfThread.h>
#include <yarp/os/RFModule.h>


class zeroDisparityFilterMod : public yarp::os::RFModule {

    enum {
        COMMAND_VOCAB_IS = yarp::os::createVocab('i', 's'),
        COMMAND_VOCAB_HELP = yarp::os::createVocab('h', 'e', 'l', 'p'),
        COMMAND_VOCAB_SET = yarp::os::createVocab('s', 'e', 't'),
        COMMAND_VOCAB_GET = yarp::os::createVocab('g', 'e', 't'),
        COMMAND_VOCAB_K1 = yarp::os::createVocab('k', '1'), //data penalty
        COMMAND_VOCAB_K2 = yarp::os::createVocab('k', '2'), //smoothness penalty base
        COMMAND_VOCAB_K3 = yarp::os::createVocab('k', '3'), //smoothness penalty
        COMMAND_VOCAB_K4 = yarp::os::createVocab('k', '4'),//radial penalty
        COMMAND_VOCAB_K5 = yarp::os::createVocab('k', '5'), //smoothness 3sigmaon2
        COMMAND_VOCAB_K6 = yarp::os::createVocab('k', '6'), //bland dog thresh
        COMMAND_VOCAB_K7 = yarp::os::createVocab('k', '7'), //bland prob
        COMMAND_VOCAB_K8 = yarp::os::createVocab('k', '8'), //max spread
        COMMAND_VOCAB_FAILED = yarp::os::createVocab('f', 'a' , 'i', 'l'), //max spread
        COMMAND_VOCAB_OK = yarp::os::createVocab('o', 'k') //max spread
    };


    /* module parameters */
    std::string moduleName;
    std::string handlerName;
    std::string workWith;

    yarp::os::Port handlerPort;      //a port to handle messages

    struct MultiClass::Parameters parameters; // multi class parameters passed to the thread
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    ZDFThread *zdfThread;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle &command, yarp::os::Bottle &reply);

    double getPeriod();

    bool updateModule();
};

#endif //ZERODISPARITYFILTER_ZERODISPARITYMODULE_H
//empty line to make gcc happy
