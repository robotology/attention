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

#include "iCub/zeroDisparityFilterThread.h"


#define COMMAND_VOCAB_IS   VOCAB2('i','s')
#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SET  VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET  VOCAB3('g','e','t')
#define COMMAND_VOCAB_K1   VOCAB2('k','1') //data penalty
#define COMMAND_VOCAB_K2   VOCAB2('k','2') //smoothness penalty base
#define COMMAND_VOCAB_K3   VOCAB2('k','3') //smoothness penalty
#define COMMAND_VOCAB_K4   VOCAB2('k','4') //radial penalty
#define COMMAND_VOCAB_K5   VOCAB2('k','5') //smoothness 3sigmaon2
#define COMMAND_VOCAB_K6   VOCAB2('k','6') //bland dog thresh
#define COMMAND_VOCAB_K7   VOCAB2('k','7') //bland prob
#define COMMAND_VOCAB_K8   VOCAB2('k','8') //max spread



class zeroDisparityFilterMod : public yarp::os::RFModule {
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
