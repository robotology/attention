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

#ifndef _HELPER_FUNCTIONS_H_
#define _HELPER_FUNCTIONS_H_



#include <stdio.h>
#include <yarp/os/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Vocab.h>
using namespace yarp::os;

const int32_t VOCAB_SET = yarp::os::createVocab('s','e','t');
const int32_t COMMAND_VOCAB_SET  = yarp::os::createVocab('s','e','t');
const int32_t COMMAND_VOCAB_GET  = yarp::os::createVocab('g','e','t');
const int32_t COMMAND_VOCAB_KBU  = yarp::os::createVocab('k','b','u'); //weight of the bottom-up algorithm
const int32_t COMMAND_VOCAB_KTD  = yarp::os::createVocab('k','t','d'); //weight of top-down algorithm
const int32_t COMMAND_VOCAB_RIN  = yarp::os::createVocab('r','i','n'); //red intensity value
const int32_t COMMAND_VOCAB_GIN  = yarp::os::createVocab('g','i','n'); //green intensity value
const int32_t COMMAND_VOCAB_BIN  = yarp::os::createVocab('b','i','n'); //blue intensity value
const int32_t COMMAND_VOCAB_K1  = yarp::os::createVocab('k','1');
const int32_t COMMAND_VOCAB_K2  = yarp::os::createVocab('k','2');
const int32_t COMMAND_VOCAB_K3  = yarp::os::createVocab('k','3');
const int32_t COMMAND_VOCAB_K4  = yarp::os::createVocab('k','4');
const int32_t COMMAND_VOCAB_K5  = yarp::os::createVocab('k','5');
const int32_t COMMAND_VOCAB_K6  = yarp::os::createVocab('k','6');
const int32_t COMMAND_VOCAB_KC1  = yarp::os::createVocab('k','c','1');
const int32_t COMMAND_VOCAB_KC2  = yarp::os::createVocab('k','c','2');
const int32_t COMMAND_VOCAB_KC3  = yarp::os::createVocab('k','c','3');
const int32_t COMMAND_VOCAB_KC4  = yarp::os::createVocab('k','c','4');
const int32_t COMMAND_VOCAB_KC5  = yarp::os::createVocab('k','c','5');
const int32_t COMMAND_VOCAB_KC6  = yarp::os::createVocab('k','c','6');
const int32_t COMMAND_VOCAB_KMOT  = yarp::os::createVocab('k','m','o','t');

class helperFunctions{
public:
    static void printBottle(Bottle& bottle);
    static Bottle createSetVocabBottle(NetInt32 vocab ,double value);
};

#endif
