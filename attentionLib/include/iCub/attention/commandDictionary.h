// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Francesco Rea
 * email:  francesco.rea@iit.it
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

#ifndef __COMMAND_DICTIONARY_H__
#define __COMMAND_DICTIONARY_H__

#include <yarp/os/all.h>

namespace attention
{

namespace dictionary
{

const int32_t COMMAND_VOCAB_WEIGHT              = yarp::os::createVocab('w');

const int32_t COMMAND_VOCAB_ON                  = yarp::os::createVocab('o','n');
const int32_t COMMAND_VOCAB_IS                  = yarp::os::createVocab('i','s');
const int32_t COMMAND_VOCAB_OK                  = yarp::os::createVocab('o','k');
const int32_t COMMAND_VOCAB_P0                  = yarp::os::createVocab('p','0');
const int32_t COMMAND_VOCAB_FB                  = yarp::os::createVocab('f','b');
const int32_t COMMAND_VOCAB_TV                  = yarp::os::createVocab('t','v');
const int32_t COMMAND_VOCAB_LK                  = yarp::os::createVocab('l','k');

const int32_t COMMAND_VOCAB_RED                 = yarp::os::createVocab('r','e','d');
const int32_t COMMAND_VOCAB_SET                 = yarp::os::createVocab('s','e','t');
const int32_t COMMAND_VOCAB_GET                 = yarp::os::createVocab('g','e','t');
const int32_t COMMAND_VOCAB_RUN                 = yarp::os::createVocab('r','u','n');
const int32_t COMMAND_VOCAB_SUSPEND             = yarp::os::createVocab('s','u','s');
const int32_t COMMAND_VOCAB_RESUME              = yarp::os::createVocab('r','e','s');
const int32_t COMMAND_VOCAB_FIX                 = yarp::os::createVocab('f','i','x');
const int32_t COMMAND_VOCAB_ADD                 = yarp::os::createVocab('a','d','d');
const int32_t COMMAND_VOCAB_HOR                 = yarp::os::createVocab('h','o','r');
const int32_t COMMAND_VOCAB_VER                 = yarp::os::createVocab('v','e','r');
const int32_t COMMAND_VOCAB_45                  = yarp::os::createVocab('o','4','5');
const int32_t COMMAND_VOCAB_P45                 = yarp::os::createVocab('p','4','5');
const int32_t COMMAND_VOCAB_N45                 = yarp::os::createVocab('n','4','5');
const int32_t COMMAND_VOCAB_M45                 = yarp::os::createVocab('M','4','5');
const int32_t COMMAND_VOCAB_P90                 = yarp::os::createVocab('p','9','0');
const int32_t COMMAND_VOCAB_ORI                 = yarp::os::createVocab('o','r','i');
const int32_t COMMAND_VOCAB_VIS                 = yarp::os::createVocab('v','i','s');
const int32_t COMMAND_VOCAB_OFF                 = yarp::os::createVocab('o','f','f');

const int32_t COMMAND_VOCAB_FAILED              = yarp::os::createVocab('f','a','i','l');
const int32_t COMMAND_VOCAB_SEEK                = yarp::os::createVocab('s','e','e','k');
const int32_t COMMAND_VOCAB_CENT                = yarp::os::createVocab('c','e','n','t');
const int32_t COMMAND_VOCAB_STOP                = yarp::os::createVocab('s','t','o','p');
const int32_t COMMAND_VOCAB_HELP                = yarp::os::createVocab('h','e','l','p');
const int32_t COMMAND_VOCAB_QUIT                = yarp::os::createVocab('q','u','i','t');
const int32_t COMMAND_VOCAB_TEST                = yarp::os::createVocab('t','e','s','t');
const int32_t COMMAND_VOCAB_ALGO                = yarp::os::createVocab('a','l','g','o');


}

}
#endif


