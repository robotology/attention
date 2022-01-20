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

        const int32_t COMMAND_VOCAB_WEIGHT              = yarp::os::createVocab32('w');

        const int32_t COMMAND_VOCAB_3D                  = yarp::os::createVocab32('3','D');
        const int32_t COMMAND_VOCAB_PROJ                = yarp::os::createVocab32('p','r','o','j');
        const int32_t COMMAND_VOCAB_ACK                = yarp::os::createVocab32('a','c','k');
        const int32_t COMMAND_VOCAB_ON                  = yarp::os::createVocab32('o','n');
        const int32_t COMMAND_VOCAB_IS                  = yarp::os::createVocab32('i','s');
        const int32_t COMMAND_VOCAB_OK                  = yarp::os::createVocab32('o','k');
        const int32_t COMMAND_VOCAB_P0                  = yarp::os::createVocab32('p','0');
        const int32_t COMMAND_VOCAB_FB                  = yarp::os::createVocab32('f','b');
        const int32_t COMMAND_VOCAB_TV                  = yarp::os::createVocab32('t','v');
        const int32_t COMMAND_VOCAB_LK                  = yarp::os::createVocab32('l','k');
        const int32_t COMMAND_VOCAB_ERROR               = yarp::os::createVocab32('E','R','R');

        const int32_t COMMAND_VOCAB_ABS                = yarp::os::createVocab32('A','B','S');
        const int32_t COMMAND_VOCAB_REL                = yarp::os::createVocab32('R','E','L');
        const int32_t COMMAND_VOCAB_INIT               = yarp::os::createVocab32('I','N','I','T');
        const int32_t COMMAND_VOCAB_CURRENT            = yarp::os::createVocab32('C','U','R');

        const int32_t COMMAND_VOCAB_MAX                 = yarp::os::createVocab32('m','a','x');
        const int32_t COMMAND_VOCAB_MEAN                = yarp::os::createVocab32('m','e','a','n');
        const int32_t COMMAND_VOCAB_STD                 = yarp::os::createVocab32('s','t','d');
        const int32_t COMMAND_VOCAB_3SIGMA              = yarp::os::createVocab32('3','s');

        const int32_t COMMAND_VOCAB_MODE               = yarp::os::createVocab32('M','O','D','E');
        const int32_t COMMAND_VOCAB_BLOB               = yarp::os::createVocab32('B','L','O');
        const int32_t COMMAND_VOCAB_CHROMINANCE        = yarp::os::createVocab32('C','H','R');
        const int32_t COMMAND_VOCAB_INTENSITY          = yarp::os::createVocab32('I','N','T');
        const int32_t COMMAND_VOCAB_MOTION             = yarp::os::createVocab32('M','O','T');
        const int32_t COMMAND_VOCAB_ORIENTATION        = yarp::os::createVocab32('O','R','I');
        const int32_t COMMAND_VOCAB_EDGES              = yarp::os::createVocab32('E','D','G');
        const int32_t COMMAND_VOCAB_RANDOM             = yarp::os::createVocab32('R','A','N');
        const int32_t COMMAND_VOCAB_RED                 = yarp::os::createVocab32('r','e','d');
        const int32_t COMMAND_VOCAB_SET                 = yarp::os::createVocab32('s','e','t');
        const int32_t COMMAND_VOCAB_RESET               = yarp::os::createVocab32('r','s','e','t');
        const int32_t COMMAND_VOCAB_GET                 = yarp::os::createVocab32('g','e','t');
        const int32_t COMMAND_VOCAB_RUN                 = yarp::os::createVocab32('r','u','n');
        const int32_t COMMAND_VOCAB_SUSPEND             = yarp::os::createVocab32('s','u','s');
        const int32_t COMMAND_VOCAB_RESUME              = yarp::os::createVocab32('r','e','s');
        const int32_t COMMAND_VOCAB_FIX                 = yarp::os::createVocab32('f','i','x');
        const int32_t COMMAND_VOCAB_ADD                 = yarp::os::createVocab32('a','d','d');
        const int32_t COMMAND_VOCAB_HOR                 = yarp::os::createVocab32('h','o','r');
        const int32_t COMMAND_VOCAB_VER                 = yarp::os::createVocab32('v','e','r');
        const int32_t COMMAND_VOCAB_45                  = yarp::os::createVocab32('o','4','5');
        const int32_t COMMAND_VOCAB_P45                 = yarp::os::createVocab32('p','4','5');
        const int32_t COMMAND_VOCAB_N45                 = yarp::os::createVocab32('n','4','5');
        const int32_t COMMAND_VOCAB_M45                 = yarp::os::createVocab32('M','4','5');
        const int32_t COMMAND_VOCAB_P90                 = yarp::os::createVocab32('p','9','0');
        const int32_t COMMAND_VOCAB_ORI                 = yarp::os::createVocab32('o','r','i');
        const int32_t COMMAND_VOCAB_VIS                 = yarp::os::createVocab32('v','i','s');
        const int32_t COMMAND_VOCAB_OFF                 = yarp::os::createVocab32('o','f','f');
        const int32_t COMMAND_VOCAB_THRESHOLD           = yarp::os::createVocab32('T','H','R');

        const int32_t COMMAND_VOCAB_FACE                = yarp::os::createVocab32('F','A','C','E');
        const int32_t COMMAND_VOCAB_BIO_MOTION          = yarp::os::createVocab32('B','M','O','T');
        const int32_t COMMAND_VOCAB_AUDIO               = yarp::os::createVocab32('A','U','D','I');
        const int32_t COMMAND_VOCAB_FAILED              = yarp::os::createVocab32('f','a','i','l');
        const int32_t COMMAND_VOCAB_SEEK                = yarp::os::createVocab32('s','e','e','k');
        const int32_t COMMAND_VOCAB_CENT                = yarp::os::createVocab32('c','e','n','t');
        const int32_t COMMAND_VOCAB_STOP                = yarp::os::createVocab32('s','t','o','p');
        const int32_t COMMAND_VOCAB_HELP                = yarp::os::createVocab32('h','e','l','p');
        const int32_t COMMAND_VOCAB_QUIT                = yarp::os::createVocab32('q','u','i','t');
        const int32_t COMMAND_VOCAB_TEST                = yarp::os::createVocab32('t','e','s','t');
        const int32_t COMMAND_VOCAB_ALGO                = yarp::os::createVocab32('a','l','g','o');


        const int32_t COMMAND_VOCAB_KBU  = yarp::os::createVocab32('k','b','u'); //weight of the bottom-up algorithm
        const int32_t COMMAND_VOCAB_KTD  = yarp::os::createVocab32('k','t','d'); //weight of top-down algorithm
        const int32_t COMMAND_VOCAB_RIN  = yarp::os::createVocab32('r','i','n'); //red intensity value
        const int32_t COMMAND_VOCAB_GIN  = yarp::os::createVocab32('g','i','n'); //green intensity value
        const int32_t COMMAND_VOCAB_BIN  = yarp::os::createVocab32('b','i','n'); //blue intensity value
        const int32_t COMMAND_VOCAB_K1  = yarp::os::createVocab32('k','1');
        const int32_t COMMAND_VOCAB_K2  = yarp::os::createVocab32('k','2');
        const int32_t COMMAND_VOCAB_K3  = yarp::os::createVocab32('k','3');
        const int32_t COMMAND_VOCAB_K4  = yarp::os::createVocab32('k','4');
        const int32_t COMMAND_VOCAB_K5  = yarp::os::createVocab32('k','5');
        const int32_t COMMAND_VOCAB_K6  = yarp::os::createVocab32('k','6');
        const int32_t COMMAND_VOCAB_KC1  = yarp::os::createVocab32('k','c','1');
        const int32_t COMMAND_VOCAB_KC2  = yarp::os::createVocab32('k','c','2');
        const int32_t COMMAND_VOCAB_KC3  = yarp::os::createVocab32('k','c','3');
        const int32_t COMMAND_VOCAB_KC4  = yarp::os::createVocab32('k','c','4');
        const int32_t COMMAND_VOCAB_KC5  = yarp::os::createVocab32('k','c','5');
        const int32_t COMMAND_VOCAB_KC6  = yarp::os::createVocab32('k','c','6');
        const int32_t COMMAND_VOCAB_KMOT  = yarp::os::createVocab32('k','m','o','t');


    }

}
#endif


