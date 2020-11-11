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

        const int32_t COMMAND_VOCAB_3D                  = yarp::os::createVocab('3','D');
        const int32_t COMMAND_VOCAB_PROJ                = yarp::os::createVocab('p','r','o','j');
        const int32_t COMMAND_VOCAB_ACK                = yarp::os::createVocab('a','c','k');
        const int32_t COMMAND_VOCAB_ON                  = yarp::os::createVocab('o','n');
        const int32_t COMMAND_VOCAB_IS                  = yarp::os::createVocab('i','s');
        const int32_t COMMAND_VOCAB_OK                  = yarp::os::createVocab('o','k');
        const int32_t COMMAND_VOCAB_P0                  = yarp::os::createVocab('p','0');
        const int32_t COMMAND_VOCAB_FB                  = yarp::os::createVocab('f','b');
        const int32_t COMMAND_VOCAB_TV                  = yarp::os::createVocab('t','v');
        const int32_t COMMAND_VOCAB_LK                  = yarp::os::createVocab('l','k');
        const int32_t COMMAND_VOCAB_ERROR               = yarp::os::createVocab('E','R','R');

        const int32_t COMMAND_VOCAB_ABS                = yarp::os::createVocab('A','B','S');
        const int32_t COMMAND_VOCAB_REL                = yarp::os::createVocab('R','E','L');

        const int32_t COMMAND_VOCAB_MAX                 = yarp::os::createVocab('m','a','x');
        const int32_t COMMAND_VOCAB_MEAN                = yarp::os::createVocab('m','e','a','n');
        const int32_t COMMAND_VOCAB_STD                 = yarp::os::createVocab('s','t','d');
        const int32_t COMMAND_VOCAB_3SIGMA              = yarp::os::createVocab('3','s');

        const int32_t COMMAND_VOCAB_MODE               = yarp::os::createVocab('M','O','D','E');
        const int32_t COMMAND_VOCAB_BLOB               = yarp::os::createVocab('B','L','O');
        const int32_t COMMAND_VOCAB_CHROMINANCE        = yarp::os::createVocab('C','H','R');
        const int32_t COMMAND_VOCAB_INTENSITY          = yarp::os::createVocab('I','N','T');
        const int32_t COMMAND_VOCAB_MOTION             = yarp::os::createVocab('M','O','T');
        const int32_t COMMAND_VOCAB_ORIENTATION        = yarp::os::createVocab('O','R','I');
        const int32_t COMMAND_VOCAB_EDGES              = yarp::os::createVocab('E','D','G');
        const int32_t COMMAND_VOCAB_RANDOM             = yarp::os::createVocab('R','A','N');
        const int32_t COMMAND_VOCAB_RED                 = yarp::os::createVocab('r','e','d');
        const int32_t COMMAND_VOCAB_SET                 = yarp::os::createVocab('s','e','t');
        const int32_t COMMAND_VOCAB_RESET               = yarp::os::createVocab('r','e','s');
        const int32_t COMMAND_VOCAB_GET                 = yarp::os::createVocab('g','e','t');
        const int32_t COMMAND_VOCAB_RUN                 = yarp::os::createVocab('r','u','n');
        const int32_t COMMAND_VOCAB_SUSPEND             = yarp::os::createVocab('s','u','s');
        const int32_t COMMAND_VOCAB_RESUME              = yarp::os::createVocab('R','E','S');
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
        const int32_t COMMAND_VOCAB_THRESHOLD           = yarp::os::createVocab('T','H','R');

        const int32_t COMMAND_VOCAB_FACE                = yarp::os::createVocab('F','A','C','E');
        const int32_t COMMAND_VOCAB_BIO_MOTION          = yarp::os::createVocab('B','M','O','T');
        const int32_t COMMAND_VOCAB_AUDIO               = yarp::os::createVocab('A','U','D','I');
        const int32_t COMMAND_VOCAB_FAILED              = yarp::os::createVocab('f','a','i','l');
        const int32_t COMMAND_VOCAB_SEEK                = yarp::os::createVocab('s','e','e','k');
        const int32_t COMMAND_VOCAB_CENT                = yarp::os::createVocab('c','e','n','t');
        const int32_t COMMAND_VOCAB_STOP                = yarp::os::createVocab('s','t','o','p');
        const int32_t COMMAND_VOCAB_HELP                = yarp::os::createVocab('h','e','l','p');
        const int32_t COMMAND_VOCAB_QUIT                = yarp::os::createVocab('q','u','i','t');
        const int32_t COMMAND_VOCAB_TEST                = yarp::os::createVocab('t','e','s','t');
        const int32_t COMMAND_VOCAB_ALGO                = yarp::os::createVocab('a','l','g','o');


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


    }

}
#endif


