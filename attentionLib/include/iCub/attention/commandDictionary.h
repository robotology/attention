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

#define COMMAND_VOCAB_WEIGHT             VOCAB1('w')

#define COMMAND_VOCAB_ON                 VOCAB2('o','n')
#define COMMAND_VOCAB_IS                 VOCAB2('i','s')
#define COMMAND_VOCAB_OK                 VOCAB2('o','k')
#define COMMAND_VOCAB_P0                 VOCAB2('p','0')

#define COMMAND_VOCAB_RED                VOCAB3('r','e','d')
#define COMMAND_VOCAB_SET                VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET                VOCAB3('g','e','t')
#define COMMAND_VOCAB_RUN                VOCAB3('r','u','n')
#define COMMAND_VOCAB_SUSPEND            VOCAB3('s','u','s')
#define COMMAND_VOCAB_RESUME             VOCAB3('r','e','s')
#define COMMAND_VOCAB_FIX                VOCAB3('f','i','x')
#define COMMAND_VOCAB_ADD                VOCAB3('a','d','d')
#define COMMAND_VOCAB_HOR                VOCAB3('h','o','r')
#define COMMAND_VOCAB_VER                VOCAB3('v','e','r')
#define COMMAND_VOCAB_45                 VOCAB3('o','4','5')
#define COMMAND_VOCAB_P45                VOCAB3('p','4','5')
#define COMMAND_VOCAB_N45                VOCAB3('n','4','5')
#define COMMAND_VOCAB_M45                VOCAB3('M','4','5')
#define COMMAND_VOCAB_P90                VOCAB3('p','9','0')
#define COMMAND_VOCAB_ORI                VOCAB3('o','r','i')
#define COMMAND_VOCAB_VIS                VOCAB3('v','i','s')
#define COMMAND_VOCAB_OFF                VOCAB3('o','f','f')

#define COMMAND_VOCAB_FAILED             VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_SEEK               VOCAB4('s','e','e','k')
#define COMMAND_VOCAB_CENT               VOCAB4('c','e','n','t')
#define COMMAND_VOCAB_STOP               VOCAB4('s','t','o','p')
#define COMMAND_VOCAB_HELP               VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_QUIT               VOCAB4('q','u','i','t')
#define COMMAND_VOCAB_FAILED             VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_TEST               VOCAB4('t','e','s','t')


}

}
#endif


