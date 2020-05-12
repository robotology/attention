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
#include <iostream>
#include <yarp/os/all.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Vocab.h>

#include <iCub/attention/commandDictionary.h>
using namespace yarp::os;



class helperFunctions{
public:
    static void printBottle(Bottle& bottle);
    static Bottle createSetVocabBottle(NetInt32 vocab ,double value);
};

#endif
