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

#include "iCub/stateOrientationOnly.h"
using namespace attention::dictionary;
using namespace std;

stateOrientationOnly::stateOrientationOnly():state("stateOrientationOnly",4){}

Bottle* stateOrientationOnly::getSettings(){
    Bottle* settings = new Bottle[6];
    settings[0] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K1,0);
    settings[1] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K2,0);
    settings[2] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K3,0);
    settings[3] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K4,1);
    settings[4] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K5,0);
    settings[5] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K6,0);
    return settings;
}

int stateOrientationOnly::getSettingsSize(){
    return 6;
}
