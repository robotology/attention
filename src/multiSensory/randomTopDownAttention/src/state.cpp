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

#include "iCub/state.h"
using namespace attention::dictionary;

map<ATTENTION_MODES,int> modesIndMap{{ATTENTION_MODES::RANDOM,-1},
                                            {ATTENTION_MODES::INTENSITY,0},
                                            {ATTENTION_MODES::MOTION,1},
                                            {ATTENTION_MODES::CHROMINANCE,2},
                                            {ATTENTION_MODES::ORIENTATION,3},
                                            {ATTENTION_MODES::EDGES,4},
                                            {ATTENTION_MODES::BLOB,5},
                                            {ATTENTION_MODES::FACE,6},
                                            {ATTENTION_MODES::BIO_MOTION,7},
                                            {ATTENTION_MODES::AUDIO,8}};

map<std::string ,std::string> statesCoefMap = {{"INTENSITY","k1"},
                                                      {"MOTION","k2"},
                                                      {"CHROMINANCE","k3"},
                                                      {"ORIENTATION","k4"},
                                                      {"EDGES","k5"},
                                                      {"BLOB","k6"},
                                                      {"FACE","kc1"},
                                                      {"BIO_MOTION","kc2"},
                                                      {"AUDIO","kc3"}};

state::state(string stateName){
    this->stateName = stateName;

    map<string,string>::iterator statesCoefMapIterator;
    statesCoefMapIterator = statesCoefMap.find(stateName);
    if(statesCoefMapIterator != statesCoefMap.end())
        setCoefValByCoef(statesCoefMapIterator->second,1);
}
string state::getName()const{
    return stateName;
}
bool state::setCoefValByState(string mapName, double val){
    map<string,string>::iterator statesCoefMapIterator;
    statesCoefMapIterator = statesCoefMap.find(mapName);
    if(statesCoefMapIterator == statesCoefMap.end())
        return false;
    coefValueMap.at(statesCoefMapIterator->second) = val;
    return true;
}
void state::setCoefValByCoef(string coefName, double val){
    coefValueMap.at(coefName) = val;
}

Bottle* state::getSettings() const{
    Bottle* settings = new Bottle[9];
    settings[0] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K1,coefValueMap.at("k1"));
    settings[1] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K2,coefValueMap.at("k2"));
    settings[2] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K3,coefValueMap.at("k3"));
    settings[3] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K4,coefValueMap.at("k4"));
    settings[4] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K5,coefValueMap.at("k5"));
    settings[5] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_K6,coefValueMap.at("k6"));
    settings[6] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_KC1,coefValueMap.at("kc1"));
    settings[7] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_KC2,coefValueMap.at("kc2"));
    settings[8] = helperFunctions::createSetVocabBottle(COMMAND_VOCAB_KC3,coefValueMap.at("kc3"));
    return settings;
}

int state::getSettingsSize() const{
    return 9;
}
