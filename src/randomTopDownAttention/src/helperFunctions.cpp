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

#include "iCub/helperFunctions.h"


void helperFunctions::printBottle(Bottle& bottle){
    //this function prints in details the bottle object (msg from to ports)
    // it is used to make sure that the msg is created proparly without any errors
    for (int i=0; i<bottle.size(); i++) {
        yInfo("[%d]: ", i);
        Value& element = bottle.get(i);
        switch (element.getCode()) {
        case BOTTLE_TAG_INT32:
            yInfo("int %d\n", element.asInt32());
            break;
        case BOTTLE_TAG_FLOAT64:
            yInfo("float %g\n", element.asFloat64());
            break;
        case BOTTLE_TAG_STRING:
            yInfo("string \"%s\"\n", element.asString().c_str());
            break;
        case BOTTLE_TAG_BLOB:
            yInfo("binary blob of length %zd\n", element.asBlobLength());
            break;
        case BOTTLE_TAG_VOCAB:
            yInfo("vocab [%s]\n", Vocab::decode(element.asVocab()).c_str());
            break;
        default:
            if (element.isList()) {
                Bottle *lst = element.asList();
                printf("list of %zu elements\n", lst->size());
                printBottle(*lst);
            } else {
                printf("unrecognized type\n");
            }
            break;
        }
    }
}


Bottle helperFunctions::createSetVocabBottle(NetInt32 vocab ,double value){
    //this function creates a set commad vocab command
    Bottle cmd;
    cmd.addVocab(COMMAND_VOCAB_SET);
    cmd.addVocab(vocab);
    cmd.addDouble(value);
    return cmd;
}
