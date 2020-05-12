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

#ifndef _STATE_H_
#define _STATE_H_
#include <string>
#include <map>
#include <yarp/os/all.h>
#include "iCub/helperFunctions.h"
using namespace yarp::os;
using namespace std;



enum class ATTENTION_MODES{RANDOM,INTENSITY,MOTION,CHROMINANCE,ORIENTATION,EDGES,BLOB,FACE,BIO_MOTION,AUDIO};

extern map<ATTENTION_MODES,int> modesIndMap;
extern map<std::string ,std::string> statesCoefMap;


class state{
private:
     map<string,double> coefValueMap{{"k1",0},
                                    {"k2",0},
                                    {"k3",0},
                                    {"k4",0},
                                    {"k5",0},
                                    {"k6",0},
                                    {"kc1",0},
                                    {"kc2",0},
                                    {"kc3",0}};

    void setCoefValByCoef(string coefName, double val);
public:
    string stateName;
    state(string stateName = "NO_NAME");
    bool setCoefValByState(string mapName, double val);
    double gerCoefVal(string mapName) const;
    string getName() const;
    Bottle* getSettings() const;
    int getSettingsSize() const;
};
#endif
