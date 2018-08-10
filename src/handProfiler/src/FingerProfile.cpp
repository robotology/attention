// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
  * Copyright (C)2015  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file MotionProfile.cpp
 * @brief Implementation of the MotionProfile class (see MotionProfile.h).
 */

#include <iCub/FingerProfile.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;
using namespace fingerFactory;

//**************************************************************************************************************

FingerProfile::FingerProfile()  {


    //////////Fabio open port for data gathering
    //string velName("");
    //velName.append(getName("/vel:o"));
    if(!dataPort.open("/FingerProfile/data:o")) {
          yError("dataPort is not open with success. Check for conflicts");
    }

    graspHome.resize(9);
    graspFinal.resize(9);
    graspHome[0] = 59.0; graspHome[1] = 20.0; graspHome[2] = 20.0; graspHome[3] = 20.0; graspHome[4] = 10.0; graspHome[5] = 10.0; graspHome[6] = 10.0; graspHome[7] = 10.0; graspHome[8] = 10.0;
    graspFinal[0] = 45.0; graspFinal[1] = 50.0; graspFinal[2] = 21.0; graspFinal[3] = 50.0; graspFinal[4] = 50.0; graspFinal[5] = 50.0; graspFinal[6] = 50.0; graspFinal[7] = 50.0; graspFinal[8] = 125.0;
}

FingerProfile::~FingerProfile() {

}

Vector* FingerProfile::compute(Vector target) {
    Vector* nextPosition;
    (*nextPosition).resize(9);
    if(target[1]<graspFinal[1]){
        for(int i = 7; i<16; i++){
            (*nextPosition)[i-7] = target[i]+((graspFinal[i]-graspHome[i])/200);
        }
        yInfo("Grasping...");
    }
    return nextPosition;
}
