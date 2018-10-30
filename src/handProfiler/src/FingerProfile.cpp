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
    graspHome[0] = 40.0;
    graspHome[1] = 40.0;
    graspHome[2] = 0.0;
    graspHome[3] = 0.0;
    graspHome[4] = 0.0;
    graspHome[5] = 0.0;
    graspHome[6] = 0.0;
    graspHome[7] = 0.0;
    graspHome[8] = 0.0;
    graspFinal[0] = 41.0;
    graspFinal[1] = 50.0;
    graspFinal[2] = 20.0;
    graspFinal[3] = 50.0;
    graspFinal[4] = 50.0;
    graspFinal[5] = 50.0;
    graspFinal[6] = 50.0;
    graspFinal[7] = 50.0;
    graspFinal[8] = 125.0;
    
}

FingerProfile::~FingerProfile() {

}


//*********************************************************************************************

CVFingerProfile::CVFingerProfile(){
    
}

CVFingerProfile::~CVFingerProfile(){

}

Vector* CVFingerProfile::compute(Vector target) {
  
    nextPosition = new Vector(9);

    if(target[15]<graspFinal[8]){
        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = target[i+7]+((graspFinal[i]-graspHome[i])/100);
            //yWarning("current %f       amount %f        final %f", target[i+7], ((graspFinal[i]-graspHome[i])/200), (*nextPosition)[i]);
        }
    //yDebug("%f %f %f %f %f %f %f %f %f",(*nextPosition)[0],(*nextPosition)[1],(*nextPosition)[2],(*nextPosition)[3],(*nextPosition)[4],(*nextPosition)[5],(*nextPosition)[6],(*nextPosition)[7],(*nextPosition)[8]);
    }
    yDebug("inside fp");
    return nextPosition;
}


//*********************************************************************************************

CVVFingerProfile::CVVFingerProfile(){
    nextPosition = new Vector(9);
}

CVVFingerProfile::~CVVFingerProfile(){
    delete nextPosition;
}

Vector* CVVFingerProfile::compute(Vector target, double t) {
    nextPosition->clear(); // check if this is necessary
    double speed2Via   = 100;
    double speed2Final = 100;

    if (t < graspHomeTime) {
        // before the beginning of the grasping
        //for(int i = 0; i<9; i++){
        //    (*nextPosition)[i] = target[i+7]+graspFinal[i];
        //}
    }
    else if ((graspHomeTime < t) && ( t < graspViaTime)) {
        // after the beginning and before the viaPoint
        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = target[i+7]+((graspVia[i]-graspHome[i])/speed2Via);
        }
    }
    else if ((graspViaTime < t)  && ( t < graspFinalTime)) {
        // after the viaPoint and before the end of the grasping
        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = target[i+7]+((graspFinal[i]-graspVia[i])/speed2Final);
        }
    }
    else{
        // after the end of the grasping
        // target[15] == graspFinal[8]
    }
    
    return nextPosition;
}
