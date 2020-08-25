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
    //string velName("");`
    //velName.append(getName("/vel:o"));
    if(!dataPort.open("/FingerProfile/data:o")) {
          yError("dataPort is not open with success. Check for conflicts");
    }
    
    graspHome.resize(9);
    graspVia.resize(9);
    graspFinal.resize(9);
    /*graspHome[0] = 40.0;
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
    graspFinal[8] = 125.0;*/

    //small
    graspHome[0] = 40.0;
    graspHome[1] = 40.0;
    graspHome[2] = 0.0;
    graspHome[3] = 0.0;
    graspHome[4] = 0.0;
    graspHome[5] = 0.0;
    graspHome[6] = 0.0;
    graspHome[7] = 0.0;
    graspHome[8] = 0.0;
    graspVia[0] = 40.0;
    graspVia[1] = 50.0;
    graspVia[2] = 10.0;
    graspVia[3] = 20.0;
    graspVia[4] = 40.0;
    graspVia[5] = 7.5;
    graspVia[6] = 55.0;
    graspVia[7] = 165.0;
    graspVia[8] = 232.0;
    graspFinal[0] = 41.0;
    graspFinal[1] = 60.0;
    graspFinal[2] = 20.0;
    graspFinal[3] = 30.0;
    graspFinal[4] = 80.0;
    graspFinal[5] = 15.0;
    graspFinal[6] = 55.0;
    graspFinal[7] = 165.0;
    graspFinal[8] = 232.0;
    
    //BIG
    /*graspHome[0] = 40.0;
    graspHome[1] = 40.0;
    graspHome[2] = 0.0;
    graspHome[3] = 0.0;
    graspHome[4] = 0.0;
    graspHome[5] = 0.0;
    graspHome[6] = 0.0;
    graspHome[7] = 0.0;
    graspHome[8] = 0.0;
    graspVia[0] = 40.0;
    graspVia[1] = 45.0;
    graspVia[2] = 5.0;
    graspVia[3] = 5.0;
    graspVia[4] = 10.0;
    graspVia[5] = 5.0;
    graspVia[6] = 10.0;
    graspVia[7] = 5.0;
    graspVia[8] = 50.0;
    graspFinal[0] = 41.0;
    graspFinal[1] = 50.0;
    graspFinal[2] = 10.0;
    graspFinal[3] = 10.0;
    graspFinal[4] = 20.0;
    graspFinal[5] = 10.0;
    graspFinal[6] = 20.0;
    graspFinal[7] = 10.0;
    graspFinal[8] = 100.0;*/
    
}

FingerProfile::~FingerProfile() {

}


//*********************************************************************************************

CVFingerProfile::CVFingerProfile(){
    
}

CVFingerProfile::~CVFingerProfile(){

}

Vector* CVFingerProfile::compute(Vector target, double t) {
  
    nextPosition = new Vector(9);

    if(target[15]<graspFinal[8]){
        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = target[i+7]+((graspFinal[i]-graspHome[i])/300);
            //yWarning("current %f       amount %f        final %f", target[i+7], ((graspFinal[i]-graspHome[i])/200), (*nextPosition)[i]);
        }
    //yDebug("%f %f %f %f %f %f %f %f %f",(*nextPosition)[0],(*nextPosition)[1],(*nextPosition)[2],(*nextPosition)[3],(*nextPosition)[4],(*nextPosition)[5],(*nextPosition)[6],(*nextPosition)[7],(*nextPosition)[8]);
    }
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
    
    nextPosition = new Vector(9);
    //nextPosition->clear(); // check if this is necessary
    double speed2Via   = 300;
    double speed2Final = 300;
    graspHomeTime = 0.01; 
    graspViaTime = 0.3;
    graspFinalTime = 0.6;

    yWarning("tempo %f", t);

    if (t < graspHomeTime) {
        // before the beginning of the grasping
        yDebug("before for %f ", t);
        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = target[i+7];
        }
        //yDebug("fase 1 %f ", t);
    }
    else if ((graspHomeTime < t) && ( t < graspViaTime)) {
        // after the beginning and before the viaPoint
        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = target[i+7]+((graspVia[i]-graspHome[i])/speed2Via);
        }
        //yWarning("fase 2 %f ", t);
    }
    else if ((graspViaTime < t)  && ( t < graspFinalTime)) {
        // after the viaPoint and before the end of the grasping
        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = target[i+7]+((graspFinal[i]-graspVia[i])/speed2Final);
        }
        //yInfo("fase 3 %f ", t);
    }
    else{
        // after the end of the grasping
        // target[15] == graspFinal[8]

        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = graspFinal[i];
        }
        //yError("fase 4 %f ", t);
    }
    
    return nextPosition;
}
