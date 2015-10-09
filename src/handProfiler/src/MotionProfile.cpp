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

#include <iCub/MotionProfile.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;

MotionProfile::MotionProfile() {
    A.resize(3);
    B.resize(3);
    C.resize(3);        
    AO.resize(3);
    BO.resize(3);
}

MotionProfile::~MotionProfile() {
    // do nothing
}

void MotionProfile::setCenter(Vector _O){
    O = _O;
}

void MotionProfile::setViaPoints(Vector _A, Vector _B, Vector _C){
    A = _A; 
    B = _B;
    C = _C;

    AO[0] = A[0] - O[0];
    AO[1] = A[1] - O[1];
    AO[2] = A[2] - O[2];
    AO[0] = AO[0] / -0.1;
    AO[1] = AO[1] / -0.1;
    AO[2] = AO[2] / -0.1;    

    BO[0] = B[0] - O[0];
    BO[1] = B[1] - O[1];
    BO[2] = B[2] - O[2]; 
    BO[0] = BO[0] / 0.1;
    BO[1] = BO[1] / 0.1;
    BO[2] = BO[2] / 0.1;
    
    printf("A0 %s \n", AO.toString().c_str());
    printf("B0 %s \n", BO.toString().c_str());
    
    Vector N(3);    
    N = cross(AO, BO);
}

void MotionProfile::setAxes(double _majAxis, double _minAxis) {
    minAxis = _minAxis;
    majAxis = _majAxis;    
}

Vector MotionProfile::compute(double t, double t0) {
    double theta =2.0 * M_PI * 0.1 * (t - t0);
    Vector xd(3);

    xd[0]=-0.3;
    xd[1]=-0.1+0.1*cos(theta);
    xd[2]=+0.1+0.1*sin(theta); 

    //xd[0]=-0.3;//+ majAxis*cos(theta) * BO[0] + minAxis * sin(theta) * AO[0];
    //xd[1]=-0.1 + majAxis*cos(theta) * BO[1] + minAxis * sin(theta) * AO[1];
    //xd[2]=+0.1 + majAxis*cos(theta) * BO[2] + minAxis * sin(theta) * AO[2]; 

    return xd;
}


