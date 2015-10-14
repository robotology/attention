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
using namespace profileFactory;

//namespace profileFactory{

//}
//**************************************************************************************************************

MotionProfile::MotionProfile() : valid(false), type("")  {
    A.resize(3);
    B.resize(3);
    C.resize(3);
    //AO = new Vector(3);        
    //BO = new Vector(3);
    AO.resize(3);
    BO.resize(3);
    O.resize(3);
}

MotionProfile::~MotionProfile() {
    // do nothing
    //delete AO;
    //delete BO;
}

void MotionProfile::setCenter(Vector _O){
    O = _O;
}

void MotionProfile::setViaPoints(const Vector _A, const Vector _B, const Vector _C){  
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
    
    yDebug("A0 %08X", &AO);
    yDebug("B0 %08X", &BO);
    
    //Vector N(3);    
    //N = cross(AO, BO);
}

void MotionProfile::setAxes(double _majAxis, double _minAxis) {
    minAxis = _minAxis;
    majAxis = _majAxis;    
}

Vector MotionProfile::compute(double t, double t0) {
    yDebug("motionProfile::compute");
    double velocity = 0.1; //expressed in frequency [Hz], 1/10 of 2PI per second.    
    double theta =2.0 * M_PI * 0.1 * (t - t0);
    
    Vector xd(3);
    double minAxis = 0.1;
    double majAxis = 0.1;

    yDebug("Success!");
    
    yDebug("A0 %s", AO.toString().c_str());
    yDebug("B0 %s", BO.toString().c_str());   

    //xd[0]=-0.3;
    //xd[1]=-0.1+0.1*cos(theta);
    //xd[2]=+0.1+0.1*sin(theta); 

    xd[0]=-0.3;//+ majAxis*cos(theta) * BO[0] + minAxis * sin(theta) * AO[0];
    xd[1]=-0.1 + majAxis*cos(theta) * AO[1] + minAxis * sin(theta) * BO[1];
    xd[2]=+0.1 + majAxis*cos(theta) * AO[2] + minAxis * sin(theta) * BO[2]; 
        
    
    return xd;
}

//***********************************************************************************************************************

CVMotionProfile::CVMotionProfile(){
    type = "CVP";
    valid = true;
    A.resize(3);
    B.resize(3);
    C.resize(3);        
    //AO = new Vector(3);
    //BO = new Vector(3);
    AO.resize(3);
    BO.resize(3);
    O.resize(3);
}

CVMotionProfile::CVMotionProfile(const CVMotionProfile &cvmp){
    valid = cvmp.valid;
    type  = cvmp.type;
    A.resize(3);
    B.resize(3);
    C.resize(3);        
    //AO = new Vector(3);
    //BO = new Vector(3);
    AO.resize(3);
    BO.resize(3);
    O.resize(3);
}
    
CVMotionProfile::CVMotionProfile(const Bottle& bInit) {
    valid = false;    
    Bottle* b = bInit.get(0).asList();
    yDebug("bottle size %d", b->size());
    if(b->size() < 6){
        //extracing the features from the bottle
        //((xa,ya,za) (xb,yb,zb) (xc,yc,zc) (0,0.7853,1.5707) (0.1))
        Bottle* aVector = b->get(0).asList();
        yDebug("bottleA:%s", aVector->toString().c_str());    
        Vector aVec(3);aVec[0] = aVector->get(0).asDouble(); aVec[1] = aVector->get(1).asDouble(); aVec[2] = aVector->get(2).asDouble();
        Bottle* bVector = b->get(1).asList();
        yDebug("bottleB:%s", bVector->toString().c_str());    
        Vector bVec(3);bVec[0] = bVector->get(0).asDouble(); bVec[1] = bVector->get(1).asDouble(); bVec[2] = bVector->get(2).asDouble();
        Bottle* cVector = b->get(2).asList();
        yDebug("bottleC:%s", cVector->toString().c_str());       
        Vector cVec(3);cVec[0] = cVector->get(0).asDouble(); cVec[1] = cVector->get(1).asDouble(); cVec[2] = cVector->get(2).asDouble();
        Bottle* angles  = b->get(3).asList();
        yDebug("angles:%s", angles->toString().c_str());    
        Bottle* params  = b->get(4).asList();
        yDebug("params:%s", params->toString().c_str());    
        setViaPoints(aVec, bVec, cVec);
        valid = true;
    }   
}

bool CVMotionProfile::operator==(const CVMotionProfile &cvmp)
{
    return ((valid==cvmp.valid)&&(type==cvmp.type));
}



