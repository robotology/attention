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
    O.resize(3);
    AO.resize(3);
    BO.resize(3);
    xd = new Vector(3);
}

MotionProfile::~MotionProfile() {
    // do nothing
    delete xd;
}

void MotionProfile::setCenter(Vector _O){
    O = _O;
}

void MotionProfile::setStartStop(const double tA, const double tB, const double tC){  
    thetaA = tA; 
    thetaB = tB;
    thetaC = tC;     
}

void MotionProfile::setViaPoints(const Vector _A, const Vector _B, const Vector _C){  
    A = _A; 
    B = _B;
    C = _C;     

    //       x            y           z
    O[0] = -0.3; O[1] = -0.1; O[2] = 0.1;

    AO[0] = A[0] - O[0];
    AO[1] = A[1] - O[1];
    AO[2] = A[2] - O[2];
    AO[0] = AO[0];          // x
    AO[1] = AO[1] / xAxis;  // y
    AO[2] = AO[2] / xAxis;  // z    

    BO[0] = B[0] - O[0];
    BO[1] = B[1] - O[1];
    BO[2] = B[2] - O[2]; 
    BO[0] = BO[0];          // x
    BO[1] = BO[1] / yAxis;  // y
    BO[2] = BO[2] / yAxis;  // z
    
    Vector N(3);    
    N = cross(AO, BO);
}

void MotionProfile::setAxes(const double _xAxis,const double _yAxis) {
    xAxis = _xAxis;
    yAxis = _yAxis;    
}

double MotionProfile::computeRadius(const double theta) {
    double cos2theta = cos(theta) * cos(theta);
    double sin2theta = sin(theta) * sin(theta);    
    double aSquare = xAxis * xAxis;
    double bSquare = yAxis * yAxis;
    double val = 1 / (cos2theta/aSquare + sin2theta/bSquare);
    return sqrt(val);
}

double MotionProfile::computeAngVelocity(const double theta) {
    double revA2B2  = 1/(xAxis * xAxis) - 1/(yAxis * yAxis);
    yDebug("revA2B2: %f", revA2B2);
    double r2       = radius * radius;
    double r3       = radius * radius * radius;
    yDebug("radius %f", radius);
    double drdtheta = (r3 / 2) * revA2B2 * sin(theta);
    yDebug("drdtheta: %f", drdtheta);
    double v2       = tanVelocity * tanVelocity;
    double den      = drdtheta * drdtheta + r2;
    yDebug("denom:  %f", den);
    return            sqrt(v2 / den); 
}

/*
Vector* MotionProfile::compute(double t, double t0) {
    double velocity  = 0.1; //expressed in frequency [Hz], 1/10 of 2PI per second.         
    double theta     = 2.0 * M_PI * 0.1 * (t - t0);
    //yInfo("theta angle [deg] %f", theta * 180 / M_PI);
    
    Vector xdes = *xd;
    //double minAxis = 0.1;
    //double majAxis = 0.1;   

    //xd[0]=-0.3;
    //xd[1]=-0.1+0.1*cos(theta);
    //xd[2]=+0.1+0.1*sin(theta); 

    

    if(theta == 0) {
        yInfo("theta=0 check");
        (*xd)[0]=O[0] + majAxis * cos(theta) * AO[0] + minAxis * sin(theta) * BO[0];
        (*xd)[1]=O[1] + majAxis * cos(theta) * AO[1] + minAxis * sin(theta) * BO[1];
        (*xd)[2]=O[2] + majAxis * cos(theta) * AO[2] + minAxis * sin(theta) * BO[2]; 
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){
            
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            Time::delay(5.0);
        }
    }
    else if ((theta >= thetaA) && (theta<=thetaC)) {
        yDebug("In the range");
        (*xd)[0]=O[0] + majAxis * cos(theta) * AO[0] + minAxis * sin(theta) * BO[0];
        (*xd)[1]=O[1] + majAxis * cos(theta) * AO[1] + minAxis * sin(theta) * BO[1];
        (*xd)[2]=O[2] + majAxis * cos(theta) * AO[2] + minAxis * sin(theta) * BO[2]; 
    
        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        //yInfo("Movementcompleted");
        return NULL;
    }
    
    return xd;    
    
}
*/

//***********************************************************************************************************************

CVMotionProfile::CVMotionProfile(){
    type = "CVP";
    valid = true;
    A.resize(3);
    B.resize(3);
    C.resize(3); 
    O.resize(3);       
    AO.resize(3);
    BO.resize(3);
    xd = new Vector(3);
}

CVMotionProfile::CVMotionProfile(const CVMotionProfile &cvmp){
    valid = cvmp.valid;
    type  = cvmp.type;
    A.resize(3);
    B.resize(3);
    C.resize(3);        
    O.resize(3);
    AO.resize(3);
    BO.resize(3);    
    xd = new Vector(3);
}

CVMotionProfile::~CVMotionProfile(){
    delete xd;
}
    
CVMotionProfile::CVMotionProfile(const Bottle& bInit) {
    valid = false;    
    Bottle* b = bInit.get(0).asList();
    if(b->size() < 6){
        //extracing the features from the bottle
        //((xa,ya,za) (xb,yb,zb) (xc,yc,zc) (0,0.7853,1.5707) (0.1))
        Bottle* aVector = b->get(0).asList();
        yDebug("bottleA:%s", aVector->toString().c_str());    
        Vector aVec(3);
        aVec[0] = aVector->get(0).asDouble();
        aVec[1] = aVector->get(1).asDouble();
        aVec[2] = aVector->get(2).asDouble();

        Bottle* bVector = b->get(1).asList();
        yDebug("bottleB:%s", bVector->toString().c_str());    
        Vector bVec(3);
        bVec[0] = bVector->get(0).asDouble();
        bVec[1] = bVector->get(1).asDouble();
        bVec[2] = bVector->get(2).asDouble();

        Bottle* cVector = b->get(2).asList();
        yDebug("bottleC:%s", cVector->toString().c_str());       
        Vector cVec(3);
        cVec[0] = cVector->get(0).asDouble();
        cVec[1] = cVector->get(1).asDouble();
        cVec[2] = cVector->get(2).asDouble();
        
        Bottle* angles  = b->get(3).asList();
        yDebug("angles:%s", angles->toString().c_str());
        setStartStop(angles->get(0).asDouble(), angles->get(1).asDouble(), angles->get(2).asDouble());  
    
        Bottle* params  = b->get(4).asList();
        yDebug("params:%s", params->toString().c_str());        
        if(params->size()!=3){
            return;
        }    
        setAxes(params->get(0).asDouble(), params->get(1).asDouble());
        setVelocity(params->get(2).asDouble());  
        setViaPoints(aVec, bVec, cVec);
       
        valid = true;
    }   
}

bool CVMotionProfile::operator==(const CVMotionProfile &cvmp)
{
    return ((valid==cvmp.valid)&&(type==cvmp.type));
}

Vector* CVMotionProfile::compute(double t, double t0) {
    //double velocity  = 0.1; 

    if(t-t0 == 0) {
        theta = 0;
    }
    else {
        theta =  thetaPrev + angVelocity * (t - tprev);
    }
    thetaPrev = theta;    
    tprev     = t;
    
    //angular velocity expressed in frequency [Hz],
    //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
    //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec         
    //ONLY FOR CIRCLES: angular velocity constant to obtain constant tang.velocity 
    //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
    radius = computeRadius(theta);
    yInfo("computed radius %f", radius);
    angVelocity = computeAngVelocity(theta);
    yInfo("computed angular velocity %f", angVelocity);
    //angVelocity = 0.1;
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    yInfo("theta angle [rad] %f in %f", theta, t-t0);
    
    Vector xdes = *xd;
    //double minAxis = 0.1;
    //double majAxis = 0.1;   

    //xd[0]=-0.3;
    //xd[1]=-0.1+0.1*cos(theta);
    //xd[2]=+0.1+0.1*sin(theta); 

    if(theta == 0) {
        yInfo("theta=0 check");
        (*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        (*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        (*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2]; 
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){
            
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            Time::delay(5.0);
        }
    }
    else if ((theta >= thetaA) && (theta<=thetaC)) {
        yDebug("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        (*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        (*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        (*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2]; 
    
        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return NULL;
    }
    
    return xd;    
    
}

//***********************************************************************************************************************

TTPLMotionProfile::TTPLMotionProfile(){
    type = "TTPL";
    valid = true;
    A.resize(3);
    B.resize(3);
    C.resize(3); 
    O.resize(3);       
    AO.resize(3);
    BO.resize(3);
    xd = new Vector(3);
}

TTPLMotionProfile::TTPLMotionProfile(const TTPLMotionProfile &ttplmp){
    valid = ttplmp.valid;
    type  = ttplmp.type;
    A.resize(3);
    B.resize(3);
    C.resize(3);        
    O.resize(3);
    AO.resize(3);
    BO.resize(3);    
    xd = new Vector(3);
}

TTPLMotionProfile::~TTPLMotionProfile(){
    delete xd;
}    

TTPLMotionProfile::TTPLMotionProfile(const Bottle& bInit) {
    valid = false;    
    Bottle* b = bInit.get(0).asList();
    if(b->size() < 6){
        //extracing the features from the bottle
        //((xa,ya,za) (xb,yb,zb) (xc,yc,zc) (0,0.7853,1.5707) (0.1))
        Bottle* aVector = b->get(0).asList();
        yDebug("bottleA:%s", aVector->toString().c_str());    
        Vector aVec(3);
        aVec[0] = aVector->get(0).asDouble();
        aVec[1] = aVector->get(1).asDouble();
        aVec[2] = aVector->get(2).asDouble();

        Bottle* bVector = b->get(1).asList();
        yDebug("bottleB:%s", bVector->toString().c_str());    
        Vector bVec(3);
        bVec[0] = bVector->get(0).asDouble();
        bVec[1] = bVector->get(1).asDouble();
        bVec[2] = bVector->get(2).asDouble();

        Bottle* cVector = b->get(2).asList();
        yDebug("bottleC:%s", cVector->toString().c_str());       
        Vector cVec(3);
        cVec[0] = cVector->get(0).asDouble();
        cVec[1] = cVector->get(1).asDouble();
        cVec[2] = cVector->get(2).asDouble();

        setViaPoints(aVec, bVec, cVec);

        Bottle* angles  = b->get(3).asList();
        yDebug("angles:%s", angles->toString().c_str());
        setStartStop(angles->get(0).asDouble(), angles->get(1).asDouble(), angles->get(2).asDouble());  
    
        Bottle* params  = b->get(4).asList();
        yDebug("params:%s", params->toString().c_str());        
        if(params->size() != 4){
            return;
        }    
        setAxes(params->get(0).asDouble(), params->get(1).asDouble());
        setGain(params->get(2).asDouble()); 
        setBeta(params->get(3).asDouble());
       
        valid = true;
    }   
}

bool TTPLMotionProfile::operator==(const TTPLMotionProfile &ttplmp)
{
    return ((valid==ttplmp.valid)&&(type==ttplmp.type));
}

double TTPLMotionProfile::computeTangVelocity() {
    double reBeta = -1 * beta;
    double curvature = 1 / radius;
    double vel = gain * pow(curvature, reBeta);
    return vel;
}


Vector* TTPLMotionProfile::compute(double t, double t0) {
    //double velocity  = 0.1; 
    double theta;

    if(t-t0 == 0) {
        theta = 0;
    }
    else {
        theta =  thetaPrev + angVelocity * (t - t0);
    }

    //velocity expressed in frequency [Hz],
    //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
    //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec         
    //double angVelocity = 0.1;
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    radius          = computeRadius(theta);
    tanVelocity     = computeTangVelocity();
    angVelocity     = computeAngVelocity(theta);
    yInfo("theta angle [rad] %f in %f", theta, t-t0);
    
    Vector xdes = *xd;
    //double xAxis = 0.1;
    //double yAxis = 0.1;   

    //xd[0]=-0.3;
    //xd[1]=-0.1+0.1*cos(theta);
    //xd[2]=+0.1+0.1*sin(theta); 

    if(theta == 0) {
        yInfo("theta = 0 check");
        (*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        (*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        (*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2]; 
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){
            
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            Time::delay(5.0);
        }
    }
    else if ((theta >= thetaA) && (theta<=thetaC)) {
        yDebug("In the range majAxis:%d minAxis:%d");
        (*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        (*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        (*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2]; 
    
        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return NULL;
    }
    
    return xd;    
    
}


