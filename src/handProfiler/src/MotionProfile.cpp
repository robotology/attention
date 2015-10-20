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
    xPrev.resize(3);
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
    yInfo("MotionProfile::setViaPoints");
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

    double revA2   = 1/(xAxis * xAxis);
    double revB2   = 1/(yAxis * yAxis);
    subA2B2 = revA2 - revB2;
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
    double invR2 = (cos2theta/aSquare + sin2theta/bSquare);
    double R2 = 1 / invR2;
    r = sqrt(R2);
    r2 = r * r;
    r3 = r2 * r;
    return r;
}

double MotionProfile::computeAngVelocity(const double theta) {
    //double revA2    = 1/(xAxis * xAxis);
    //double revB2    = 1/(yAxis * yAxis);
    //double subA2B2  = revA2 - revB2;
    //double r2       = radius * radius;
    //double r3       = radius * radius * radius;
    double drdtheta = (r3 / 2) * subA2B2 * sin(2 * theta);
    double v2       = tanVelocity * tanVelocity;
    double den      = drdtheta * drdtheta + r2;
    return            sqrt(v2 / den); 
}

double MotionProfile::checkTanVelocity(const double theta) {
    //double revA2   = 1/(xAxis * xAxis);
    //double revB2   = 1/(yAxis * yAxis);
    //double subA2B2 = revA2 - revB2;
    //double r2       = radius * radius;
    //double r3       = radius * radius * radius;
    double drdtheta = (r3 / 2) * subA2B2 * sin(2 * theta);
    double w2       = angVelocity * angVelocity;
    double den      = drdtheta * drdtheta + r2;
    return            sqrt(w2 * den); 
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
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;
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
    xPrev.resize(3); 
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;
}

CVMotionProfile::~CVMotionProfile(){
    delete xd;
}
    
CVMotionProfile::CVMotionProfile(const Bottle& bInit) {
    type = "CVP";    
    valid = false;    
    
    A.resize(3);
    B.resize(3);
    C.resize(3); 
    O.resize(3);       
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;   

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
        if(params->size()==3){
            setAxes(params->get(0).asDouble(), params->get(1).asDouble());
            setVelocity(params->get(2).asDouble());  
            setViaPoints(aVec, bVec, cVec);
            valid = true;
        }
    }   
}

bool CVMotionProfile::operator==(const CVMotionProfile &cvmp)
{
    return ((valid==cvmp.valid)&&(type==cvmp.type));
}

void CVMotionProfile::preComputation(const double theta) {
    //angular velocity expressed in frequency [Hz],
    //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
    //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec         
    //ONLY FOR CIRCLES: angular velocity constant to obtain constant tang.velocity 
    //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
    radius = computeRadius(theta);
    /*    
    double rDiff2     = (radius - radiusPrev) * (radius-radiusPrev);
    double thetaDiff2 = (theta  - thetaPrev) * (theta - thetaPrev); 
    double sDiff2     = rDiff2  + (radius * radius) * thetaDiff2;
    yInfo("rDiff2 %f thetaDiff2 %f radius %f sDiff %f", rDiff2, thetaDiff2, radius, sqrt(sDiff2));
    */

    /*    
    double drdtheta   = (radius - radiusPrev) / (theta - thetaPrev);
    double drdtheta2  =  drdtheta * drdtheta;
    double v2         = (drdtheta2 + (radius * radius)) * (angVelocity * angVelocity);
    yInfo("v %f", sqrt(v2));
    */

    yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
    // computing angular velocity in function of the radius, theta, tangential velocity
    angVelocity = computeAngVelocity(theta);
    yInfo("computed angular velocity %f in rad/s", angVelocity);
    double tanVelocity_temp = checkTanVelocity(theta);
    yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    yInfo("theta angle [rad] %f", theta); 
}

Vector* CVMotionProfile::compute(double t, double t0) {
    if(t-t0 == 0) {
        theta = 0;
    }
    else {
        theta =  thetaPrev + (t - tprev) * angVelocity;
    }
    
    Vector xdes = *xd;
    if(theta == 0) {
        yInfo("theta=0 check");
        preComputation(theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2]; 
        (*xd)[0]=O[0] + radius * cos(theta) * AO[0] + radius * sin(theta) * BO[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AO[1] + radius * sin(theta) * BO[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AO[2] + radius * sin(theta) * BO[2];
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){   
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            Time::delay(5.0);
        }
    }
    else if ((theta >= thetaA) && (theta<=thetaC)) {
        yInfo("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        preComputation(theta);        
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AO[0] + radius * sin(theta) * BO[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AO[1] + radius * sin(theta) * BO[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AO[2] + radius * sin(theta) * BO[2]; 
    
        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return NULL;
    }
    
    Vector distance = (*xd) - xPrev;
    yInfo("travelled distance x = %f , travelled distance y = %f absolute = %f", distance[1]/(t-tprev), distance[2]/(t-tprev), 
    sqrt(distance[1] * distance[1] + distance[2] * distance[2])/(t-tprev));     
    

    // updating the previous incremental step values
    radiusPrev =  radius;
    thetaPrev = theta;    
    tprev     = t;    
    xPrev = (*xd);
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
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;  
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
    xPrev.resize(3); 
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;  
}

TTPLMotionProfile::~TTPLMotionProfile(){
    delete xd;
}    

TTPLMotionProfile::TTPLMotionProfile(const Bottle& bInit) {
    type = "TTPL";    
    valid = false;    
    
    A.resize(3);
    B.resize(3);
    C.resize(3); 
    O.resize(3);       
    AO.resize(3);
    BO.resize(3);
    xPrev.resize(3);
    xd = new Vector(3);
    thetaPrev = 0;
    tprev = 0;
    radiusPrev = 0.1;  
    
    Bottle* b = bInit.get(0).asList();
    if(b->size() < 6){
        //extracing the features from the bottle
        //((xa,ya,za) (xb,yb,zb) (xc,yc,zc) (0,0.7853,1.5707) (0.1 0.1 0.2 0.1))
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
        if(params->size() == 4){
            setAxes(params->get(0).asDouble(), params->get(1).asDouble());
            setGain(params->get(2).asDouble()); 
            setBeta(params->get(3).asDouble());
            setViaPoints(aVec, bVec, cVec);
            valid = true;
        }
    }   
}

bool TTPLMotionProfile::operator==(const TTPLMotionProfile &ttplmp)
{
    return ((valid==ttplmp.valid)&&(type==ttplmp.type));
}

double TTPLMotionProfile::computeTangVelocity() {
    double reBeta = -1 * beta;
    yDebug("reBeta %f", reBeta);
    double curvature = 1 / radius;
    yDebug("curvature %f", curvature);
    double vel = gain * pow(curvature, reBeta);
    yDebug("vel %f", vel);
    return vel;
}

void TTPLMotionProfile::preComputation(const double theta) {
    //angular velocity expressed in frequency [Hz],
    //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
    //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec         
    //ONLY FOR CIRCLES: angular velocity constant to obtain constant tang.velocity 
    //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
    radius = computeRadius(theta);
    /*    
    double rDiff2     = (radius - radiusPrev) * (radius-radiusPrev);
    double thetaDiff2 = (theta  - thetaPrev) * (theta - thetaPrev); 
    double sDiff2     = rDiff2  + (radius * radius) * thetaDiff2;
    yInfo("rDiff2 %f thetaDiff2 %f radius %f sDiff %f", rDiff2, thetaDiff2, radius, sqrt(sDiff2));
    */

    /*    
    double drdtheta   = (radius - radiusPrev) / (theta - thetaPrev);
    double drdtheta2  =  drdtheta * drdtheta;
    double v2         = (drdtheta2 + (radius * radius)) * (angVelocity * angVelocity);
    yInfo("v %f", sqrt(v2));
    */
    //tanVelocity = computeTangVelocity();
    tanVelocity = 0.1;
    yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
    // computing angular velocity in function of the radius, theta, tangential velocity
    angVelocity = computeAngVelocity(theta);
    yInfo("computed angular velocity %f in rad/s", angVelocity);
    double tanVelocity_temp = checkTanVelocity(theta);
    yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    yInfo("theta angle [rad] %f", theta); 
}

Vector* TTPLMotionProfile::compute(double t, double t0) {
    if(t-t0 == 0) {
        theta = 0;
    }
    else {
        theta =  thetaPrev + (t - tprev) * angVelocity;
    }
    
    Vector xdes = *xd;
    if(theta == 0) {
        yInfo("theta=0 check");
        preComputation(theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2]; 
        (*xd)[0]=O[0] + radius * cos(theta) * AO[0] + radius * sin(theta) * BO[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AO[1] + radius * sin(theta) * BO[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AO[2] + radius * sin(theta) * BO[2];
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){   
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            Time::delay(5.0);
        }
    }
    else if ((theta >= thetaA) && (theta<=thetaC)) {
        yInfo("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        preComputation(theta);        
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AO[0] + radius * sin(theta) * BO[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AO[1] + radius * sin(theta) * BO[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AO[2] + radius * sin(theta) * BO[2]; 
    
        yInfo("xdes %s", xd->toString().c_str());
    }
    else {
        return NULL;
    }
    
    Vector distance = (*xd) - xPrev;
    yInfo("travelled distance x = %f , travelled distance y = %f absolute = %f", distance[1]/(t-tprev), distance[2]/(t-tprev), 
    sqrt(distance[1] * distance[1] + distance[2] * distance[2])/(t-tprev));     
    

    // updating the previous incremental step values
    radiusPrev =  radius;
    thetaPrev = theta;    
    tprev     = t;    
    xPrev = (*xd);
    return xd;    
    
}


