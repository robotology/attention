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


inline void extractVector(const string str, Vector& res) {
    std::string temp;
	size_t i = 0, start = 0, end;
	do {
		end = str.find_first_of (' ', start );
		temp = str.substr( start, end );
			res[i] = ( atof ( temp.c_str ( ) ) );
			++i;
		start = end + 1;
	} while ( start );
}

//**************************************************************************************************************

MotionProfile::MotionProfile() : valid(false), type("")  {

    initRange();
	
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

void MotionProfile::initRange(){
	 minX = -0.35;					   
     maxX = -0.20;					   
     minY = -0.35;					   
     maxY = +0.35;					   
     minZ = -0.15;					   
     maxZ = +0.25;		
}

void MotionProfile::setCenter(Vector _O){
    O = _O;
}

void MotionProfile::setStartStop(const double tA, const double tB, const double tC){  
    if(reverse == 1) {
        yInfo("Setting for NOT REVERSE");
        thetaA = tA; 
        thetaB = tB;
        thetaC = tC;     
    }
    else if(reverse == -1) { 
        yInfo("Setting for REVERSE %d", reverse);
        thetaA = tC; 
        thetaB = tB;
        thetaC = tA;
    }
    else {
        reverse = 0;
        thetaA = tA; 
        thetaB = tB;
        thetaC = tC;
    }
}

Vector MotionProfile::normalizeVector(const Vector u, double const radius) {
    double module = sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    yDebug("module %f", module);
    Vector unorm(3);
    if(module != 0) {
        unorm[0] = (u[0] / module);
        unorm[1] = (u[1] / module);
        unorm[2] = (u[2] / module);
    }
    return unorm;
}

void MotionProfile::setViaPoints(const Vector _A, const Vector _B, const Vector _C){  
    yInfo("MotionProfile::setViaPoints");
    A = _A; 
    B = _B;
    C = _C;     

    //       x            y           z
    //  O[0] = -0.3; O[1] = -0.1; O[2] = 0.1;

    // scaled by the radius in A
    AO[0] = A[0] - O[0];
    AO[1] = A[1] - O[1];
    AO[2] = A[2] - O[2];
    AOnorm = normalizeVector(AO, 0.1);
    //AO[0] = AO[0] / xAxis;  // x
    //AO[1] = AO[1] / xAxis;  // y
    //AO[2] = AO[2] / xAxis;  // z   
    
    yInfo("AO = %s", AO.toString().c_str());
    yInfo("AOnorm = %s scaled by radius: %f", AOnorm.toString().c_str(), xAxis);

    //scaled by the radius in B     
    BO[0] = B[0] - O[0];            
    BO[1] = B[1] - O[1];
    BO[2] = B[2] - O[2]; 
    BOnorm = normalizeVector(BO, 0.2);
    //BO[0] = BO[0];          // x
    //BO[1] = BO[1] / yAxis;  // y
    //BO[2] = BO[2] / yAxis;  // z

    yInfo("BO = %s", BO.toString().c_str());
    yInfo("BOnorm = %s scaled by the radius %f", BOnorm.toString().c_str(), yAxis);
    
    Vector N(3);    
    N = cross(AO, BO);
        
    yInfo("NormVector = %s", N.toString().c_str());

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

bool MotionProfile::sanityCheck(const Vector array[], const int size) {
    for(int i=0; i < size; i++) {  
        Vector toCheck = array[0];
        if((toCheck[0] < minX) || (toCheck[0] > maxX)) return false;
        if((toCheck[1] < minY) || (toCheck[1] > maxY)) return false;    
        if((toCheck[2] < minZ) || (toCheck[2] > maxZ)) return false;
    }
    return true;
}

Vector MotionProfile::getInitial() {
    Vector locInitial(3);
    //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
    radius = computeRadius(thetaA);
    //preComputation(t, thetaA);  //computes the radius
    locInitial[0]=O[0] + radius * cos(thetaA) * AOnorm[0] + radius * sin(thetaA) * BOnorm[0];
    locInitial[1]=O[1] + radius * cos(thetaA) * AOnorm[1] + radius * sin(thetaA) * BOnorm[1];
    locInitial[2]=O[2] + radius * cos(thetaA) * AOnorm[2] + radius * sin(thetaA) * BOnorm[2];
    yInfo("getting the initial position %s for thetaStart %f with radius", locInitial.toString().c_str(), thetaA, radius); 
    return locInitial;
}

bool MotionProfile::inRange(double theta) {
    //reverseGain = 1 when reverse = FALSE
    if(reverse == 1) {
        if((theta > thetaA) && (theta< thetaC)) 
            return true;
        else
            return false;
    }
    //reverseGain = -1 when reverse = TRUE
    else {
       if((theta > thetaC) && (theta < thetaA)) 
            return true;
        else
            return false; 
    }
}

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

    Bottle* b = bInit.get(0).asList();
    ResourceFinder rf;
    rf.setVerbose(true);
	//fix: max size would be 8 * 2 + 1; round it to 20 @amaroyo 18/01/2016
    //int argc = b->size() * 2 + 1;   
	const int argc = 20;
    string stringArray[argc];  
    char* argv[argc];
    stringArray[0].append("./motionProfile");
    argv[0] = (char*) stringArray[0].c_str();
    yDebug("added the first %s", argv[0]);    
        
    for (int j = 0; j < b->size(); j++) {
        Bottle* vector = b->get(j).asList();
        stringArray[j * 2 + 1].append("--");
        //stringArray[j * 2 + 1].append("A");    
        stringArray[j * 2 + 1].append(vector->get(0).asString().c_str());
        char temp[50];
        sprintf(temp,"%f %f %f", vector->get(1).asDouble(), vector->get(2).asDouble(), vector->get(3).asDouble());
        stringArray[j * 2 + 2].append(&temp[0]);
        argv[j * 2 + 1] = (char*) stringArray[j * 2 + 1].c_str();
        argv[j * 2 + 2] = (char*) stringArray[j * 2 + 2].c_str();
        yDebug("param %s", argv[j * 2 + 1]);
        yDebug("value %s", argv[j * 2 + 2]);
    }     
    // configuring the resource finder
    rf.configure(argc, argv);
    yInfo("resorceFinder: %s",rf.toString().c_str());
    // visiting the parameters using the RF
    Vector Ovector(3);    
    Vector Avector(3);
    Vector Bvector(3);
    Vector Cvector(3);
    string Ostring = rf.check("O", 
                           Value("-0.3 -0.1 0.1"), 
                           "position O (string)").asString();
    extractVector(Ostring, Ovector);
    yDebug("got O value %s", Ovector.toString().c_str());
    string Astring = rf.check("A", 
                           Value("-0.3 0.0 0.1"), 
                           "position A (string)").asString();
    extractVector(Astring, Avector);
    yDebug("got A value %s", Avector.toString().c_str());
    string Bstring = rf.check("B", 
                           Value("-0.3 -0.1 0.3"), 
                           "position B (string)").asString();
    extractVector(Bstring, Bvector);
    yDebug("got B value %s", Bvector.toString().c_str());
    string Cstring = rf.check("C", 
                           Value("-0.3 -0.2 0.1"), 
                           "position C (string)").asString();
    extractVector(Cstring, Cvector);
    yDebug("got C value %s", Cvector.toString().c_str());
    Vector thetaVector(3);
    string  thetaString = rf.check("theta", 
                           Value("0 1.57 6.28"), 
                           "theta angles (string)").asString();
    extractVector(thetaString, thetaVector);
    yDebug("got theta angles:(%s)", thetaVector.toString().c_str());
    Vector axisVector(2);
    string  axisString = rf.check("axes", 
                           Value("0.1 0.2"), 
                           "minor and major axes (string)").asString();
    extractVector(axisString, axisVector);
    yDebug("got minor and major axes:(%s)", axisVector.toString().c_str());
    Vector paramVector(1);
    string  paramString = rf.check("param", 
                           Value("0.1"), 
                           "profile parameters (string)").asString();
    extractVector(paramString, paramVector);
    yDebug("got profile parameters:(%s)", paramVector.toString().c_str());
    bool reverse = rf.check("rev");
    reverse? yDebug("reverse is ON") : yDebug("reverse is ON");
    
    if(b->size() == 6){
        //extracing the features from the bottle
        //((xa,ya,za) (xb,yb,zb) (xc,yc,zc) (0,0.7853,1.5707) (0.1))
        Bottle* aVector = b->get(0).asList();            
        Vector aVec(3);
        aVec[0] = aVector->get(1).asDouble();
        aVec[1] = aVector->get(2).asDouble();
        aVec[2] = aVector->get(3).asDouble();
        yDebug("bottleA:%s", aVec.toString().c_str());

        Bottle* bVector = b->get(1).asList();    
        Vector bVec(3);
        bVec[0] = bVector->get(1).asDouble();
        bVec[1] = bVector->get(2).asDouble();
        bVec[2] = bVector->get(3).asDouble();
        yDebug("bottleB:%s", bVec.toString().c_str()); 
    
        Bottle* cVector = b->get(2).asList(); 
        Vector cVec(3);
        cVec[0] = cVector->get(1).asDouble();
        cVec[1] = cVector->get(2).asDouble();
        cVec[2] = cVector->get(3).asDouble();
        yDebug("bottleC:%s", cVec.toString().c_str());       
        
        Bottle* angles  = b->get(3).asList();
        yDebug("angles:%s", angles->toString().c_str());
        setStartStop(angles->get(1).asDouble(), angles->get(2).asDouble(), angles->get(3).asDouble());  
    
        Bottle* axis  = b->get(4).asList();
        yDebug("axis:%s", axis->toString().c_str());
        setAxes(axis->get(1).asDouble(), axis->get(2).asDouble());     
    
        Bottle* params  = b->get(5).asList();
        yDebug("params:%s", params->toString().c_str());        
        setVelocity(params->get(1).asDouble());  
        setViaPoints(aVec, bVec, cVec);
        valid = true;
        
    }
    setCenter(Ovector);
    setReverse(reverse);
    setStartStop(thetaVector[0], thetaVector[1], thetaVector[2]);
    setAxes(axisVector[0], axisVector[1]);
    setVelocity(paramVector[0]);
    Vector array[3];
    array[0] = Avector;  
    array[1] = Bvector;
    array[2] = Cvector;
    valid = true;
    setViaPoints(Avector,Bvector,Cvector);
}

bool CVMotionProfile::operator==(const CVMotionProfile &cvmp)
{
    return ((valid==cvmp.valid)&&(type==cvmp.type));
}



void CVMotionProfile::preComputation(const double t, const double theta) {
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

Vector* CVMotionProfile::compute(double t) {
    if(t-t0 == 0) {
        theta = thetaA;
    }
    else {
        theta =  thetaPrev + (t - tprev) * angVelocity;
    }
    
    Vector xdes = *xd;
    if(theta == thetaA) {
        yInfo("theta=thetaA check");
        preComputation(t, theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2]; 
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){   
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            //Time::delay(5.0);
        }
    }
    else if ((theta > thetaA) && (theta<=thetaC)) {
        yInfo("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        preComputation(t, theta);        
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2]; 
    
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
    ResourceFinder rf;
    rf.setVerbose(true); 
	//fix: max size would be 8 * 2 + 1; round it to 20 @amaroyo 18/01/2016
    //int argc = b->size() * 2 + 1;   
	const int argc = 20;
    string stringArray[argc];  
    char* argv[argc];
    stringArray[0].append("./motionProfile");
    argv[0] = (char*) stringArray[0].c_str();
    yDebug("added the first %s", argv[0]);    
        
    for (int j = 0; j < b->size(); j++) {
        Bottle* vector = b->get(j).asList();
        stringArray[j * 2 + 1].append("--");
        //stringArray[j * 2 + 1].append("A");    
        stringArray[j * 2 + 1].append(vector->get(0).asString().c_str());
        char temp[50];
        sprintf(temp,"%f %f %f", vector->get(1).asDouble(), vector->get(2).asDouble(), vector->get(3).asDouble());
        stringArray[j * 2 + 2].append(&temp[0]);
        argv[j * 2 + 1] = (char*) stringArray[j * 2 + 1].c_str();
        argv[j * 2 + 2] = (char*) stringArray[j * 2 + 2].c_str();
        yDebug("param %s", argv[j * 2 + 1]);
        yDebug("value %s", argv[j * 2 + 2]);
    }     
    // configuring the resource finder
    rf.configure(argc, argv);
    yInfo("resorceFinder: %s",rf.toString().c_str());
    // visiting the parameters using the RF
    Vector Ovector(3);    
    Vector Avector(3);
    Vector Bvector(3);
    Vector Cvector(3);
    string Ostring = rf.check("O", 
                           Value("-0.3 -0.1 0.1"), 
                           "position O (string)").asString();
    extractVector(Ostring, Ovector);
    yDebug("got O value %s", Ovector.toString().c_str());
    string  Astring = rf.check("A", 
                           Value("-0.3 0.0 0.1"), 
                           "position A (string)").asString();
    extractVector(Astring, Avector);
    yDebug("got A value %s", Avector.toString().c_str());
    string  Bstring = rf.check("B", 
                           Value("-0.3 -0.1 0.2"), 
                           "position B (string)").asString();
    extractVector(Bstring, Bvector);
    yDebug("got B value %s", Bvector.toString().c_str());
    string  Cstring = rf.check("C", 
                           Value("-0.3 -0.2 0.1"), 
                           "position C (string)").asString();
    extractVector(Cstring, Cvector);
    yDebug("got C value %s", Cvector.toString().c_str());
    Vector thetaVector(3);
    string  thetaString = rf.check("theta", 
                           Value("0 1.57 6.28"), 
                           "theta angles (string)").asString();
    extractVector(thetaString, thetaVector);
    yDebug("got theta angles:(%s)", thetaVector.toString().c_str());
    Vector axisVector(2);
    string  axisString = rf.check("axes", 
                           Value("0.1 0.2"), 
                           "minor and major axes (string)").asString();
    extractVector(axisString, axisVector);
    yDebug("got minor and major axes:(%s)", axisVector.toString().c_str());
    Vector paramVector(1);
    string  paramString = rf.check("param", 
                           Value("0.1"), 
                           "profile parameters (string)").asString();
    extractVector(paramString, paramVector);
    yDebug("got profile parameters:(%s)", paramVector.toString().c_str());
    bool reverse = rf.check("rev");
    reverse? yDebug("reverse is ON") : yDebug("reverse is OFF");

    
    if(b->size() == 6){
        //extracing the features from the bottle
        //((xa,ya,za) (xb,yb,zb) (xc,yc,zc) (0,0.7853,1.5707) (0.1 0.1 0.2 0.1))
        Bottle* aVector = b->get(0).asList();
        yDebug("bottleA:%s", aVector->toString().c_str());    
        Vector aVec(3);
        aVec[0] = aVector->get(1).asDouble();
        aVec[1] = aVector->get(2).asDouble();
        aVec[2] = aVector->get(3).asDouble();

        Bottle* bVector = b->get(1).asList();
        yDebug("bottleB:%s", bVector->toString().c_str());    
        Vector bVec(3);
        bVec[0] = bVector->get(1).asDouble();
        bVec[1] = bVector->get(2).asDouble();
        bVec[2] = bVector->get(3).asDouble();

        Bottle* cVector = b->get(2).asList();
        yDebug("bottleC:%s", cVector->toString().c_str());       
        Vector cVec(3);
        cVec[0] = cVector->get(1).asDouble();
        cVec[1] = cVector->get(2).asDouble();
        cVec[2] = cVector->get(3).asDouble();

        Bottle* angles  = b->get(3).asList();
        yDebug("angles:%s", angles->toString().c_str());
        setStartStop(angles->get(1).asDouble(), angles->get(2).asDouble(), angles->get(3).asDouble()); 
        
        Bottle* axes  = b->get(4).asList();
        yDebug("axes:%s", axes->toString().c_str());
        setAxes(axes->get(1).asDouble(), axes->get(2).asDouble());
    
        Bottle* params  = b->get(5).asList();
        yDebug("params:%s", params->toString().c_str());        
        setGain(params->get(1).asDouble()); 
        setBeta(params->get(2).asDouble());
        setViaPoints(aVec, bVec, cVec);
        valid = true;
        
    }
    setCenter(Ovector); 
    setReverse(reverse);
    setStartStop(thetaVector[0], thetaVector[1], thetaVector[2]);
    setAxes(axisVector[0], axisVector[1]);
    setGain(paramVector[0]);    
    setBeta(paramVector[1]);
    Vector array[3];
    array[0] = Avector;  
    array[1] = Bvector;
    array[2] = Cvector;
    valid = true;
    setViaPoints(Avector,Bvector,Cvector);
}

bool TTPLMotionProfile::operator==(const TTPLMotionProfile &ttplmp)
{
    return ((valid==ttplmp.valid)&&(type==ttplmp.type));
}

double TTPLMotionProfile::computeTangVelocity() {
    //v = g * K ^ (-beta);    beta = 0.33;
    double reBeta = -1 * beta;
    double curvature = 1 / radius;
    double vel = gain * pow(curvature, reBeta);
    return vel;
}

void TTPLMotionProfile::preComputation(const double  t, const double theta) {
    //angular velocity expressed in frequency [Hz],
    //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
    //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec         
    //ONLY FOR CIRCLES: angular veloc.090001	 0.099486

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
    tanVelocity = computeTangVelocity();
    yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
    // computing angular velocity in function of the radius, theta, tangential velocity
    angVelocity = computeAngVelocity(theta);
    yInfo("computed angular velocity %f in rad/s", angVelocity);
    double tanVelocity_temp = checkTanVelocity(theta);
    yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    yInfo("theta angle [rad] %f", theta); 
}

Vector* TTPLMotionProfile::compute(double t) {
    double thetaStart;
    
    if(t-t0 == 0) {
        theta = thetaA;
        thetaStart =  thetaA;
    }
    else {
        theta =  thetaPrev + reverse * (t - tprev) * angVelocity;
    }

    Vector xdes = *xd;
    if(theta == thetaStart) {
        yInfo("theta=thetaStart check");
        preComputation(t, theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2]; 
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){   
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            //Time::delay(5.0);
        }
    }
    else if (inRange(theta)) {
        yInfo("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        preComputation(t, theta);        
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];     
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

MJMotionProfile::MJMotionProfile(){
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

MJMotionProfile::MJMotionProfile(const MJMotionProfile &cvmp){
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

MJMotionProfile::~MJMotionProfile(){
    delete xd;
}
    
MJMotionProfile::MJMotionProfile(const Bottle& bInit) {
    type = "MJP";
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

    Bottle* b = bInit.get(0).asList();
    yDebug("Analyzing the list of vectors from bottle b:%s", b->toString().c_str());
    ResourceFinder rf;
    rf.setVerbose(true);
    //fix: max size would be 8 * 2 + 1; round it to 20 @amaroyo 18/01/2016
    //int argc = b->size() * 2 + 1;   
	const int argc = 20;
    string stringArray[argc];  
    char* argv[argc];
    stringArray[0].append("./motionProfile");
    argv[0] = (char*) stringArray[0].c_str();
    yDebug("added the first %s", argv[0]);    
        
    for (int j = 0; j < b->size(); j++) {
        Bottle* vector = b->get(j).asList();
        stringArray[j * 2 + 1].append("--");
        //stringArray[j * 2 + 1].append("A");    
        stringArray[j * 2 + 1].append(vector->get(0).asString().c_str());
        char temp[50];
        sprintf(temp,"%f %f %f", vector->get(1).asDouble(), vector->get(2).asDouble(), vector->get(3).asDouble());
        stringArray[j * 2 + 2].append(&temp[0]);
        argv[j * 2 + 1] = (char*) stringArray[j * 2 + 1].c_str();
        argv[j * 2 + 2] = (char*) stringArray[j * 2 + 2].c_str();
        yDebug("param %s", argv[j * 2 + 1]);
        yDebug("value %s", argv[j * 2 + 2]);
    }     
    // configuring the resource finder
    rf.configure(argc, argv);
    yInfo("resorceFinder: %s",rf.toString().c_str());
    // visiting the parameters using the RF
    Vector Ovector(3);    
    Vector Avector(3);
    Vector Bvector(3);
    Vector Cvector(3);
    string Ostring = rf.check("O", 
                           Value("-0.3 -0.1 0.1"), 
                           "position O (string)").asString();
    extractVector(Ostring, Ovector);  
    string  Astring = rf.check("A", 
                           Value("-0.3 0.0 -0.1"), 
                           "position A (string)").asString();
    extractVector(Astring, Avector);
    yDebug("got A value %s", Avector.toString().c_str());
    string  Bstring = rf.check("B", 
                           Value("-0.3 0.0 -0.1"), 
                           "position B (string)").asString();
    extractVector(Bstring, Bvector);
    yDebug("got B value %s", Bvector.toString().c_str());
    string  Cstring = rf.check("C", 
                           Value("-0.3 0.0 -0.1"), 
                           "position C (string)").asString();
    extractVector(Cstring, Cvector);
    yDebug("got C value %s", Cvector.toString().c_str());
    Vector thetaVector(3);
    string  thetaString = rf.check("theta", 
                           Value("0 1.57 6.28"), 
                           "theta angles (string)").asString();
    extractVector(thetaString, thetaVector);
    yDebug("got theta angles:(%s)", thetaVector.toString().c_str());
    Vector axisVector(2);
    string  axisString = rf.check("axes", 
                           Value("0.1 0.2"), 
                           "minor and major axes (string)").asString();
    extractVector(axisString, axisVector);
    yDebug("got minor and major axes:(%s)", axisVector.toString().c_str());
    Vector paramVector(1);
    string  paramString = rf.check("param", 
                           Value("0.1"), 
                           "profile parameters (string)").asString();
    extractVector(paramString, paramVector);
    yDebug("got profile parameters:(%s)", paramVector.toString().c_str());
    bool reverse = rf.check("rev");
    reverse? yDebug("reverse is ON") : yDebug("reverse is ON");
    
    if(b->size() == 6){
        //extracing the features from the bottle
        //((xa,ya,za) (xb,yb,zb) (xc,yc,zc) (0,0.7853,1.5707) (0.1))
        Bottle* aVector = b->get(0).asList();            
        Vector aVec(3);
        aVec[0] = aVector->get(1).asDouble();
        aVec[1] = aVector->get(2).asDouble();
        aVec[2] = aVector->get(3).asDouble();
        yDebug("bottleA:%s", aVec.toString().c_str());

        Bottle* bVector = b->get(1).asList();    
        Vector bVec(3);
        bVec[0] = bVector->get(1).asDouble();
        bVec[1] = bVector->get(2).asDouble();
        bVec[2] = bVector->get(3).asDouble();
        yDebug("bottleB:%s", bVec.toString().c_str()); 
    
        Bottle* cVector = b->get(2).asList(); 
        Vector cVec(3);
        cVec[0] = cVector->get(1).asDouble();
        cVec[1] = cVector->get(2).asDouble();
        cVec[2] = cVector->get(3).asDouble();
        yDebug("bottleC:%s", cVec.toString().c_str());       
        
        Bottle* angles  = b->get(3).asList();
        yDebug("angles:%s", angles->toString().c_str());
        setStartStop(angles->get(1).asDouble(), angles->get(2).asDouble(), angles->get(3).asDouble());  
    
        Bottle* axis  = b->get(4).asList();
        yDebug("axis:%s", axis->toString().c_str());
        setAxes(axis->get(1).asDouble(), axis->get(2).asDouble());     
    
        Bottle* params  = b->get(5).asList();
        yDebug("params:%s", params->toString().c_str());        
        setVelocity(params->get(1).asDouble());  
        setViaPoints(aVec, bVec, cVec);
        valid = true;
    }
    setCenter(Ovector); 
    setStartStop(thetaVector[0], thetaVector[1], thetaVector[2]);
    setAxes(axisVector[0], axisVector[1]);
    setVelocity(paramVector[0]);
    Vector array[3];
    array[0] = Avector;  
    array[1] = Bvector;
    array[2] = Cvector;
    if(sanityCheck(array, 3)) {
        valid = true;
        setViaPoints(Avector,Bvector,Cvector);
    }
    else {
        yError("Error positions are not allowed!");
    }   
}

bool MJMotionProfile::operator==(const MJMotionProfile &cvmp)
{
    return ((valid==cvmp.valid)&&(type==cvmp.type));
}

double MJMotionProfile::computeTangVelocity(const double t) {
    //v = g * K ^ (-beta);    beta = 0.33;
    //double reBeta = -1 * beta;
    //double curvature = 1 / radius;
    thetaGoal = 1.57;
    double goalRadius = computeRadius(thetaGoal);
    Vector goal(3);
    goal[0]=O[0] + goalRadius * cos(thetaGoal) * AOnorm[0] + goalRadius * sin(thetaGoal) * BOnorm[0];
    goal[1]=O[1] + goalRadius * cos(thetaGoal) * AOnorm[1] + goalRadius * sin(thetaGoal) * BOnorm[1];
    goal[2]=O[2] + goalRadius * cos(thetaGoal) * AOnorm[2] + goalRadius * sin(thetaGoal) * BOnorm[2]; 
    Vector distanceVect = goal - (*xd);
    yDebug("distanceVector: %s", distanceVect.toString().c_str());
    double distance = sqrt(distanceVect[0] * distanceVect[0] + distanceVect[1] * distanceVect[1] + distanceVect[2] * distanceVect[2]);
    yDebug("distance:%f", distance);
    double epsilon = (t - t0) / (tfinal - t0);
    yDebug("epsilon:%f t:%f tfinal:%f", epsilon, t, tfinal);
    double epsilon3 = epsilon * epsilon * epsilon;
    double epsilon4 = epsilon3 * epsilon;
    double epsilon5 = epsilon4 * epsilon;
    double vel = distance * (15 * epsilon4  - 6 * epsilon5  - 10 * epsilon3);
    return vel;
}


void MJMotionProfile::preComputation(const double t, const double theta) {
    //angular velocity expressed in frequency [Hz],
    //e.g.: 0.1 => 1/10 of 2PI per second; 2PI in 10sec
    //e.g.: 0.2 => 1/20 of 2PI per second; 2PI in 20sec         
    //ONLY FOR CIRCLES: angular velocity constant to obtain constant tang.velocity  
    //FOR ELLIPSE: compute angular velocity from A,B and desired tang.velocity
    radius = computeRadius(theta);
    tanVelocity = computeTangVelocity(t);
    yInfo("computed radius %f [m] for tangVelocity %f [m/s]", radius, tanVelocity);
    // computing angular velocity in function of the radius, theta, tangential velocity
    angVelocity = computeAngVelocity(theta);
    yInfo("computed angular velocity %f in rad/s", angVelocity);
    double tanVelocity_temp = checkTanVelocity(theta);
    yInfo("computed tang velocity %f in m/s", tanVelocity_temp);
    //double theta = 2.0 * M_PI * angVelocity * (t - t0);
    yInfo("theta angle [rad] %f", theta); 
}

Vector* MJMotionProfile::compute(double t) {
    if(t-t0 == 0) {
        theta = thetaA;
        tfinal = t0 + 5.0;
    }
    else {
        theta =  thetaPrev + (t - tprev) * angVelocity;
    }
    
    Vector xdes = *xd;
    if(theta == thetaA) {
        yInfo("theta=thetaA check");
        preComputation(t,theta);
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2]; 
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2];
        if( ((*xd)[0]!=A[0]) || ((*xd)[1]!=A[1]) || ((*xd)[2]!=A[2]) ){   
            yInfo("Beware: difference between desired location A and computed position!");
            yInfo("vector xdes: %s", xd->toString().c_str());
            //Time::delay(5.0);
        }
    }
    else if ((theta > thetaA) && (theta<=thetaC)) {
        yInfo("In the range xAxis:%f yAxis:%f", xAxis,yAxis);
        preComputation(t, theta);        
        //(*xd)[0]=O[0] + xAxis * cos(theta) * AO[0] + yAxis * sin(theta) * BO[0];
        //(*xd)[1]=O[1] + xAxis * cos(theta) * AO[1] + yAxis * sin(theta) * BO[1];
        //(*xd)[2]=O[2] + xAxis * cos(theta) * AO[2] + yAxis * sin(theta) * BO[2];
        (*xd)[0]=O[0] + radius * cos(theta) * AOnorm[0] + radius * sin(theta) * BOnorm[0];
        (*xd)[1]=O[1] + radius * cos(theta) * AOnorm[1] + radius * sin(theta) * BOnorm[1];
        (*xd)[2]=O[2] + radius * cos(theta) * AOnorm[2] + radius * sin(theta) * BOnorm[2]; 
    
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
