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
    graspHomePitch.resize(9);
    graspViaPitch.resize(9);
    graspFinalPitch.resize(9);
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

    //pitch
    graspHomePitch[0] = 40.0;
    graspHomePitch[1] = 40.0;
    graspHomePitch[2] = 0.0;
    graspHomePitch[3] = 0.0;
    graspHomePitch[4] = 0.0;
    graspHomePitch[5] = 0.0;
    graspHomePitch[6] = 0.0;
    graspHomePitch[7] = 0.0;
    graspHomePitch[8] = 0.0;
    graspViaPitch[0] = 40.0;
    graspViaPitch[1] = 50.0;
    graspViaPitch[2] = 10.0;
    graspViaPitch[3] = 20.0;
    graspViaPitch[4] = 40.0;
    graspViaPitch[5] = 7.5;
    graspViaPitch[6] = 55.0;
    graspViaPitch[7] = 165.0;
    graspViaPitch[8] = 232.0;
    graspFinalPitch[0] = 41.0;
    graspFinalPitch[1] = 60.0;
    graspFinalPitch[2] = 20.0;
    graspFinalPitch[3] = 30.0;
    graspFinalPitch[4] = 80.0;
    graspFinalPitch[5] = 15.0;
    graspFinalPitch[6] = 55.0;
    graspFinalPitch[7] = 165.0;
    graspFinalPitch[8] = 232.0;
    graspHome = graspHomePitch;
    graspVia  = graspViaPitch;
    graspFinal= graspFinalPitch;
    
    //power
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
    type = "CVV";
    nextPosition = new Vector(9);
}

CVVFingerProfile::CVVFingerProfile(const Bottle& bInit){
    type = "CVV";
    nextPosition = new Vector(9);


    Bottle* b = bInit.get(0).asList();
    ResourceFinder rf;
    rf.setVerbose(true);
	//fix: max size would be 8 * 2 + 1; round it to 20 @amaroyo 18/01/2016
    //int argc = b->size() * 2 + 1;
	// fix:
	const int argc = 20;
    string stringArray[argc];
    char* argv[argc];
    stringArray[0].append("./motionProfile");
    argv[0] = (char*) stringArray[0].c_str();
    //yDebug("added first %s", argv[0]);

    for (int j = 0; j < b->size(); j++) {
        Bottle* vector = b->get(j).asList();
        stringArray[j * 2 + 1].append("--");
        //stringArray[j * 2 + 1].append("A");
        stringArray[j * 2 + 1].append(vector->get(0).asString().c_str());
        char temp[50];
        sprintf(temp,"%f %f %f", vector->get(1).asFloat32(), vector->get(2).asFloat32(), vector->get(3).asFloat32());
        yDebug("stringArray %s", stringArray[j * 2 + 1].c_str());
        stringArray[j * 2 + 2].append(&temp[0]);
        argv[j * 2 + 1] = (char*) stringArray[j * 2 + 1].c_str();
        argv[j * 2 + 2] = (char*) stringArray[j * 2 + 2].c_str();
        yDebug("param %d %s", j, argv[j * 2 + 1]);
        yDebug("value %s", argv[j * 2 + 2]);
    }
    yDebug("parsing ");
    yDebug("%s",argv[0] );
    yDebug("%s, %s",argv[1], argv[2] );
    // configuring the resource finder
    rf.configure(b->size() * 2 + 1, argv);
    yInfo("resorceFinder: %s",rf.toString().c_str());
    // visiting the parameters using the RF
    Vector tvector(3);
    Vector pvector(3);
    Vector pitchVector(3);
    
    string pitchString = rf.check("pitch",
                           Value("0.1 0.1 0.1"),
                           "pitch grasp (string)").asString();
    extractVector(pitchString, pitchVector);
    string tstring = rf.check("time",
                           Value("0.2 0.2 0.2"),
                           "time t (string)").asString();
    extractVector(tstring, tvector);
    yDebug("got t value %s", tvector.toString().c_str());
    string pstring = rf.check("param",
                           Value("0.3 0.3 0.3"),
                           "param p (string)").asString();
    extractVector(pstring, pvector);
    yDebug("got p value %s", pvector.toString().c_str());

    if(b->size() == 2){
        //extracing the features from the bottle
        //((ta,tb,tc) (pa,pb,pc))
        Bottle* tVector = b->get(0).asList();
        Vector tVec(3);
        tVec[0] = tVector->get(1).asFloat32();
        tVec[1] = tVector->get(2).asFloat32();
        tVec[2] = tVector->get(3).asFloat32();
        yDebug("bottlet:%s", tVec.toString().c_str());

        Bottle* pVector = b->get(1).asList();
        Vector pVec(3);
        pVec[0] = pVector->get(1).asFloat32();
        pVec[1] = pVector->get(2).asFloat32();
        pVec[2] = pVector->get(3).asFloat32();
        yDebug("bottlep:%s", pVec.toString().c_str());

        
    }
    if(true) {
       setType("pitch");
    }
    setTiming(tvector);
    setParameters(pvector);
    
       
}

CVVFingerProfile::~CVVFingerProfile(){
    delete nextPosition;
}

Vector* CVVFingerProfile::compute(Vector target, double t) {
    
    nextPosition = new Vector(9);
    //nextPosition->clear(); // check if this is necessary
    //double speed2Via   = 300;
    //double speed2Final = 300;
    //graspHomeTime  = 0.01;   // time necessary to inizialize the home position
    //graspViaTime   = 0.3;    // time necessary to reach the viapoint position
    //graspFinalTime = 0.6;    // time necessary to reach the end position

    yWarning("tempo %f", t);

    if (t < graspHomeTime) {
        // before the beginning of the grasping
        yDebug("before for %f ", t);
        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = target[i+7];
        }
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

        for(int i = 0; i<9; i++){
            (*nextPosition)[i] = graspFinal[i];
        }
    }
    return nextPosition;
}
