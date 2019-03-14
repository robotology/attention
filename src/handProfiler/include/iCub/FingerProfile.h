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
 * @file MotionProfile.h
 * @brief Definition of the profile of the movement as plannar parmatric ellipse oriented in the space.
 */


#ifndef _FINGER_PROFILE_H_
#define _FINGER_PROFILE_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/math/Math.h>

#include <gsl/gsl_math.h>

#include <iostream>
#include <fstream>
#include <time.h>
#include <cv.h>
#include <stdio.h>
//#include <cmath>
#include <math.h>

#define minX_DF  -0.35;  // working space x
#define maxX_DF  -0.20;  // working space x
#define minY_DF  -0.35;  // working space y
#define maxY_DF  +0.35;  // working space y
#define minZ_DF  -0.15;  // working space z
#define maxZ_DF  +0.25;  // working space z

namespace fingerFactory {

class FingerProfile {
private:
   
    yarp::os::BufferedPort<yarp::os::Bottle>  dataPort;            // output port to plot event
    
	void initRange();


protected:
    yarp::sig::Vector graspHome;
    yarp::sig::Vector graspVia;
    yarp::sig::Vector graspFinal;
    yarp::sig::Vector* nextPosition;
    double graspHomeTime;
    double graspViaTime;
    double graspFinalTime;
    
public:
    /**
    * constructor default
    */
    FingerProfile();

    /**
    * constructor
    * @param robotname name of the robot
    */
    FingerProfile(int i){};

    /**
     * destructor
     */
    ~FingerProfile();

    /**
     * function that set the time when the grasp should start
     * @param time time in second when
     */
    void setHomeTime(double time) { graspHomeTime = time;};

    /**
     * function that set the time when the via point should be visited
     * @param time time in second when
     */
    void setViaTime(double time) { graspViaTime = time;};

    /**
     * function that set the time when the grasp should finish
     * @param time time in second when
     */
    void setFinalTime(double time) { graspFinalTime = time;};
    

    /**
     * computing the vector location
     * @return the vector representing the next position
     */
    virtual yarp::sig::Vector* compute(yarp::sig::Vector target, double t) = 0;
};


/**
* motion profile with constant velocity
*/
class CVFingerProfile : public FingerProfile {
protected:
    
    //double velocity;          // desired tangential velocity

public:
    CVFingerProfile();
    ~CVFingerProfile();
    //CVFingerProfile(const CVFingerProfile &cvmp);
    //CVFingerProfile(const yarp::os::Bottle &b);
  
    //CVFingerProfile &operator=(const CVFingerProfile &cvmp);
    //bool operator==(const CVFingerProfile &cvmp);
    //bool operator==(const FingerProfile &mp) {return operator==(dynamic_cast<const CVFingerProfile&>(mp));}    

    /**
    * function that sets the desired tangential velocity of the endEffector
    */
    //void setVelocity(const double vel) {tanVelocity = vel;};
    //void preComputation(const double t, const double theta);	
    yarp::sig::Vector* compute(yarp::sig::Vector target, double t);
};

    //************************************************************************************************

/**
* motion profile with constant velocity and a via point
*/
class CVVFingerProfile : public FingerProfile {
protected:
    
    //double velocity;          // desired tangential velocity

public:
    CVVFingerProfile();
    ~CVVFingerProfile();
    //CVFingerProfile(const CVFingerProfile &cvmp);
    //CVFingerProfile(const yarp::os::Bottle &b);
  
    //CVFingerProfile &operator=(const CVFingerProfile &cvmp);
    //bool operator==(const CVFingerProfile &cvmp);
    //bool operator==(const FingerProfile &mp) {return operator==(dynamic_cast<const CVFingerProfile&>(mp));}    

    /**
    * function that sets the desired tangential velocity of the endEffector
    */
    //void setVelocity(const double vel) {tanVelocity = vel;};

    
    //void preComputation(const double t, const double theta);

    
    yarp::sig::Vector* compute(yarp::sig::Vector target, double t);
};
    
    
}
#endif  //_FINGER_PROFILE_H_PROFILE_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
