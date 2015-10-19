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


#ifndef _MOTION_PROFILE_H_
#define _MOTION_PROFILE_H_

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

namespace profileFactory {

class MotionProfile {
protected:
    yarp::sig::Vector O;               // vector representating the position of the center ellipse
    yarp::sig::Vector A;               // vector representating the initial position for the hand
    yarp::sig::Vector B;               // vector representating the final desired position for the hand
    yarp::sig::Vector C;               // vector representating the check point of the hand
    yarp::sig::Vector AO;              // vector from A to center of the ellipse
    yarp::sig::Vector BO;              // vector from B to center of the ellipse
    yarp::sig::Vector od;              // vector representing the desired orientation of the hand
    yarp::sig::Vector* xd;             // vector representing the desired position of the hand

    std::string type;                  // vocab representing the type

    double majAxis;                    // dimension of the major axis
    double minAxis;                    // dimension of the minor axis

    int a, b, c, d;                    // parameters of the plane 

    double thetaA;                     // angular position of the point A
    double thetaB;                     // angular position of the point B
    double thetaC;                     // angular position of the point C

    bool valid;                        // flag indicating whether the motionProfile is valid class

public:
    /**
    * constructor default
    */
    MotionProfile();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    MotionProfile(int i){};

    /**
     * destructor
     */
    ~MotionProfile();

    /**
    * checks whether the parameters are valid
    */
    bool isValid() const {return valid; };

    /**
    * function solving the operator ==
    */  
    virtual bool operator==(const MotionProfile& mp)=0;

    /**
    * function that sets where the action should start and stop
    * @param thetaA angle of the starting point (in rad)
    * @param thetaB angle of the viapoint (in rad)
    * @param thataC angle of the stop point (in rad)
    */   
    void setStartStop(const double thetaA, const double thetaB, const double thetaC);

    /**
    * function to set the three via points in 3D space
    */
    void setCenter(yarp::sig::Vector O);

    /**
    * function to set the three via points in 3D space
    */
    void setViaPoints(const yarp::sig::Vector A,const yarp::sig::Vector B,const yarp::sig::Vector C);

    /**
    * function to se the main axes of the ellipse that belong to the reference plane
    */
    void setAxes(const double majAxis,const double minAxis);

    /**
    * function that computes the angular velocity given desired tang.Velocity, minAxis and majAxis
    */
    double computeAngVelocity();

    /**
    * function that computed the radius in ellipse give a theta angle
    * @param theta angle at which the radius is computed [rad]
    * @return radius of the ellipse/circle at a given angle theta
    */
    double computeRadius(const double theta);

    /**
    * vector returning the 3D location at the instant t 
    * @return the pointer to the desired position of the hand
    */  
    virtual yarp::sig::Vector* compute(double t,double t0) = 0;
    
};

/**
* motion profile with constant velocity
*/
class CVMotionProfile : public MotionProfile {
protected:
    
    double velocity;          // desired tangential velocity
    double angVelocity;       // computed angular velocity

public:
    CVMotionProfile();
    ~CVMotionProfile();
    CVMotionProfile(const CVMotionProfile &cvmp);
    CVMotionProfile(const yarp::os::Bottle &b);
  
    CVMotionProfile &operator=(const CVMotionProfile &cvmp);
    bool operator==(const CVMotionProfile &cvmp);
    bool operator==(const MotionProfile &mp) {return operator==(dynamic_cast<const CVMotionProfile&>(mp));}    

    /**
    * function that sets the desired tangential velocity of the endEffector
    */
    void setVelocity(const double vel) {velocity = vel;};	
    yarp::sig::Vector* compute(double t, double t0);
    double computeRadius(const double theta);
};


/**
* motion profile with minimumJerk profile
*/
class MJMotionProfile : public MotionProfile {
protected:
    
    double timeHorizon;
    double distance;

public:
    MJMotionProfile();
    ~MJMotionProfile(){};        
    MJMotionProfile(const MJMotionProfile &mjmp);

    MJMotionProfile &operator=(const MJMotionProfile &mjmp);
    bool operator==(const MJMotionProfile &mjmp);    
};


/**
* motion profile respecting the 2/3 power law
*/
class TTPLMotionProfile : public MotionProfile {
protected:
    
    double gain;
    double beta;
    double radius;           // radius of the ellipse function of the angle theta, a, b;
    double tanVelocity;      // tangential velocity function of curvature, gain and beta
    double angVelocity;      // angular velocity in rad/s
    double theta;            // angle in rad
    double thetaPrev;        // previous angle in rad

public:
    TTPLMotionProfile();
    ~TTPLMotionProfile();        
    TTPLMotionProfile(const TTPLMotionProfile &ttplmp);
    TTPLMotionProfile(const yarp::os::Bottle &b);

    TTPLMotionProfile &operator=(const MJMotionProfile &ttplmp);
    bool operator==(const TTPLMotionProfile &ttplmp);    

    void setGain(const double _gain) { gain = _gain; };
    void setBeta(const double _beta) { beta = _beta; };
    double computeTangVelocity();
    yarp::sig::Vector* compute(double t, double t0);
};



}
#endif  //_MOTION_PROFILE_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

