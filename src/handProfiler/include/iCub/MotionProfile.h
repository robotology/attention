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

namespace profileFactory {



class MotionProfile {
protected:
    yarp::sig::Vector O;               // vector representating the position of the center ellipse
    yarp::sig::Vector A;               // vector representating the initial position for the hand
    yarp::sig::Vector B;               // vector representating the final desired position for the hand
    yarp::sig::Vector C;               // vector representating the check point of the hand
    yarp::sig::Vector AO;              // vector from A to center of the ellipse
    yarp::sig::Vector BO;              // vector from B to center of the ellipse
    yarp::sig::Vector od;              // vector representating the desired orientation for the hand

    std::string type;                  // vocab representing the type

    int majAxis;                       // dimension of the major axis
    int minAxis;                       // dimension of the minor axis

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
    * function to set the three via points in 3D space
    */
    void setCenter(yarp::sig::Vector O);

    /**
    * function to set the three via points in 3D space
    */
    void setViaPoints(yarp::sig::Vector A, yarp::sig::Vector B, yarp::sig::Vector C);

    /**
    * function to se the main axes of the ellipse that belong to the reference plane
    */
    void setAxes(double majAxis, double minAxis);

    /**
    * vector returning the 3D location at the instant t 
    */  
    yarp::sig::Vector compute(double t,double t0);
    
};

/**
* motion profile with constant velocity
*/
class CVMotionProfile : public MotionProfile {
protected:
    
    double velocity;

public:
    CVMotionProfile();
    CVMotionProfile(const CVMotionProfile &cvmp);
    CVMotionProfile(const yarp::os::Bottle &b);

    CVMotionProfile &operator=(const CVMotionProfile &cvmp);
    bool operator==(const CVMotionProfile &cvmp);
    bool operator==(const MotionProfile &mp) {return operator==(dynamic_cast<const CVMotionProfile&>(mp));}    
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
    MJMotionProfile(const MJMotionProfile &mjmp);

    MJMotionProfile &operator=(const MJMotionProfile &mjmp);
    bool operator==(const MJMotionProfile &mjmp);    
};



}

#endif  //_MOTION_PROFILE_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

