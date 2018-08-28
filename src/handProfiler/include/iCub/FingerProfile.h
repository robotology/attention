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
		yarp::sig::Vector graspHome;
		yarp::sig::Vector graspFinal;
        yarp::sig::Vector* nextPosition;
		yarp::os::BufferedPort<yarp::os::Bottle>  dataPort;            // output port to plot event

	void initRange();


protected:

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
     * computing the vector location
     */
    yarp::sig::Vector* compute(yarp::sig::Vector target);
};



}
#endif  //_FINGER_PROFILE_H_PROFILE_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
