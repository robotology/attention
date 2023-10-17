// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

/**
 * @file observer.h
 * @brief Definition of th interface that gives the directives to the implementation of Observers
 */

#ifndef _OBSERVER_H_
#define _OBSERVER_H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/sig/all.h>
#include <iostream>

class observable;

class argument{
};

class observer {
public:
  /**
  * Called by the observed object, whenever 
  *  the observed object is changed:
  */
    virtual void update(observable* o, yarp::os::Bottle * arg) = 0;
};


#endif  //_OBSERVER_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

