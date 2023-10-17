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
 * @file Observable.h
 * @brief Definition of the class which must interrupt the Observer
 * (see Observer.h).
 */

#ifndef _OBSERVABLE_H_
#define _OBSERVABLE_H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Bottle.h>
#include <yarp/sig/all.h>
#include <iostream>
#include <set>

//within project includes
#include <iCub/observer.h>

class observable {
private:
    bool changed;
    std::set<observer*> observers;

protected:
    virtual void setChanged() { changed = true; }

    virtual void clearChanged(){ changed = false; }

public:
    virtual void addObserver(observer& o) {
        observers.insert(&o);
    }
    
    virtual void deleteObserver(observer& o) {
        observers.erase(&o);
    }
    
    virtual void deleteObservers() {
        observers.clear();
    }
    
    virtual int countObservers() {
        return observers.size();
    }

    virtual bool hasChanged() { return changed; }

    /**
    * If this object has changed, notify all
    * of its observers:
    */
    virtual void notifyObservers(yarp::os::Bottle* arg=0) {
        if(!hasChanged()) {
            return;
        }
        clearChanged(); // Not "changed" anymore
        std::set<observer*>::iterator it;
        for(it = observers.begin(); it != observers.end(); it++) {
            (*it)->update(this, arg);
        }
    }
};

#endif  //_OBSERVABLE_H_
//----- end-of-file --- ( next line intentionally left blank ) ------------------

