// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C)2020  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Omar Eldardeer
  * email: omar.eldardeer@iit.it
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
#include "iCub/attentionManagerThread.h"

#define THPERIOD 0.01

attentionManagerThread::attentionManagerThread(string moduleName):PeriodicThread(THPERIOD){

    this->moduleName = moduleName;


}

attentionManagerThread::~attentionManagerThread(){

}
bool attentionManagerThread::configure(yarp::os::ResourceFinder &rf){

    return true;
}


bool attentionManagerThread::threadInit() {

    /* ===========================================================================
	 *  Initialize all ports. If any fail to open, return false to
	 *    let RFModule know initialization was unsuccessful.
	 * =========================================================================== */

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}
void attentionManagerThread::run() {


}

void attentionManagerThread::threadRelease() {

    //-- Stop all threads.


    //-- Close the threads.

}


string attentionManagerThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}