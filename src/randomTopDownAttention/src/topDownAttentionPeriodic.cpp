/*
  * Copyright (C)2019  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
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

#include <iCub/topDownAttentionPeriodic.h>
#include "iCub/helperFunctions.h"
#include "iCub/attendingColourfulState.h"
#include "iCub/attendingMovementState.h"
#include "iCub/attendingFacesState.h"
#include "iCub/attendingAudioState.h"
#include "iCub/attendingIntensityState.h"
#include "iCub/attendingObjectState.h"
#include <cstdlib>
#include <ctime>
using namespace std;







topDownAttentionPeriodic::topDownAttentionPeriodic(double p,string moduleName):PeriodicThread(p){
    msgCount=0;
    this->moduleName = moduleName;
    this->clientName = moduleName+"/cmd";
    attentionStates = new state*[6];
    attentionStates[0] = new attendingColourfulState();
    attentionStates[1] = new attendingMovementState();
    attentionStates[2] = new attendingFacesState();
    attentionStates[3] = new attendingAudioState();
    attentionStates[4] = new attendingIntensityState();
    attentionStates[5] = new attendingObjectState();
    srand(time(NULL));

}

void topDownAttentionPeriodic::afterStart(bool s)
{
    if (s)
        yInfo("Thread started successfully\n");
    else
        yError("Thread did not start\n");
}

bool topDownAttentionPeriodic::threadInit()
{
  port.open(clientName);
  yInfo("Starting thread\n");
  return true;
}
void topDownAttentionPeriodic::run()
{
    sendAttentionToPort();
}
void topDownAttentionPeriodic::threadRelease()
{
    yInfo("Goodbye from thread\n");
}

void topDownAttentionPeriodic::sendAttentionToPort(){
    if (port.getOutputCount()==0) {
        printf("No Connection to  %s\n", clientName.c_str());
    } else {
        Bottle cmd;
        int randomNum = (rand()%6);
        cmd = attentionStates[randomNum]->getSettings();
        yInfo("Sending message... ");
        helperFunctions::printBottle(cmd);
        Bottle response;
        port.write(cmd,response);
        yInfo("Got response: %s\n", response.toString().c_str());
    }
}
