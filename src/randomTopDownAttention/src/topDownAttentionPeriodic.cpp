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

//the constructor function
topDownAttentionPeriodic::topDownAttentionPeriodic(double p,string moduleName):PeriodicThread(p){
    //set the module name
    this->moduleName = moduleName;

    //initialize the output port name with moduleName/cmd
    this->clientName = moduleName+"/cmd";

    //create a new state array of pointers of 6 states and create instances from each state
    attentionStates = new state*[6];
    attentionStates[0] = new attendingColourfulState();
    attentionStates[1] = new attendingMovementState();
    attentionStates[2] = new attendingFacesState();
    attentionStates[3] = new attendingAudioState();
    attentionStates[4] = new attendingIntensityState();
    attentionStates[5] = new attendingObjectState();

    //initialize the randomisation function
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
    //this function is called each periodically

    //this to make sure that the port is connected
    if (port.getOutputCount()==0) {
        printf("No Connection to  %s\n", clientName.c_str());
    } else {
        // if the port is connected, create a bottle output command
        Bottle cmd;

        //then randommly pich a number between 0 and 5
        int randomNum = (rand()%6);

        //get the command of this the state which has an index of the generated random number in the state array
        cmd = attentionStates[randomNum]->getSettings();
        yInfo("Sending message... ");
        helperFunctions::printBottle(cmd);
        Bottle response;
        //send the command, recieve the responce, and print the responce 
        port.write(cmd,response);
        yInfo("Got response: %s\n", response.toString().c_str());
    }
}
