// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
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
    attentionStates[0] = state("INTENSITY");
    attentionStates[1] = state("MOTION");
    attentionStates[2] = state("CHROMINANCE");
    attentionStates[3] = state("ORIENTATION");
    attentionStates[4] = state("EDGES");
    attentionStates[5] = state("BLOB");
    attentionStates[6] = state("FACE");
    attentionStates[7] = state("BIO_MOTION");
    attentionStates[8] = state("AUDIO");

    randomMode = true;

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
    sendAttentionToPort(ATTENTION_MODES::RANDOM);
}
void topDownAttentionPeriodic::threadRelease()
{
    yDebug("Releasing the thread \n");
    port.interrupt();
    port.close();
}




void topDownAttentionPeriodic::sendAttentionToPort(ATTENTION_MODES mode) {
    //this to make sure that the port is connected
    if (port.getOutputCount()==0) {
        printf("No Connection to  %s\n", clientName.c_str());
    } else {
        // if the port is connected, create a bottle output command
        Bottle cmd;
        int modeIdx = (rand()%attentionStates.size());
        if(mode != ATTENTION_MODES::RANDOM)
            modeIdx = modesIndMap.at(mode);

        //get the command of this the state which has an index of the generated random number in the state array
        Bottle* allCmds = attentionStates[modeIdx].getSettings();
        for(int i =0; i< attentionStates[modeIdx].getSettingsSize(); i++){
            cmd = allCmds[i];
            yInfo("Sending message... ");
            helperFunctions::printBottle(cmd);
            Bottle response;
            //send the command, receive the response, and print the response
            port.write(cmd,response);
            yInfo("Got response: %s\n", response.toString().c_str());
        }
    }
}


void topDownAttentionPeriodic::setRandomMode() {
    randomMode = true;
    if(isSuspended())
        resume();
}

void topDownAttentionPeriodic::setBlobMode() {
    randomMode = false;
    if(!isSuspended())
        suspend();
    sendAttentionToPort(ATTENTION_MODES::BLOB);
}

void topDownAttentionPeriodic::setIntensityMode() {
    randomMode = false;
    if(!isSuspended())
        suspend();
    sendAttentionToPort(ATTENTION_MODES::INTENSITY);
}

void topDownAttentionPeriodic::setChrominanceMode() {
    randomMode = false;
    if(!isSuspended())
        suspend();
    sendAttentionToPort(ATTENTION_MODES::CHROMINANCE);
}

void topDownAttentionPeriodic::setEdgesMode() {
    randomMode = false;
    if(!isSuspended())
        suspend();
    sendAttentionToPort(ATTENTION_MODES::EDGES);
}

void topDownAttentionPeriodic::setMotionMode() {
    randomMode = false;
    if(!isSuspended())
        suspend();
    sendAttentionToPort(ATTENTION_MODES::MOTION);
}

void topDownAttentionPeriodic::setOrientationMode() {
    randomMode = false;
    if(!isSuspended())
        suspend();
    sendAttentionToPort(ATTENTION_MODES::ORIENTATION);
}

void topDownAttentionPeriodic::setFaceMode() {
    randomMode = false;
    if(!isSuspended())
        suspend();
    sendAttentionToPort(ATTENTION_MODES::FACE);
}

void topDownAttentionPeriodic::setBioMotioMode()  {
    randomMode = false;
    if(!isSuspended())
        suspend();
    sendAttentionToPort(ATTENTION_MODES::BIO_MOTION);
}

void topDownAttentionPeriodic::setAudioMode()  {
    randomMode = false;
    if(!isSuspended())
        suspend();
    sendAttentionToPort(ATTENTION_MODES::AUDIO);
}
