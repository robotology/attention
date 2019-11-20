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


#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_KBU VOCAB3('k','b','u') //weight of the bottom-up algorithm
#define COMMAND_VOCAB_KTD VOCAB3('k','t','d') //weight of top-down algorithm
#define COMMAND_VOCAB_RIN VOCAB3('r','i','n') //red intensity value
#define COMMAND_VOCAB_GIN VOCAB3('g','i','n') //green intensity value
#define COMMAND_VOCAB_BIN VOCAB3('b','i','n') //blue intensity value
#define COMMAND_VOCAB_K1 VOCAB2('k','1')
#define COMMAND_VOCAB_K2 VOCAB2('k','2')
#define COMMAND_VOCAB_K3 VOCAB2('k','3')
#define COMMAND_VOCAB_K4 VOCAB2('k','4')
#define COMMAND_VOCAB_K5 VOCAB2('k','5')
#define COMMAND_VOCAB_K6 VOCAB2('k','6')
#define COMMAND_VOCAB_KC1 VOCAB3('k','c','1')
#define COMMAND_VOCAB_KC2 VOCAB3('k','c','2')
#define COMMAND_VOCAB_KC3 VOCAB3('k','c','3')
#define COMMAND_VOCAB_KC4 VOCAB3('k','c','4')
#define COMMAND_VOCAB_KC5 VOCAB3('k','c','5')
#define COMMAND_VOCAB_KC6 VOCAB3('k','c','6')
#define COMMAND_VOCAB_KMOT VOCAB4('k','m','o','t')



topDownAttentionPeriodic::topDownAttentionPeriodic(double p,string client_name, string server_name):PeriodicThread(p){
    msgCount=0;
    this->client_name = client_name;
    this->server_name = server_name;
}

void topDownAttentionPeriodic::afterStart(bool s)
{
    if (s)
        printf("Thread started successfully\n");
    else
        printf("Thread did not start\n");
}

bool topDownAttentionPeriodic::threadInit()
{
  port.open(client_name);
  printf("Starting thread\n");
  return true;
}
void topDownAttentionPeriodic::run()
{
    sendAttentionToPoer();
}
void topDownAttentionPeriodic::threadRelease()
{
    printf("Goodbye from thread1\n");
}

void topDownAttentionPeriodic::sendAttentionToPoer(){
  if (port.getOutputCount()==0) {
      printf("Trying to connect to %s\n", server_name.c_str());
    yarp.connect(client_name,server_name);
  } else {
    Bottle cmd;
     // cmd.addString("Attention");
     // cmd.addInt32(msgCount);
     // cmd.addVocab(COMMAND_VOCAB_SET);
     // msgCount++;
    cmd = createSetVocabBottle(COMMAND_VOCAB_KTD,5);
    printf("Sending message... ");
    helperFunctions::printBottle(cmd);
    Bottle response;
    port.write(cmd,response);
    printf("Got response: %s\n", response.toString().c_str());
  }
}

Bottle topDownAttentionPeriodic::createSetVocabBottle(NetInt32 vocab ,double value){
  Bottle cmd;
  cmd.addString("Attention");
  cmd.addVocab(COMMAND_VOCAB_SET);
  cmd.addVocab(vocab);
  cmd.addDouble(value);
  return cmd;
}
