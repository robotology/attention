// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ali Paikan
 * email:  ali.paikan@iit.it
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
#include <stdio.h>
#include <yarp/os/Network.h>


#include "Detector.h"
#include <yarp/os/RFModule.h>

using namespace yarp::os;

class DetectorModule: public RFModule
{
   Detector* detector;
   yarp::os::Port handlerPort; 

public:

    DetectorModule()
    {
        
        detector = new Detector();
    }

    ~DetectorModule()
    {
        delete detector;        
    }


    bool configure(ResourceFinder &rf)
    {
        ConstString cascade;
        ConstString nestedCascade; 
        if(!rf.check("cascade") /*|| !rf.check("nested-cascade")*/)
        {
            yError("Could not find the cascade file in the parameters.");
            return false;
        }

        detector->strCascade = rf.getContextPath() + "/" + rf.find("cascade").asString();
        printf("cascade: %s\n", detector->strCascade.c_str());
        //detector->strNestedCascade= rf.find("nested-cascade").asString();

        /* 
        * attach a port of the same name as the module (not prefixed with a /) to the module
        * so tha    t messages received from the port are redirected to the respond method
        */  
        std::string handlerPortName =  "/faceDetector/control/rpc";
        //handlerPortName += getName();         // use getName() rather than a literal 

        if (!handlerPort.open(handlerPortName.c_str())) {           
            std::cout << getName() << ": Unable to open port " << handlerPortName << std::endl;  
            return false;
        }
        attach(handlerPort);                  // attach to port

        return detector->open(rf);
    }

    double getPeriod()
    {
        return 0.01;
    }
    
    bool updateModule()
    { 
        detector->loop();
        return true; 
    }

    bool interruptModule()
    {
        fprintf(stderr, "Interrupting\n");
        detector->interrupt();
        handlerPort.interrupt();
        return true;
    }

    bool close()
    {
        fprintf(stderr, "Calling close\n");
        detector->close();
        handlerPort.close();
        return true;
    }

    bool respond(const Bottle& command, Bottle& reply) {
    std::string helpMessage =  std::string(getName().c_str()) + 
        " commands are: \n" +  
        "help \n" + 
        "quit \n";

    reply.clear(); 

    if (command.get(0).asString()=="quit") {
        reply.addString("quitting");
        return false;     
    }
    else if (command.get(0).asString()=="help") {
        std::cout << helpMessage;
        reply.addString("ok");
    }
    else if ((command.get(0).asString()=="sus") || (command.get(0).asString()=="\"sus\"")) {
        
        detector->suspend();
        reply.addString("ok");
    }
    else if (command.get(0).asString()=="res" || command.get(0).asString()=="\"res\"" ) {
        detector->resume();
        reply.addString("ok");
    }
    
    return true;
}


};

int main(int argc, char *argv[]) 
{
    Network yarp;
    //YARP_REGISTER_DEVICES(icubmod)

    DetectorModule module;
    ResourceFinder rf;
    rf.setVerbose();
	rf.setDefaultConfigFile("face_detector.ini");
    rf.setDefaultContext("faceDetector/conf");
    rf.configure(argc, argv);
   
    if (!module.configure(rf))
    {
        fprintf(stderr, "Error configuring module returning\n");
        return -1;
    }
    
    module.runModule();

    printf("Module shutting down\n");

    return 0;
}
