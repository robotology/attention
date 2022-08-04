// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <math.h> 
#include <yarp/os/Network.h>
#include <yarp/os/all.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/GazeControl.h>

#include <string>

const int32_t COMMAND_VOCAB_ON    = yarp::os::createVocab32('o','n')
const int32_t COMMAND_VOCAB_OFF   = yarp::os::createVocab32('o','f','f')
const int32_t COMMAND_VOCAB_DUMP  VOCAB4('d','u','m','p')
const int32_t COMMAND_VOCAB_SYNC  VOCAB4('s','y','n','c')

#define DELTAENC 0.0000001
#define deg2rad  3.1415/180

using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

YARP_DECLARE_DEVICES(icubmod)

void moveJoints(IPositionControl *_pos, Vector& _command)
{
    _pos->positionMove(_command.data());
    Time::delay(0.1);
}

int main(int argc, char *argv[]) 
{
    Network yarp;
    YARP_REGISTER_DEVICES(icubmod)

    //-----------------------------------------------------------------------

    //initializing gazecontrollerclient
    printf("initialising gazeControllerClient \n");
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    std::string localCon("/client/gaze");
    localCon.append("simpleSaccade");
    option.put("local",localCon.c_str());

    yarp::dev::PolyDriver* clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    yarp::dev::IGazeControl* igaze=NULL;

    int originalContext;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    else {
        return false;
    }
    igaze->storeContext(&originalContext);
    igaze->blockNeckPitch(0);
    igaze->blockNeckRoll();
    
    //------------------------------------------------------------------------
    
    BufferedPort<Bottle>* _pInPort  = new BufferedPort<Bottle>;
    Port* _pOutPort = new Port;
    //_options.portName+="/command:o";
    std::string portName="/simpleSaccade/cmd:o";
    std::string portNameIn = "/simpleSaccade/cmd:i";
    _pOutPort->open(portName.c_str());
    _pInPort->open(portNameIn.c_str());

    Property params;
    params.fromCommand(argc, argv);
    if(params.check("help"))
    {
        fprintf(stderr, "%s --robot robotName --loop numberOfLoop", argv[0]);
    }
    /*    
    if (!params.check("robot"))
    {
        fprintf(stderr, "Please specify the name of the robot\n");
        fprintf(stderr, "--robot name (e.g. icub)\n");
        return -1;
    }
    if (!params.check("loop"))
    {
        fprintf(stderr, "Please specify the number of repetition\n");
        fprintf(stderr, "--loop number\n");
        return -1;
    }
    */

    
    std::string robotName = params.check("robot", 
                                       Value("icub"), 
                                       "robotname").asString();
    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/head"; //"/right_arm"

    //int nOl=atoi(params.find("loop").asString().c_str());
    int nOl=params.find("loop").asInt16();

    //Network::connect(portName.c_str(), "/aexGrabber");

    std::string localPorts="/test/client";

    Property options;
    options.put("device", "remote_controlboard");
    options.put("local", localPorts.c_str());   //local port names
    options.put("remote", remotePorts.c_str());         //where we connect to

    // create a device
    PolyDriver robotDevice(options);
    if (!robotDevice.isValid()) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
        return 0;
    }

    IPositionControl *pos;
    IEncoders *encs;

    bool ok;
    ok = robotDevice.view(pos);
    ok = ok && robotDevice.view(encs);

    if (!ok) {
        printf("Problems acquiring interfaces\n");
        return 0;
    }

    int nj=0;
    pos->getAxes(&nj);
    Vector encoders;
    Vector command;
    Vector tmp;
    encoders.resize(nj);
    tmp.resize(nj);
    command.resize(nj);
    
    int i;
    for (i = 0; i < nj; i++) {
         tmp[i] = 90.0;
    }
    pos->setRefAccelerations(tmp.data());

    for (i = 0; i < nj; i++) {
        tmp[i] = 50.0;
        pos->setRefSpeed(i, tmp[i]);
    }

    //pos->setRefSpeeds(tmp.data()))
    
    //fisrst zero all joints
    //
    for(i=0; i<nj; i++)
        command[i]=0;
    /******************************
    * SPECIFIC STARTING POSITIONS *
    ******************************/
    command[0]=0;

    //pos->positionMove(command.data());//(4,deg);
    double startPos2;
    double startPos3;
    double startPos4;

    encs->getEncoder(3, &startPos3);
    encs->getEncoder(4, &startPos4);
    printf("start position value of joint 3: %lf\n", startPos3);
    printf("start position value of joint 4: %lf\n", startPos4);
    bool first=true;
    int deltaSacc=2;
    int times=0;
    yarp::os::Bottle bot; //= _pOutPort->prepare();
    bot.clear();
    bot.addVocab32(COMMAND_VOCAB_DUMP);
    bot.addVocab32(COMMAND_VOCAB_ON);
    Bottle inOn;
    _pOutPort->write(bot,inOn);

    Time::delay(0.1);
    
    fprintf(stderr, "Start saccade(s), number of repetition: %d\n", nOl);
    /*
    while(times<nOl)
    {
        times++;
        
	
	command[4]=-deltaSacc;
	//moveJoints(pos, command);
    pos->positionMove(command.data());
    if(first)
    {
        double curPos;
        encs->getEncoder(4, &curPos);
    	printf("current position value of joint 4: %lf\n", curPos);
        while((curPos>=startPos4-DELTAENC) && (curPos<=startPos4+DELTAENC))
        {
    	    printf("current position value of joint 4: %lf\n", curPos);
            encs->getEncoder(4, &curPos);
        }
        bot.clear();
        bot.addVocab32(COMMAND_VOCAB_SYNC);
        Bottle inStart;
        _pOutPort->write(bot,inStart);
    	printf("1st synch asked\n");
        first=false;
    }
    Time::delay(0.1);
    */

    /*
	
	command[4]=deltaSacc;
	moveJoints(pos, command);	
	command[4]=0;
	moveJoints(pos, command);
    */

	/*
	command[3]=-deltaSacc;
	moveJoints(pos, command);
		command[3]=deltaSacc;
	moveJoints(pos, command);
	command[3]=0;
	moveJoints(pos, command);
    if(times>=nOl)
    {
        double curPos;
        encs->getEncoder(3, &curPos);
    	printf("current position value of joint 3: %lf\n", curPos);
        //while((curPos<startPos3-DELTAENC) || (curPos>startPos3+DELTAENC))
        //{
	    //    command[3]=0;
    	//    printf("current position value of joint 3: %lf\n", curPos);
        //    encs->getEncoder(3, &curPos);
        //}
        bot.clear();
        bot.addVocab32(COMMAND_VOCAB_SYNC);
        Bottle inEnd;
        _pOutPort->write(bot,inEnd);
    	printf("2nd synch asked\n");
    }


    }
    */
    
    double value = 0;
    double endPos2;
    double r = 0.5;  //meters
    Vector angles(3);
    angles(0) =  0.0;
    angles(1) =  0.0;
    angles(2) =  0.0;
    Vector position(3);
    position(0) = -0.5;
    position(1) =  0.0;
    position(2) =  0.4;

    while(true){
        if (_pInPort->getInputCount()) {
            Bottle* b = _pInPort->read(true);
            value = b->get(0).asFloat32();
            printf("got the double %f \n", value);
            encs->getEncoder(2, &startPos2);
            //printf("getEncoder3 position %f \n", startPos2);
            //if ((value+startPos2 < 50) && (value+startPos2 > -50)){
                endPos2 = startPos2 - value ;

                command[2] = endPos2;
                //moveJoints(pos, command);

                // iKinGazeCtrl convention: positive angles toward robot right hand side
                position(0) = -1 * (cos(deg2rad * endPos2) * r);
                position(1) = -1 * (sin(deg2rad * endPos2) * r);
                printf("sending vector %s \n", position.toString().c_str());
                igaze->lookAtFixationPoint(position);

                
                //angles(0) = value;
                //printf("sending vector %s \n", angles.toString().c_str());
                //igaze->lookAtRelAngles(angles);
                //}
                //else{
                //if (value+startPos2 > 50) {
                //    command[2] = 50;
                    //moveJoints(pos, command);
                    
                //}
                //else {
                //    command[2] = -50;
                    //moveJoints(pos, command);   
                //}
                //}
        }  
    }

	Time::delay(0.1);


    /*bot.clear();
    bot.addVocab32(COMMAND_VOCAB_SYNC);
    Bottle inEnd;
    _pOutPort->write(bot,inEnd);
    */


    bot.clear();
    bot.addVocab32(COMMAND_VOCAB_DUMP);
    bot.addVocab32(COMMAND_VOCAB_OFF);
    Bottle inOff;
    _pOutPort->write(bot,inOff);

    _pOutPort->close();
    robotDevice.close();
    //-------------------------------------------

    igaze->restoreContext(originalContext);
    delete igaze;
    
    
    return 0;
}
