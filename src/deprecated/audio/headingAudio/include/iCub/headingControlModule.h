// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2014 Robotics, Brain and Cognitive Science (RBCS)
 * Author: Rea Francesco
 * email:  francesco.rea@iit.it
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
\defgroup headingControl headingControl
 
@ingroup icub_audio_attention  
 
The manager module for the Joint Grasping Demo developed by IIT 
and ISR. 

Copyright (C) 2010 RobotCub Consortium
 
Author: Rea Francesco 

CopyPolicy: Released under the terms of the GNU GPL v2.0.

\section intro_sec Description
This module collects the 3-d object positions estimated by the 
particle filter and sends data to the head and arm controllers 
in order to gaze at the target, reach for it and eventually 
grasp it. 
It relies on the YARP ICartesianControl interface to control 
both arms and on the YARP IGazeControl interface to control the 
gaze. 
 
Furthermore, there exists a second modality that enables to 
estimate the 3-d object position usin-g stereo vision that needs 
to be calibrated in advance relying on a feed-forward neural 
network. 
 
\section lib_sec Libraries 
- ctrlLib. 
- iKin.  
- YARP libraries. 

\section parameters_sec Parameters
None. 
 
\section portsa_sec Ports Accessed 
The robot interface is assumed to be operative; in particular, 
the ICartesianControl interface must be available. The 
\ref iKinGazeCtrl must be running.
 
\section portsc_sec Ports Created 
 
- \e /demoGraspManager_IIT_ISR/trackTarget:i receives the 3-d 
  position to track.
 
- \e /demoGraspManager_IIT_ISR/imdTargetLeft:i receives the 
  blobs list as produced by the \ref motionCUT module for the
  left eye.
 
- \e /demoGraspManager_IIT_ISR/imdTargetRight:i receives the 
  blobs list as produced by the \ref motionCUT module for the
  right eye.
 
- \e /demoGraspManager_IIT_ISR/cmdFace:o sends out commands to 
  the face expression high level interface in order to give an
  emotional representation of the current robot state.
 
- \e /demoGraspManager_IIT_ISR/gui:o sends out info to update target
  within the \ref icub_gui.

- \e /demoGraspManager_IIT_ISR/rpc remote procedure 
    call. Recognized remote commands:
    -'quit' quit the module
 
\section in_files_sec Input Data Files
None.

\section out_data_sec Output Data Files 
None. 
 
\section conf_file_sec Configuration Files
The configuration file passed through the option \e --from
should look like as follows:
 
\code 
[general]
// the robot name to connect to 
robot           icub
// the thread period [ms] 
thread_period   30
// left arm switch 
left_arm        on 
// right arm switch 
right_arm       on 
// arm trajectory execution time [s]
traj_time       2.0 
// reaching tolerance [m]
reach_tol       0.01 
// eye used 
eye             left 
// homes limbs if target detection timeout expires [s]
idle_tmo        5.0 
// enable the use of stereo vision calibrated by NN 
use_network off 
// NN configuration file 
network         network.ini 

[torso] 
// joint switch (min **) (max **) [deg]; 'min', 'max' optional 
pitch on  (max 30.0) 
roll off 
yaw on

[left_arm]
// the offset [m] to be added to the desired position  
reach_offset        0.0 -0.15 -0.05
// the offset [m] for grasping 
grasp_offset        0.0 0.0 -0.05
// perturbation given as standard deviation [m] 
grasp_sigma 0.01 0.01 0.01 
// hand orientation to be kept [axis-angle rep.] 
hand_orientation 0.064485 0.707066 0.704201 3.140572 
// enable impedance velocity mode 
impedance_velocity_mode off 
impedance_stiffness 0.5 0.5 0.5 0.2 0.1 
impedance_damping 60.0 60.0 60.0 20.0 0.0 

[right_arm]
reach_offset        0.0 0.15 -0.05
grasp_offset        0.0 0.0 -0.05
grasp_sigma	        0.01 0.01 0.01
hand_orientation    -0.012968 -0.721210 0.692595 2.917075
impedance_velocity_mode off 
impedance_stiffness 0.5 0.5 0.5 0.2 0.1 
impedance_damping 60.0 60.0 60.0 20.0 0.0 
 
[home_arm]
// home position [deg] 
poss    -30.0 30.0 0.0  45.0 0.0  0.0  0.0
// velocities to reach home positions [deg/s] 
vels    10.0  10.0 10.0 10.0 10.0 10.0 10.0

[arm_selection]
// hysteresis range added around plane y=0 [m]
hysteresis_thres 0.1

[grasp]
// ball radius [m] for still target detection 
sphere_radius   0.05 
// timeout [s] for still target detection 
sphere_tmo      3.0 
// timeout [s] to open hand after closure 
release_tmo     3.0 
// open hand positions [deg] 
open_hand       0.0 0.0 0.0   0.0   0.0 0.0 0.0   0.0   0.0 
// close hand positions [deg] 
close_hand      0.0 80.0 12.0 18.0 27.0 50.0 20.0  50.0 135.0 
// velocities to reach hand positions [deg/s] 
vels_hand       10.0 10.0  10.0 10.0 10.0 10.0 10.0 10.0  10.0 
\endcode 

\section tested_os_sec Tested OS
Windows, Linux

\author Rea Francesco
*/ 


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
#include <iCub/managerThread.h>

#include <string>

#define COMMAND_VOCAB_ON    VOCAB2('o','n')
#define COMMAND_VOCAB_OFF   VOCAB3('o','f','f')
#define COMMAND_VOCAB_DUMP  VOCAB4('d','u','m','p')
#define COMMAND_VOCAB_SYNC  VOCAB4('s','y','n','c')

#define DELTAENC 0.0000001
#define deg2rad  3.1415/180

// general command vocab's
#define COMMAND_VOCAB_IS     VOCAB2('i','s')
#define COMMAND_VOCAB_OK     VOCAB2('o','k')

#define COMMAND_VOCAB_HELP   VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_POINT  VOCAB4('p','o','i','n')
#define COMMAND_VOCAB_LOOK   VOCAB4('l','o','o','k')
#define COMMAND_VOCAB_FAILED VOCAB4('f','a','i','l')
#define COMMAND_VOCAB_TRED   VOCAB4('t','r','e','d')
#define COMMAND_VOCAB_TGRE   VOCAB4('t','g','r','e')
#define COMMAND_VOCAB_TBLU   VOCAB4('t','b','l','u')
#define COMMAND_VOCAB_FRED   VOCAB4('f','r','e','d')       // request of fovea blob color (red)
#define COMMAND_VOCAB_FBLU   VOCAB4('f','b','l','u')       // request of fovea blob color (red)
#define COMMAND_VOCAB_FGRE   VOCAB4('f','g','r','e')       // request of fovea blob color (red)
#define COMMAND_VOCAB_FRGB   VOCAB4('f','r','g','b')       // request of fovea blob color (rgb)


#define COMMAND_VOCAB_MAXDB  VOCAB3('M','d','b')           // maximum dimension of the blob drawn
#define COMMAND_VOCAB_MINDB  VOCAB3('m','d','b')           // minimum dimension of the blob drawn
#define COMMAND_VOCAB_MBA    VOCAB3('m','B','A')           // minimum dimension of the bounding area
#define COMMAND_VOCAB_SET    VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET    VOCAB3('g','e','t')
#define COMMAND_VOCAB_WTD    VOCAB3('w','t','d')
#define COMMAND_VOCAB_WBU    VOCAB3('w','b','u')


using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::os;

YARP_DECLARE_DEVICES(icubmod)

void moveJoints(IPositionControl *_pos, Vector& _command)
{
    _pos->positionMove(_command.data());
    Time::delay(0.1);
}




/*
int main(int argc, char *argv[]) 
{
    Network yarp;
    YARP_REGISTER_DEVICES(icubmod)

    //------------------------------------------------------------------------------------
    // initializing the iKinCartesian Solver
    ResourceFinder rf;
    rf.setDefaultContext("demoGrasp_IIT_ISR");
    rf.setDefaultConfigFile("config.ini");
    managerThread* mt = new managerThread("/managerThread",rf);
    mt->start();

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
        
    //if (!params.check("robot"))
    //{
    //    fprintf(stderr, "Please specify the name of the robot\n");
    //    fprintf(stderr, "--robot name (e.g. icub)\n");
    //    return -1;
    //}
    //if (!params.check("loop"))
    //{
    //    fprintf(stderr, "Please specify the number of repetition\n");
    //    fprintf(stderr, "--loop number\n");
    //    return -1;
    //}
    

    
    std::string robotName = params.check("robot", 
                                       Value("icub"), 
                                       "robotname").asString();
    std::string remotePorts="/";
    remotePorts+=robotName;
    remotePorts+="/head"; //"/right_arm"

    //int nOl=atoi(params.find("loop").asString().c_str());
    int nOl=params.find("loop").asInt();

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
    //******************************
    //* SPECIFIC STARTING POSITIONS *
    //******************************
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
    bot.addVocab(COMMAND_VOCAB_DUMP);
    bot.addVocab(COMMAND_VOCAB_ON);
    Bottle inOn;
    _pOutPort->write(bot,inOn);

    Time::delay(0.1);
    
    fprintf(stderr, "Start saccade(s), number of repetition: %d\n", nOl);
    /*
   //wile(times<nOl)
    //
    //  times++;
    //  
	//
	//command[4]=-deltaSacc;
	//moveJoints(pos, command);
    //pos->positionMove(command.data());
    //if(first)
    //{
    //    double curPos;
    //    encs->getEncoder(4, &curPos);
    //	printf("current position value of joint 4: %lf\n", curPos);
    //    while((curPos>=startPos4-DELTAENC) && (curPos<=startPos4+DELTAENC))
    //    {
    //	    printf("current position value of joint 4: %lf\n", curPos);
    //        encs->getEncoder(4, &curPos);
     //   }
    //    bot.clear();
    //    bot.addVocab(COMMAND_VOCAB_SYNC);
    //    Bottle inStart;
    //    _pOutPort->write(bot,inStart);
   // 	printf("1st synch asked\n");
    //    first=false;
    //}
    //Time::delay(0.1);
    

    
	
	//command[4]=deltaSacc;
	//moveJoints(pos, command);	
	//command[4]=0;
	//moveJoints(pos, command);
    

	
	//command[3]=-deltaSacc;
	//moveJoints(pos, command);
	//	command[3]=deltaSacc;
	//moveJoints(pos, command);
	//command[3]=0;
	//moveJoints(pos, command);
    //if(times>=nOl)
    //{
    //    double curPos;
    //    encs->getEncoder(3, &curPos);
    // 	printf("current position value of joint 3: %lf\n", curPos);
    //    //while((curPos<startPos3-DELTAENC) || (curPos>startPos3+DELTAENC))
    //    //{
	//    //    command[3]=0;
    // 	//    printf("current position value of joint 3: %lf\n", curPos);
    //    //    encs->getEncoder(3, &curPos);
    //    //}
    //    bot.clear();
    //    bot.addVocab(COMMAND_VOCAB_SYNC);
    //    Bottle inEnd;
    //    _pOutPort->write(bot,inEnd);
    //	printf("2nd synch asked\n");
    //}
    //
    //
    //}
    
    
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
            value = b->get(0).asDouble();
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


    //bot.clear();
    //bot.addVocab(COMMAND_VOCAB_SYNC);
    //Bottle inEnd;
    //_pOutPort->write(bot,inEnd);
    

    bot.clear();
    bot.addVocab(COMMAND_VOCAB_DUMP);
    bot.addVocab(COMMAND_VOCAB_OFF);
    Bottle inOff;
    _pOutPort->write(bot,inOff);

    _pOutPort->close();
    robotDevice.close();
    //-------------------------------------------

    mt->stop();

    igaze->restoreContext(originalContext);
    delete igaze;
    
    
    return 0;
}
*/


class managerModule: public RFModule
{
protected:
    managerThread *thr;    
    Port           rpcPort;
    Semaphore      mutex;

public:
    managerModule() { }

    bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();            

        thr=new managerThread(getName().c_str(),rf);
        if (!thr->start())
        {
            delete thr;    
            return false;
        }

        rpcPort.open(getName("/rpc"));
        attach(rpcPort);

        return true;
    }

    bool close()
    {
        rpcPort.interrupt();
        rpcPort.close();

        thr->stop();
        delete thr;

        return true;
    }

    bool respond(const Bottle &command, Bottle &reply) {
        // 
        bool ok = false;
        bool rec = false; // is the command recognized?
        
        mutex.wait();
        
        switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_HELP:
            rec = true;
            {
                reply.addVocab(Vocab::encode("many"));
                reply.addString("help");
                
                //reply.addString("\n");
                reply.addString("point n \t: general point command");
                //reply.addString("\n");
                reply.addString("look  \t: general look command ");
                //reply.addString("\n");
                reply.addString("NOTE: capitalization of command name is mandatory");
                reply.addString("set Mdb : set maximum dimension allowed for blobs");
                reply.addString("set mdb : set minimum dimension allowed for blobs");
                reply.addString("set mBA : set the minimum bounding area");
                //reply.addString("\n");
                reply.addString("get Mdb : get maximum dimension allowed for blobs");
                reply.addString("get mdb : get minimum dimension allowed for blobs");
                reply.addString("get mBA : get the minimum bounding area\n");
                ok = true;
            }
            break;
            
        case COMMAND_VOCAB_LOOK:
            rec = true;
            {
                printf("*** LOOK command received \n");                
                thr->setAction("look");
                /*
                switch(command.get(1).asVocab()) {
                case COMMAND_VOCAB_MBA:
                    {
                        double w = command.get(2).asDouble();
                        cout << "set mBA: " << w << endl;
 
                        ok=true;
                    }
                    break;
                case COMMAND_VOCAB_MAXDB:
                    {
                        int w = command.get(2).asInt();
                        
                        ok=true;
                    }
                    break;
                case COMMAND_VOCAB_MINDB:
                    {
                        int w = command.get(2).asInt();
     
                        ok=true;
                    }
                    break;
                case COMMAND_VOCAB_WTD:
                    {
                        int w = command.get(2).asDouble();
    
                        ok=true;
                    }
                    break;
                case COMMAND_VOCAB_WBU:
                    {
                        int w = command.get(2).asDouble();
                       
                        ok=true;
                    }
                    break;
                case COMMAND_VOCAB_TRED:
                    {
                        int t = command.get(2).asInt();
                        
                        ok=true;
                    }
                    break;
                case COMMAND_VOCAB_TGRE:
                    {
                        int t = command.get(2).asInt();
                        
                        ok=true;
                    }
                    break;
                case COMMAND_VOCAB_TBLU:
                    {
                        int t = command.get(2).asInt();
                        
                        ok=true;
                    }
                    break;
                default:
                    cout << "received an unknown request after a SALIENCE_VOCAB_SET" << endl;
                    break;
                    }*/

                 ok = true;

            }
            break;
            
        case COMMAND_VOCAB_POINT:
            rec = true;
            {
                printf("*** POINT command received \n");
                thr->setAction("point");
                
                //reply.addVocab(COMMAND_VOCAB_IS);
                //reply.add(command.get(1));
                /*switch(command.get(1).asVocab()) {
                    
                case COMMAND_VOCAB_MAXDB:
                    {
                        
                        //reply.addInt(nb);
                        ok = true;
                    }
                    break;
                case COMMAND_VOCAB_MINDB:
                    {
                        
                        //reply.addInt(nb);
                        ok = true;
                    }
                    break;
                case COMMAND_VOCAB_FRGB:
                    {
                        int redValue,greenValue, blueValue;
                        
                        reply.addInt(redValue);
                        reply.addInt(greenValue);
                        reply.addInt(blueValue);
                        ok = true;
                    }
                    break;
                    
            
                   
                    
                default:
                    cout << "received an unknown request after a SALIENCE_VOCAB_GET" << endl;
                    break;
                    }*/

                ok = true;
                
            }
            break;
            
        }
        mutex.post();
        
        if (!rec)
            ok = RFModule::respond(command,reply);
        
        if (!ok) {
            reply.clear();
            reply.addVocab(COMMAND_VOCAB_FAILED);
        }
        else
            reply.addVocab(COMMAND_VOCAB_OK);
        
        return ok;
    } 	
    
    double getPeriod()    { return 1.0;  }
    bool   updateModule() { return true; }
};
