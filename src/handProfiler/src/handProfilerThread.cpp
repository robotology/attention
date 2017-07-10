// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
  * Copyright (C)2015  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/

/**
 * @file handProfilerThread.cpp
 * @brief Implementation of the handProfiler thread (see handProfilerThread.h).
 */

#include <iCub/handProfilerThread.h>
#include <cstring>

#define CTRL_THREAD_PER     0.02    // [s]
#define PRINT_STATUS_PER    1.0     // [s]
#define MAX_TORSO_PITCH     10.0    // [deg]
#define MIN_TORSO_YAW       -40.0   // [deg]
#define MAX_TORSO_YAW       40.0    // [deg]
#define RATETHREAD          5       // [ms]
#define TRAJTIME            0.5     // [s]
#define GAZEINTERVAL        20

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace std;
using namespace profileFactory;

MotionProfile* factoryCVMotionProfile(const Bottle &param){
    CVMotionProfile *cvmp = new CVMotionProfile(param);
    if(!cvmp->isValid()){
        yError("factory ERROR");
        delete cvmp;
        return NULL;
    }
    else {
        return static_cast<MotionProfile*>(cvmp);
    }
}

MotionProfile* factoryMJMotionProfile(const Bottle &param){
    MJMotionProfile *mjmp = new MJMotionProfile(param);
    if(!mjmp->isValid()){
        yError("factory ERROR");
        delete mjmp;
        return NULL;
    }
    else {
        return static_cast<MotionProfile*>(mjmp);
    }
}

MotionProfile* factoryTTPLMotionProfile(const Bottle &param){
    TTPLMotionProfile *ttplmp = new TTPLMotionProfile(param);
    if(ttplmp==NULL){
        yError("factory ERROR: NULL pointer");
        return NULL;
    }
    if(!ttplmp->isValid()){
        yError("factory ERROR: not valid profile");
        //delete ttplmp;
        yDebug("deleting the invalid profile");
        return NULL;
    }
    else {
        return static_cast<MotionProfile*>(ttplmp);
    }
}

MotionProfile* factoryTwoThirdMotionProfile(const Bottle &param){
    TwoThirdMotionProfile *ttplmp = new TwoThirdMotionProfile(param);
    if(ttplmp==NULL){
        yError("factory ERROR: NULL pointer");
        return NULL;
    }
    if(!ttplmp->isValid()){
        yError("factory ERROR: not valid profile");
        //delete ttplmp;
        yDebug("deleting the invalid profile");
        return NULL;
    }
    else {
        return static_cast<MotionProfile*>(ttplmp);
    }
}
//*************************************************************************************************//

handProfilerThread::handProfilerThread(): RateThread(RATETHREAD) {
    robot = "icub";
    icart = 0;
    count = 0;
    firstIteration = true;
    idle = true;
    state = none;
    fileCounter = 0;
    timestamp = new yarp::os::Stamp(0,0);
    gazetracking = false;
    saveOn = false;
    speedFactor = 1.0;
    partnerTime = 0;
    infoSamples = 0;
    firstDuration = 0;
    repsNumber = 1;
    firstPos.resize(3);
    firstOri.resize(4);
    // we want to raise an event each time the arm is at 20%
    // of the trajectory (or 70% far from the target)
    cartesianEventParameters.type="motion-ongoing";
    cartesianEventParameters.motionOngoingCheckPoint=0.2;
}


handProfilerThread::handProfilerThread(string _robot, string _configFile, ResourceFinder rf): RateThread(RATETHREAD){
    robot = _robot;
    configFile = _configFile;
    icart = 0;
    count = 0;
    firstIteration = true;
    idle = true;
    state = none;
    fileCounter = 1;
    timestamp = new Stamp(0,0);
    verbosity = true;
    gazetracking = false;
    saveOn = false;
    speedFactor = 1.0;
    partnerTime = 0;
    infoSamples = 0;
    firstDuration = 0;
    repsNumber = 1;
    this->rf = rf;
    firstPos.resize(3);
    firstOri.resize(4);
    // we wanna raise an event each time the arm is at 20%
    // of the trajectory (or 70% far from the target)
    cartesianEventParameters.type="motion-ongoing";
    cartesianEventParameters.motionOngoingCheckPoint=0.2;

}



handProfilerThread::~handProfilerThread() {
    // do nothing
}


bool handProfilerThread::threadInit() {


    /* set the cartesian conntroller for movement */
    Property optionCartesian("(device cartesiancontrollerclient)");
    string str("/");
    str.append(robot);
    str.append("/cartesianController/left_arm");
    yDebug("remote: %s", str.c_str());
    optionCartesian.put("remote","/" +  robot + "/cartesianController/left_arm");
    optionCartesian.put("local","/handProfiler/left_arm");

    if (!client.open(optionCartesian)) {
        yInfo("Client not available. Proceeding to pure imagination action performance ");
        icart = 0;
    }
    else {
        yInfo("preparing the icart");
        // open the view
        client.view(icart);

        // latch the controller context in order to preserve
        // it after closing the module
        // the context contains the dofs status, the tracking mode,
        // the resting positions, the limits and so on.
        icart->storeContext(&startup_context_id);

        // set trajectory time
        icart->setTrajTime(1.0);

        // get the torso dofs
        Vector newDof, curDof;
        icart->getDOF(curDof);
        newDof=curDof;

        // enable the torso yaw and pitch
        // disable the torso roll
        if(yawDof == 1) {
            yInfo("yawDof = ON");
            newDof[0] = yawDof;  //1;
        }
        else {
            yInfo("yawDof = OFF");
            newDof[0] = 0;
        }
        if(rollDof == 1) {
            yInfo("rollDof = ON");
            newDof[1]=rollDof; //0;
        }
        else {
            yInfo("rollDof = OFF");
            newDof[1] = 0; //0;
        }
        if(pitchDof == 1) {
            yInfo("pitchDof = ON");
            newDof[2]=pitchDof;//1;
        }
        else {
            yInfo("pitchDof = OFF");
            newDof[2] = 0; //1
        }

        // impose some restriction on the torso pitch and yaw
        limitTorsoPitch();
        limitTorsoYaw();

        // send the request for dofs reconfiguration
        icart->setDOF(newDof,curDof);

        // print out some info about the controller
        Bottle info;
        icart->getInfo(info);
        fprintf(stdout,"info = %s\n",info.toString().c_str());

        // register the event, attaching the callback
        icart->registerEvent(*this);
    }

      /* set the position controller for movement from file */
    Property optionJoints;
    optionJoints.put("device", "remote_controlboard");
    optionJoints.put("local", "/handProfiler/joints");                 //local port names
    optionJoints.put("remote", "/"+ robot+"/left_arm");                        //where we connect to

    if (!robotDevice.open(optionJoints)) {
        printf("Device not available.  Here are the known devices:\n");
        printf("%s", Drivers::factory().toString().c_str());
    }

    bool ok;
    ok = robotDevice.view(encs);
    ok = ok && robotDevice.view(ictrl);
    ok = ok && robotDevice.view(idir);


    if (!ok) {
        yError("Problems acquiring interfaces\n");
        return 0;
    }

    idir->getAxes(&njoints);
    //yDebug("njoints = %d", njoints);

    /*
    //initializing gazecontrollerclient
    printf("initialising gazeControllerClient \n");
    Property optionGaze;
    optionGaze.put("device","gazecontrollerclient");
    optionGaze.put("remote","/iKinGazeCtrl");
    string localCon("/handProfiler/gaze");
    localCon.append(getName(""));
    optionGaze.put("local",localCon.c_str());
    yInfo("activating the PolyDriver");

    clientGazeCtrl = new PolyDriver();
    clientGazeCtrl->open(optionGaze);
    igaze = NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
       igaze->storeContext(&originalContext);
       blockNeckPitchValue = -1;
       if(blockNeckPitchValue != -1) {
           igaze->blockNeckPitch(blockNeckPitchValue);
           yInfo("pitch fixed at %d \n",blockNeckPitchValue);
       }
       else {
           yInfo("pitch free to change\n");
       }
    }
    else {
        yInfo("Not Valid clientGazeCtrl");
        igaze = 0;
    }
    yInfo("Success in initialising the gaze");
    */

    string rootNameGui("");
    rootNameGui.append(getName("/gui:o"));
    if(!guiPort.open(rootNameGui.c_str())) {
          yError("guiPort is not open with success. Check for conflicts");
    }
    //string xdNameGui("");
    //xdNameGui.append(getName("/xd:o"));
    if(!xdPort.open("/handProfiler/xd:o")) {
          yError("xdPort is not open with success. Check for conflicts");
    }
    string velName("");
    velName.append(getName("/vel:o"));
    if(!velPort.open(velName.c_str())) {
          yError("velPort is not open with success. Check for conflicts");
    }

    /* initialization of the thread */
    x.resize(3);
    o.resize(4);
    xd.resize(3);
    od.resize(4);
    xdhat.resize(3);
    odhat.resize(4);
    qdhat.resize(4);

    t0 = Time::now();
    //initialization of the relevant vectors
    //od[0] = -0.096; od[1] = 0.513; od[2] = -0.8528; od[3] = 2.514;
    od[0] = -0.076; od[1] = -0.974; od[2] = 0.213; od[3] = 3.03;

    yInfo("handProfiler thread correctly started");

    return true;
}

void handProfilerThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

void handProfilerThread::loadFile(string str) {
    this->fileName=str;

    filePath = rf.findFile(fileName + ".info");           //look for info file
    infoInputFile.open(filePath.c_str());                 //open info file
    yInfo("opening file.....");
    if(infoInputFile.is_open()){
        infoInputFile >> movementDuration;                //save info from file
        infoInputFile >> sampleNumber;
        startPos.resize(3);
        startOri.resize(4);
        for(int i=0; i<3; i++){
          infoInputFile >> startPos(i);
          //yDebug("startpos %d, : %f", i, startPos(i));
        }
        for(int i=0; i<4; i++){
          infoInputFile >> startOri(i);
          //yDebug("startori %d, : %f", i, startOri(i));
        }
        yInfo("duration: %f  number: %d", movementDuration, sampleNumber );
        infoInputFile.close();
    }else{
        yError(".info File not found");
    }
}

void handProfilerThread::setPartnerStart() {
    this->partnerStart = Time::now();
    yDebug("start time: %f", partnerStart);
}

void handProfilerThread::setRepsNumber(int n) {
     this->repsNumber = n;
     yInfo("number of repetitions set to: %d", repsNumber);
}

void handProfilerThread::setPartnerStop() {
    this->partnerStop = Time::now();
    yDebug("stop time: %f", partnerStop);
}

void handProfilerThread::setPartnerTime() {
    this->partnerTime = partnerStop - partnerStart;
    yDebug("sync time: %f", partnerTime);
}

std::string handProfilerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void  handProfilerThread::rotAxisX(const double& angle) {
    Vector Anew, Bnew, Cnew;
    Anew = mp->getA();
    Bnew = mp->getB();
    Cnew = mp->getC();

    // multiplying the vector by the matrix


    mp->setA(Anew);
    mp->setB(Bnew);
    mp->setC(Cnew);
    mp->setViaPoints(Anew, Bnew, Cnew);
}

void  handProfilerThread::rotAxisY(const double& angle) {
    Vector Anew, Bnew, Cnew;
    Anew = mp->getA();
    Bnew = mp->getB();
    Cnew = mp->getC();
    mp->setA(Anew);
    mp->setB(Bnew);
    mp->setC(Cnew);
}

void  handProfilerThread::rotAxisZ(const double& angle) {
    Vector Anew, Bnew, Cnew;
    Anew = mp->getA();
    Bnew = mp->getB();
    Cnew = mp->getC();
    mp->setA(Anew);
    mp->setB(Bnew);
    mp->setC(Cnew);
}

void handProfilerThread::setInputPortName(string InpPort) {

}

bool handProfilerThread::resetExecution(){
    bool result = true;
    Vector xZero(3);
    xZero[0] = -0.3; xZero[1] = -0.1; xZero[2] = 0.1;

    if(0 != icart){
        Vector xInit(3);
        xInit = mp->getInitial();
        yInfo("resetting position to %s", xInit.toString().c_str());
        fprintf(stdout,"xd          [m] = %s\n",xInit.toString().c_str());
        fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
        icart->goToPose(xInit,od);
        if(gazetracking) {
            igaze->lookAtFixationPoint(xInit);
        }

        // we get the current arm pose in the
        // operational space
        icart->getPose(x,o);

        // we get the final destination of the arm
        // as found by the solver: it differs a bit
        // from the desired pose according to the tolerances
        icart->getDesired(xdhat,odhat,qdhat);

        double e_x=norm(xdhat-xInit);
        double e_o=norm(odhat-od);

        fprintf(stdout,"+++++++++\n");
        //fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
        fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
        fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
        fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
        fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
        fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
        fprintf(stdout,"norm(e_x)   [m] = %g\n",e_x);
        fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
        fprintf(stdout,"---------\n\n");

        if (e_x > 0.5) {
            yError("Error in resetting the initial position");
            //result = false;
        }

    }
    idle = true;
    return result;
}

bool handProfilerThread::startExecution(const bool _reverse){
    //count = 0;
    //mp->setReverse(_reverse);
    idle = false;
    state = execution;
    firstIteration = true;
    t0 = Time::now();
	return true;
}

bool handProfilerThread::startSimulation(const bool _reverse){
    //count = 0;
    //mp->setReverse(_reverse);
    idle = false;
    state = simulation;
    firstIteration = true;
    t0 = Time::now();
	return true;
}

bool handProfilerThread::saveJoints(){                                  //save to file
    //count = 0;
    //mp->setReverse(_reverse);
    saveOn = !saveOn;
    t0 = Time::now();
    if(saveOn) yInfo("save to file ENABLED");
    else yInfo("save to file DISABLED");
	return true;
}

bool handProfilerThread::startJoints(double factor = 1.0){              //move from file
    //count = 0;
    //mp->setReverse(_reverse);
    idle = false;
    state = file;
    firstIteration = false;
    t0 = Time::now();
    speedFactor = factor;
	return true;
}



bool handProfilerThread::factory(const string type, const Bottle finalB){

    if (!strcmp(type.c_str(),"CVP")) {
        mp = factoryCVMotionProfile(finalB);
        yDebug("returned from CVP factory");
        if (mp == NULL){
            yError("factory returned error");
            return false;
        }
    }
    else if(!strcmp(type.c_str(),"TTPL")) {
        mp = factoryTTPLMotionProfile(finalB);
        yDebug("returned from TTPL factory");
        if (mp == NULL){
            yError("factory returned error");
            return false;
        }
    }
    else if(!strcmp(type.c_str(),"TwoThird")) {
        mp = factoryTwoThirdMotionProfile(finalB);
        yDebug("returned from TwoThird factory");
        if (mp == NULL){
            yError("factory returned error");
            return false;
        }
    }
    else if(!strcmp(type.c_str(),"MJP")) {
        yDebug("Entering in factory")  ;
        mp = factoryMJMotionProfile(finalB);
        yDebug("returned from MJP factory");
        if (mp == NULL){
            yError("factory returned error");
            return false;
        }
    }
    else{
        yError("Error.Type is unknown.");
        return false;
    }

    bool result = true;
    result = result & resetExecution();
    return result;
}

void handProfilerThread::run() {
    double iniziaqui = Time::now();
    if(!idle) {
        //yInfo("!idle");
        count++;
        if (firstIteration) {
            yInfo("first iteration");                                                           // first itaration set variables
            t = t0;
            mp->setT0(t0);
            firstIteration = false;
            displayProfile();

            //-----file opening to save -------
            if(saveOn){                                                                         // if save is enabled, open files for output and
                ostringstream convert;                                                          // set variables for duration calculation and initial position
                convert << fileCounter;
                string fileToSave = "action_" + convert.str() + ".log";
                outputFile.open(fileToSave.c_str(), std::ofstream::out);
                fileToSave = "action_" + convert.str() + ".info";
                infoOutputFile.open(fileToSave.c_str(), std::ofstream::out);
                firstDuration = Time::now();
                icart->getPose(firstPos, firstOri);
                icart->storeContext(&icartContext);
            }
            //-------------------------------
        }
        else {
            t=Time::now();
        }

        bool success;
        switch (state) {
            case execution:
                success = generateTarget();
                //yDebug("generated target %d", success);
                if(success){
                    icart-> goToPose(xd,od, 0.01);
                    //yDebug("goToPose");
                    if(saveOn)
                        saveToArray();

                    if(gazetracking && (count%GAZEINTERVAL==0)) {
                        igaze->lookAtFixationPoint(xd);
                    }
                }else if(!success && outputFile.is_open() && saveOn){
                    yInfo("file saved");
                    yDebug("cicli: %d ", infoSamples);
                    bool motionDone;
                    icart->checkMotionDone(&motionDone);
                    while(!motionDone){
                        if(saveOn)
                            saveToArray();
                        icart->checkMotionDone(&motionDone);
                        Time::delay(0.005);
                    }
                    saveToFile();
                    saveInfo();                                           //save info to file
                    fileCounter++;                                        //movement finished, if save enabled close the files and reset the state
                    outputFile.close();
                    infoOutputFile.close();
                    firstDuration = 0;
                    infoSamples = 0;
                    state = none;
                    idle = true;
                    icart->restoreContext(icartContext);
                }else{
                    state = none;
                    idle = true;
                    icart->restoreContext(icartContext);
                }
                break;
            case simulation:
                success = generateTarget();
                if(!success){
                    state = none;
                    idle = true;
                }
                break;
            case file:
                yWarning("starting movement from file");
                startFromFile();
                break;
        }


        displayTarget();
        if(xdPort.getOutputCount()) {
            printXd();
        }
        if(velPort.getOutputCount()) {
            printVel();
        }

        // some verbosity
        if(verbosity) {
            //printStatus();
        }

        double tend = Time::now();
        double diff = (tend-t) * 1000;
        //yInfo("diff=%f [ms]", diff);


    }
    //yDebug("before %f", Time::now());
    //Time::delay(1);
    //yDebug("durata %f", Time::now()-iniziaqui);
}

void handProfilerThread::printErr() {
    Stamp ts;
    ts.update();
    Bottle& b = errPort.prepare();
    b.clear();
    //b.addDouble(mp->getError());
    b.addDouble(-1.0);
    errPort.setEnvelope(ts);
    errPort.write();
}

void handProfilerThread::printVel() {
    Stamp ts;
    ts.update();
    Bottle& b = velPort.prepare();
    b.clear();
    b.addDouble(mp->getTanVelocity());
    b.addDouble(mp->getCurvature());
    b.addDouble(mp->getRadius());
    b.addDouble(mp->getAngVelocity());
    velPort.setEnvelope(ts);
    velPort.write();
}

void handProfilerThread::printXd() {
    Stamp ts;
    ts.update();
    Bottle& b = xdPort.prepare();
    b.clear();
    b.addDouble(xd[0]);
    b.addDouble(xd[1]);
    b.addDouble(xd[2]);
    xdPort.setEnvelope(ts);
    xdPort.write();
}

bool handProfilerThread::setCurrentOrientation() {
    Vector x0,o0;
    icart->getPose(x0,o0);
    od = o0;
    return true;
}

bool handProfilerThread::setOrientation(const Vector vectorOrientation) {
    if (vectorOrientation.size() == 4) {
        od = vectorOrientation;
        return true;
    }
    return false;
}

bool handProfilerThread::generateTarget() {
    // translational target part: a circular trajectory
    // in the yz plane centered in [-0.3,-0.1,0.1] with radius=0.1 m
    // and frequency 0.1 Hz (1/10 of 2PI per second)
    //xd[0]=-0.3;
    ///xd[1]=-0.1+0.1*cos(2.0*M_PI*0.1*(t-t0));
    //xd[2]=+0.1+0.1*sin(2.0*M_PI*0.1*(t-t0));

    Vector* _xdpointer = mp->compute(t);

    //Vector _xd;
    if(_xdpointer == NULL) {
        //yInfo("STOP");
        //_xd.resize(3);
        //_xd[0] = 0.0; _xd[1] = 0.0; _xd[2] = 0.0;
        return false;
    }

    xd = *_xdpointer;
    //printf("Error %f %f %f \n", xd[0] -_xd[0], xd[1] - _xd[1], xd[2] - _xd[2]);

    // we keep the orientation of the left arm constant:
    // we want the middle finger to point forward (end-effector x-axis)
    // with the palm turned down (end-effector y-axis points leftward);
    // to achieve that it is enough to rotate the root frame of pi around z-axis
    //od[0] = 0.0; od[1] = 0.0; od[2] = 1.0; od[3] = M_PI;
    //od[0] = 0.29; od[1] = 0.40; od[2] = -0.86; od[3] = 3.09;
    //od[0] = -0.43; od[1] = -0.02; od[2] = -0.90; od[3] = 2.98;
    //od[0] = -0.06; od[1] = -0.87; od[2] = 0.49; od[3] = 2.97;

    return true;
}

void handProfilerThread::saveToArray(){                         //save to file: read encoders and save values in file with timestamp
    //yDebug("saveToArray");
    Vector tempJoints;
    tempJoints.resize(njoints);
    bool retFromEncoders = encs->getEncoders(tempJoints.data());
    if(!retFromEncoders)
        yError("encoders misreading");

    timestamp->update();
    for(int i=0; i<7; i++){
        jointsToSave.push_back(tempJoints[i]);
        //yDebug("position %d : %f", i, tempJoints[i]);
    }
    jointsToSave.push_back(timestamp->getTime());
    infoSamples++;
}

void handProfilerThread::saveToFile(){                         //save to file: read encoders and save values in file with timestamp
    //yDebug("saveToFile");
    outputFile.precision(13);
    if (outputFile.is_open()){
        for(int i=0; i<jointsToSave.length(); i++){
            if (i%8 == 0 && i != 0)
                outputFile << "\n";
            outputFile << jointsToSave[i] << " ";
            //yDebug("position %d : %f", i, jointsToSave[i]);
        }
    }
    else
        yError("Unable to open output file");

    jointsToSave.resize(0);
}

void handProfilerThread::saveInfo(){                                            //save the info in separate file
    if (infoOutputFile.is_open()){
        infoOutputFile << Time::now() - firstDuration << " ";                   //duration
        infoOutputFile << infoSamples << " ";                                   //samples number
        for(int i=0; i<3; i++){
          infoOutputFile << firstPos(i) << " ";                                 //initial cartesian position
        }
        for(int i=0; i<4; i++){
          infoOutputFile << firstOri(i) << " ";                                 //initial cartesian orientation
        }
    }
    else
        yError("Unable to open info output file");
}

void handProfilerThread::startFromFile(){                                       //move from file

    if (partnerTime != 0.0) {
        speedFactor = movementDuration / partnerTime;                           //if partnerTime is set change speedFactor
        yWarning("start from partner time");
    }

    filePath = rf.findFile(fileName + ".log");                                  //read joints file
    for(int i=0; i<repsNumber; i++){
        inputFile.open(filePath.c_str());
        yInfo("opening file.....");
        if(inputFile.is_open()){
            if(speedFactor >= 0.1 && speedFactor <= 10.0){
                yWarning("speedFactor %f ", speedFactor);
                playFromFile();
                yInfo("finished movement");
            }else{
                yError("Speed Factor value is either too low or too high, please insert value between 0.1 and 6.0");
            }
        }
        else {
            yError(".log File not found");
        }
        inputFile.close();
    }
    repsNumber = 1;
    yInfo("reps number reset to 1");
    partnerTime = 0.0;
    state = none;
    idle = true;
}

void handProfilerThread::playFromFile(){
    double jointValue = 0.0;
    double playJoints[8];
    double executionTime = 0;
    double previousTime = 0;

    int playCount = 0;

    bool done = false;
    bool first = true;

    Vector encoders;
    Vector command;
    encoders.resize(njoints);
    command.resize(njoints);

    if(startPos(0) != 0.0 && startPos(1) != 0.0 && startPos(2) != 0.0){         //go to initial position with cartesian controller
        yInfo("icart moving to initial position");
        icart->goToPose(startPos, startOri);
        icart->waitMotionDone();
    }

    encs->getEncoders(encoders.data());                                         // encoder reading from current position
    for (int i = 0; i < njoints; i++) {
        ictrl->setControlMode(i, VOCAB_CM_POSITION_DIRECT);
    }
    command = encoders;

    while(inputFile >> jointValue){
        done = false;
        playJoints[playCount%8] = jointValue;
        playCount++;
        if(playCount%8 == 0){
            if(first){
                yInfo("idir moving to initial position");
                first = false;
                for(int i=0; i<7; i++){
                    double offsetInitial = 0;                                   // put robot in initial position  (adjust position reached with cartesian controller
                    if(command[i] < playJoints[i]){                             // to be more precise according to the first position of .log file)
                        offsetInitial = 0.2;
                    }else if(command[i] > playJoints[i]){
                        offsetInitial = -0.2;
                    }
                    while(command[i] < playJoints[i]-0.2 || command[i] > playJoints[i]+0.2){
                        command[i] = command[i] + offsetInitial;
                        idir->setPositions(command.data());
                        Time::delay(0.01);
                    }
                }
                yInfo("initial position reached");
                Time::delay(2.0);
                previousTime = playJoints[7];
            }else{
                for(int i=0; i<7; i++){
                    command[i] = playJoints[i];
                }
                executionTime = playJoints[7] - previousTime;
                previousTime = playJoints[7];
                if(executionTime <= 0.0) {
                    executionTime = 0.05;
                }
                idir->setPositions(command.data());                             // move robot through trajectory
                yDebug("execution time: %f", executionTime / speedFactor);
                Time::delay(executionTime / speedFactor);                       // time to wait before reaching next point as read from file and modified with speedFactor
            }
        }
    }

    return;
}

void handProfilerThread::limitTorsoYaw() {
    int axis=1; // yaw joint
    double min, max;

    // sometimes it may be helpful to reduce
    // the range of variability of the joints;
    // for example here we don't want the torso
    // to rotate out more than neededSSS

    // we keep the lower limit
    icart->getLimits(axis,&min,&max);
    yInfo("Torso Yaw limits [%f:%f]->[%f:%f]", min, max, MIN_TORSO_YAW, MAX_TORSO_YAW);
    icart->setLimits(axis,MIN_TORSO_YAW,MAX_TORSO_YAW);
}

void handProfilerThread::limitTorsoPitch() {
    int axis=0; // pitch joint
    double min, max;

    // sometimes it may be helpful to reduce
    // the range of variability of the joints;
    // for example here we don't want the torso
    // to lean out more than 30 degrees forward

    // we keep the lower limit
    icart->getLimits(axis,&min,&max);
    icart->setLimits(axis,min,MAX_TORSO_PITCH);
}

void handProfilerThread::processing() {
}

void handProfilerThread::threadRelease() {
    // we require an immediate stop
    // before closing the client for safety reason
    if(icart) {
        yInfo("Stopping the icart");
        icart->stopControl();
        // it's a good rule to restore the controller
        // context as it was before opening the module
        icart->restoreContext(startup_context_id);
        client.close();
    }
    if(igaze){
        igaze->restoreContext(originalContext);
        delete clientGazeCtrl;
    }
    guiPort.interrupt();
    guiPort.close();
    xdPort.interrupt();
    xdPort.close();
    velPort.interrupt();
    velPort.close();

    yInfo("success in thread release");
}

void handProfilerThread::afterStart(bool s) {
    if (s)
        yInfo("Thread started successfully");
    else
        yError("Thread did not start");

    t=t0=t1=yarp::os::Time::now();
}

void handProfilerThread::displayProfile() {
    int r, g, b;
    r =255; g = 0; b = 0;

    if (guiPort.getOutputCount()) {
        // extract important parameters
        Vector vectorList[4];
        vectorList[0] = mp->getO();
        vectorList[1] = mp->getA();
        vectorList[2] = mp->getB();
        vectorList[3] = mp->getC();
        string objectName[4];
        objectName[0].append("O");
        objectName[1].append("A");
        objectName[2].append("B");
        objectName[3].append("C");

        for(int i = 0; i < 4; i++) {
            // preparing the bottle
            Bottle& obj = guiPort.prepare();
            obj.clear();
            obj.addString("object"); // command to add/update an object
            obj.addString(objectName[i]);
            // object dimensions in millimiters
            // (it will be displayed as an ellipsoid with the tag "my_object_name")
            obj.addDouble(5);
            obj.addDouble(5);
            obj.addDouble(5);
            // object position in millimiters
            // reference frame: X=fwd, Y=left, Z=up
            obj.addDouble((vectorList[i])[0] * 1000);
            obj.addDouble((vectorList[i])[1] * 1000);
            obj.addDouble((vectorList[i])[2] * 1000);
            // object orientation (roll, pitch, yaw) in degrees
            obj.addDouble(0.0);
            obj.addDouble(0.0);
            obj.addDouble(0.0);
            // object color (0-255)
            obj.addInt(r);
            obj.addInt(g);
            obj.addInt(b);
            // transparency (0.0=invisible 1.0=solid)
            obj.addDouble(1.0);

            guiPort.writeStrict();
        }
    }
}

void handProfilerThread::displayTarget() {
    int r, g, b;
    if(state == simulation) {
        r =255; g =255; b = 0;
    }
    else {
        r = 0; g =255; b =0;
    }

    if ((guiPort.getOutputCount()) && (count%30==0)) {
        Bottle& obj = guiPort.prepare();
        obj.clear();
        obj.addString("object"); // command to add/update an object
        string str("");
        sprintf((char*)str.c_str(),"%d",count);
        yInfo("displaying %s", str.c_str());
        obj.addString(str.c_str());
        // object dimensions in millimiters
        // (it will be displayed as an ellipsoid with the tag "my_object_name")
        obj.addDouble(5);
        obj.addDouble(5);
        obj.addDouble(5);
        // object position in millimiters
        // reference frame: X=fwd, Y=left, Z=up
        obj.addDouble(xd[0] * 1000);
        obj.addDouble(xd[1] * 1000);
        obj.addDouble(xd[2] * 1000);
        // object orientation (roll, pitch, yaw) in degrees
        obj.addDouble(0.0);
        obj.addDouble(0.0);
        obj.addDouble(0.0);
        // object color (0-255)
        obj.addInt(r);
        obj.addInt(g);
        obj.addInt(b);
        // transparency (0.0=invisible 1.0=solid)
        obj.addDouble(1.0);

        guiPort.writeStrict();
    }
}

/*void handProfilerThread::printStatus() {
    if (t-t1>=PRINT_STATUS_PER) {
        Vector x,o,xdhat,odhat,qdhat;

        // we get the current arm pose in the
        // operational space
        icart->getPose(x,o);

        // we get the final destination of the arm
        // as found by the solver: it differs a bit
        // from the desired pose according to the tolerances
        icart->getDesired(xdhat,odhat,qdhat);

        double e_xdhat=norm(xdhat-x);
        double e_o=norm(odhat-o);
        double e_xd=norm(xd-x);

        fprintf(stdout,"+++++++++\n");
        fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
        fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
        fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
        fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
        fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
        fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
        fprintf(stdout,"norm(e_xd)   [m] = %g\n",e_xd);
        fprintf(stdout,"norm(e_xdhat)[m] = %g\n",e_xdhat);
        fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
        fprintf(stdout,"---------\n\n");

        t1=t;
    }
    }*/

void handProfilerThread::printStatus() {
    Vector x,o,xdhat,odhat,qdhat;

    // we get the current arm pose in the
    // operational space
    icart->getPose(x,o);

    // we get the final destination of the arm
    // as found by the solver: it differs a bit
    // from the desired pose according to the tolerances
    icart->getDesired(xdhat,odhat,qdhat);

    double e_xdhat=norm(xdhat-x);
    double e_o=norm(odhat-o);
    double e_xd=norm(xd-x);

    fprintf(stdout,"+++++++++\n");
    fprintf(stdout,"xd          [m] = %s\n",xd.toString().c_str());
    fprintf(stdout,"xdhat       [m] = %s\n",xdhat.toString().c_str());
    fprintf(stdout,"x           [m] = %s\n",x.toString().c_str());
    fprintf(stdout,"od        [rad] = %s\n",od.toString().c_str());
    fprintf(stdout,"odhat     [rad] = %s\n",odhat.toString().c_str());
    fprintf(stdout,"o         [rad] = %s\n",o.toString().c_str());
    fprintf(stdout,"norm(e_xd)   [m] = %g\n",e_xd);
    fprintf(stdout,"norm(e_xdhat)[m] = %g\n",e_xdhat);
    fprintf(stdout,"norm(e_o) [rad] = %g\n",e_o);
    fprintf(stdout,"---------\n\n");

    t1=t;
}

void handProfilerThread::onStop() {
}
