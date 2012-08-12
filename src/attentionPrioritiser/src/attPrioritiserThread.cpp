// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file attPrioritiserThread.cpp
 * @brief Implementation of the gaze arbiter thread(see header attPrioritiserThread.h)
 */

#include <iCub/attPrioritiserThread.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cstring>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;
using namespace yarp::math;
using namespace iCub::iKin;
using namespace attention::predictor;
using namespace attention::evaluator;


#define THRATE          10
#define PI              3.14159265
#define BASELINE        0.068      // distance in meters between eyes
#define TIMEOUT_CONST   5          // time constant after which the motion is considered not-performed    
#define THCORR          0.99
#define corrStep        10
#define FOVEACONFID     20

//defining the frequency [event/sec] as relation between time and event
const static double frequencyRule[NUMSTATES] = { 
    0.1,  // reset
    0.1,  // wait
    0.5,  // vergenge
    10.0, // SMP
    1.0,  // planned saccade 
    0.5,  // exprSacc
    0.1   // Pred
};

inline void copy_8u_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

// ***********************************************************

 
static Vector orVector (Vector &a, Vector &b) {
    int dim = a.length();
    Vector res((const int)dim);
    for (int i = 0; i < a.length() ; i++) {
        if((a[i]==1)||(b[i]==1)) {
            res[i] = 1;
        }
        else {
            res[i] = 0;
        }
    }
    return res;
}

/************************************************************************/

bool getCamPrj(const string &configFile, const string &type, Matrix **Prj)
{
    *Prj=NULL;

    if (configFile.size())
    {
        Property par;
        par.fromConfigFile(configFile.c_str());

        Bottle parType=par.findGroup(type.c_str());
        string warning="Intrinsic parameters for "+type+" group not found";

        if (parType.size())
        {
            if (parType.check("w") && parType.check("h") &&
                parType.check("fx") && parType.check("fy"))
            {
                // we suppose that the center distorsion is already compensated
                double cx = parType.find("w").asDouble() / 2.0;
                double cy = parType.find("h").asDouble() / 2.0;
                double fx = parType.find("fx").asDouble();
                double fy = parType.find("fy").asDouble();

                Matrix K  = eye(3,3);
                Matrix Pi = zeros(3,4);

                K(0,0) = fx;
                K(1,1) = fy;
                K(0,2) = cx;
                K(1,2) = cy; 
                
                Pi(0,0) = Pi(1,1) = Pi(2,2) = 1.0;

                *Prj = new Matrix;
                **Prj = K * Pi;

                return true;
            }
            else
                fprintf(stdout,"%s\n",warning.c_str());
        }
        else
            fprintf(stdout,"%s\n",warning.c_str());
    }

    return false;
}

/**********************************************************************************/


attPrioritiserThread::attPrioritiserThread(string _configFile) : RateThread(THRATE) {
    printf("attPrioritiserThread::attPrioritiserThread \n");
    cUpdate = 0;
    collectionLocation = new int[4*2];
    numberState = 6; //null, vergence, smooth pursuit, saccade
    configFile = _configFile;
    waitTime   = 3.0;

    waitType = "ant"; //ant-anticip, fix-fixation 

    sacPlanner    = 0;
    trajPredictor = 0;
    tracker       = 0;

    // boolean values
    firstVer           = false;
    firstVergence      = true;
    done               = true;
    idleReinf          = true;
    reinfFootprint     = true;
    postSaccCorrection = true;
    firstNull          = true;
    executing          = false;
    correcting         = false;
    pred_accomplished  = false;
    stopVergence       = true;

    // initialisation of integer values
    phiTOT = 0;
    xOffset = yOffset = zOffset = 0;
    blockNeckPitchValue =-1;

    // top-down state
    topDownState[0] = 1; // no Top-Down contribution
    topDownState[1] = 0; // color Top-Down contribution
    topDownState[2] = 0; // orientation Top-Down Contribution
    topDownState[3] = 0; // motion Top-Down Contribution
    
    //selection kValue for null top-down
    kNull[0] = 0.03;
    kNull[1] = 0.03;
    kNull[2] = 0.50;
    kNull[3] = 0.01;
    kNull[4] = 0.01;
    kNull[5] = 0.00;
    //tNull    = 5000;

    // selection kValue for color top-down
    kColor[0] = 0.00;
    kColor[1] = 0.05;
    kColor[2] = 0.25;
    kColor[3] = 0.05;
    kColor[4] = 0.05;
    kColor[5] = 0.75;
    //tColor    = 5000;
    
    // selection kValue for color and orientation top-down
    kColOri[0] = 0.15;  // intensity
    kColOri[1] = 0.05;  // motion
    kColOri[2] = 0.95;  // chrominance
    kColOri[3] = 0.00;  // orientation
    kColOri[4] = 0.00;  // edges
    kColOri[5] = 0.40;  // proto-objects
    //tColOri    = 5000;


    // selection kValue for color and orientation top-down
    /*kMotion[0] = 0.00;  // intensity
    kMotion[1] = 0.90;  // motion
    kMotion[2] = 0.10;  // chrominance
    kMotion[3] = 0.00;  // orientation
    kMotion[4] = 0.00;  // edges
    kMotion[5] = 0.00;  // proto-objects
    */

    Matrix trans(NUMSTATES,NUMSTATES);
    trans(0,0) = 1.0 ; trans(0,1) = 1.0 ; trans(0,2) = 1.0 ; trans(0,3) = 1.0; trans(0,4) = 1.0; trans(0,5) = 1.0; trans(0,6) = 1.0;
    trans(1,0) = 1.0 ; trans(1,1) = 1.0 ; trans(1,2) = 1.0 ; trans(1,3) = 1.0; trans(1,4) = 1.0; trans(1,5) = 1.0; trans(1,6) = 1.0;
    trans(2,0) = 1.0 ; trans(2,1) = 1.0 ; trans(2,2) = 1.0 ; trans(2,3) = 1.0; trans(2,4) = 1.0; trans(2,5) = 1.0; trans(2,6) = 1.0;
    trans(3,0) = 1.0 ; trans(3,1) = 1.0 ; trans(3,2) = 0.0 ; trans(3,3) = 1.0; trans(3,4) = 1.0; trans(3,5) = 1.0; trans(3,6) = 1.0;
    trans(4,0) = 1.0 ; trans(4,1) = 1.0 ; trans(4,2) = 0.0 ; trans(4,3) = 1.0; trans(4,4) = 1.0; trans(4,5) = 1.0; trans(4,6) = 1.0;
    trans(5,0) = 1.0 ; trans(5,1) = 1.0 ; trans(5,2) = 1.0 ; trans(5,3) = 1.0; trans(5,4) = 1.0; trans(5,5) = 1.0; trans(5,6) = 1.0;
    trans(6,0) = 1.0 ; trans(6,1) = 1.0 ; trans(6,2) = 1.0 ; trans(6,3) = 1.0; trans(6,4) = 1.0; trans(6,5) = 1.0; trans(6,6) = 1.0;
    stateTransition=trans;

    Vector req(NUMSTATES);
    for(int i = 0; i < NUMSTATES; i++) {
        req(i) = 0;
    }
    stateRequest = req;
    allowedTransitions = req;

    Vector s(NUMSTATES);
    s(0) = 1;
    s(1) = 0;
    s(2) = 0;
    s(3) = 0;
    s(4) = 0;
    s(5) = 0;
    s(6) = 0;
    state = s;
    
    Vector t(3);
    t(0) = -0.6;
    t(1) = 0;
    t(2) = 0.6;
    xFix = t;

    // initialisation of state relative flags
    // 0 - null action
    // 1 - wait
    // 2 - vergence
    // 3 - smooth pursuit
    // 4 - planned saccade
    // 5 - express saccade
    // 6 - predict
    printf("attPrioritiserThread::attPrioritiserThread: initilisation of the states (%d)  \n", NUMSTATES);
    for (int k = 0; k < NUMSTATES; k++) {
        allowStateRequest[k] = true;
        waitResponse[k]      = false;
        bufCommand[k]        = NULL;
    }

    printf("starting the tracker.... \n");
    ResourceFinder* rf = new ResourceFinder();
    
    if(visualFeedback) {
        tracker = new trackerThread(*rf);
        tracker->setName(getName("/matchTracker").c_str());
        tracker->start();
    }
    else {
        printf("tracker not started because visualFeedback disable by user \n");
    }

    
    printf("attPrioritiserThread initialization ended correctly \n");
}

attPrioritiserThread::~attPrioritiserThread() {
    // MUST BE REMOVED THE RF AND TRACKER ALLOCATED IN THE CONSTRUCTOR
    //tracker->stop();
}

bool attPrioritiserThread::threadInit() {

    printf(" \n ---------------------------- attPrioritiserThread::threadInit:starting the thread.... \n");
    
    eyeL = new iCubEye("left");
    eyeR = new iCubEye("right");    

    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        Matrix &Prj = *PrjL;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
     
    template_size = 20;
    search_size = 100;
    point.x = 320;
    point.y = 240;

    template_roi.width = template_roi.height = template_size;
    search_roi.width   = search_roi.height   = search_size;

    //opening port section 
    string rootNameStatus("");rootNameStatus.append(         getName("/feedback:o"));
    feedbackPort.open(rootNameStatus.c_str());
    string rootNameOutput("");rootNameOutput.append(         getName("/cmd:o"));
    outputPort.open(rootNameOutput.c_str());
    string rootNameTiming("");rootNameTiming.append(         getName("/timing:o"));
    timingPort.open(rootNameTiming.c_str());
    string rootNameTemplate("");rootNameTemplate.append(     getName("/template:o"));
    templatePort.open(rootNameTemplate.c_str());
    string rootNameDatabase("");rootNameDatabase.append(     getName("/database:o"));
    blobDatabasePort.open(rootNameDatabase.c_str());
    string rootNameInhibition("");rootNameInhibition.append( getName("/inhibition:o"));
    inhibitionPort.open(rootNameInhibition.c_str());
    string nameFBEarlyVision("");nameFBEarlyVision.append(   getName("/earlyVision:o"));
    feedbackEarlyVision.open(nameFBEarlyVision.c_str());
    string nameFBSelective("");nameFBSelective.append(       getName("/selectiveAtt:o"));
    feedbackSelective.open(nameFBSelective.c_str());    
    string nameFBProtoObject("");nameFBProtoObject.append(   getName("/protoObject:o"));
    feedbackProtoObject.open(nameFBProtoObject.c_str());
    string nameHighLevelLoop("");nameHighLevelLoop.append(   getName("/highLoop:o"));
    highLevelLoopPort.open(nameHighLevelLoop.c_str());
    string nameDesiredTrack("");nameDesiredTrack.append(     getName("/desTrack:o"));
    desiredTrackPort.open(nameDesiredTrack.c_str());
    string nameTrackPosition("");nameTrackPosition.append(   getName("/trackPosition:i"));
    trackPositionPort.open(nameTrackPosition.c_str());
    string nameDirect("");nameDirect.append(                 getName("/direct:o"));
    directPort.open(nameDirect.c_str());
    string nameFace("");nameFace.append(                     getName("/face:o"));
    facePort.open(nameFace.c_str());

    inLeftPort.open(getName("/imgMono:i").c_str());
    //inRightPort.open(getName("/matchTracker/img:o").c_str());
    //firstConsistencyCheck=true;

    
    //initializing gazecontrollerclient
    printf("initialising gazeControllerClient \n");
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/attPrioritiser/gaze");
    localCon.append(getName(""));
    option.put("local",localCon.c_str());

    printf("opening the polydriver gaze Controller \n");
    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
        printf("success in opening igaze \n");
       clientGazeCtrl->view(igaze);
    }
    else {
        printf("could not open the clientGazeCtrl \n");
        return false;
    }
    
    pendingCommand   = new Bottle();
    isPendingCommand = false;

    inhibitionImage = new ImageOf<PixelMono>;
    inhibitionImage->resize(320,240);
    inhibitionImage->zero();
    unsigned char* pinhi = inhibitionImage->getRawImage();
    int padding = inhibitionImage->getPadding();
    int rowsizeInhi = inhibitionImage->getRowSize();
    
    string name = getName("");
    timeoutResponseStart = Time::now();
    
    printf("attPrioritiserThread::threadInit: starting the saccade planner \n");
    sacPlanner = new sacPlannerThread(name);       
    sacPlanner->referenceRetina(imgLeftIn);
    sacPlanner->start();

    printf("attPrioritiserThread::threadInit: starting the tracker \n");
    ResourceFinder* rf = new ResourceFinder();
    tracker = new trackerThread(*rf);
    tracker->setName(getName("/matchTracker").c_str());
    tracker->start();
    
    printf("attPrioritiserThread::threadInit:starting the trajectoryPredictor \n");
    trajPredictor = new trajectoryPredictor();
    trajPredictor->setTracker(tracker);
    trajPredictor->start();
    printf("--------------------------------attPrioritiser::threadInit:end of the threadInit \n");

    //-----------------------------------------------------------------------------------
    int rowA,colA;

    Matrix R;
    Matrix Q;
    Matrix P0;
    
    Vector z0;
    Vector x0;
    Vector z;
    Vector x;
    Vector u;
    
    eQueue = new evalQueue(false);
    //-------------------------------------------------------------------------------------------------
    printf("Creating prediction models \n");
    linVelModel* modelA = new linVelModel();
    modelA->init(1.0);
    printf("modelA\n %s \n %s \n", modelA->getA().toString().c_str(), modelA->getB().toString().c_str());
    genPredModel* mA = dynamic_cast<genPredModel*>(modelA);
    printf("after dynamic_cast setting the model \n");
    attention::evaluator::evalThread evalVel1;
    evalVel1.setModel(modelA);
    rowA = modelA->getRowA();
    colA = modelA->getColA();
    printf("success in setting the model \n");
    
    R.resize (rowA,colA);
    Q.resize (rowA,colA);
    P0.resize(rowA,colA);
    
    z0.resize (rowA);
    x0.resize (rowA);
    z.resize (colA);
    x.resize (colA);
    u.resize (1);
    
    printf("preparing the set of measurements %d %d \n", numIter, rowA);
    zMeasure.resize(numIter, rowA);
    uMeasure.resize(numIter, rowA);

    for(int j = 0; j < numIter; j++) {
        for (int k = 0 ; k < rowA; k ++) {
            zMeasure(k,j) = 1.0 + Random::uniform();
            uMeasure(k,j) = 1.0 + Random::uniform();
        }
    }
    
    printf("initialising the matrices of the Kalman Filter \n");
    for (int i = 0; i < rowA; i++) {
        for (int j = 0; j < colA; j++) { 
            Q(i, j) += 0.01; 
            R(i, j) += 0.001;
            P0(i,j) += 0.01;
        }      
    }
    
    evalVel1.init(z0, x0, P0);
    //evalVel1.start();
    //eQueue->push_back(evalVel1);
    
    return true;
}

void attPrioritiserThread::interrupt() {
    //interrupting ports
    inLeftPort.interrupt();
    inRightPort.interrupt();

    feedbackPort.interrupt();
    templatePort.interrupt();
    inhibitionPort.interrupt();
    blobDatabasePort.interrupt();
   
    timingPort.interrupt();
    feedbackEarlyVision.interrupt();
    feedbackSelective.interrupt();
    feedbackProtoObject.interrupt();

    highLevelLoopPort.interrupt();
    desiredTrackPort.interrupt();
    trackPositionPort.interrupt();
    directPort.interrupt();
    facePort.interrupt();
}

void attPrioritiserThread::threadRelease() {

    printf("--------------------------------- attPrioritiserThread::threadRelease:successfully restored previous gaze context \n"); 
    delete clientGazeCtrl;

    evalVel1.stop();

    inLeftPort.close();
    inRightPort.close();

    printf("closing feedback port \n");
    feedbackPort.close();
    printf("closing template port \n");
    templatePort.close();
    printf("closing database port \n");
    printf("closing inhibition port \n");
    inhibitionPort.close();
    blobDatabasePort.close();
    
    timingPort.close();
    printf("closing feedback ports  \n");
    feedbackEarlyVision.close();
    feedbackSelective.close();
    feedbackProtoObject.close();

    highLevelLoopPort.close();
    printf("closing particle filter ports \n");
    desiredTrackPort.close();
    trackPositionPort.close();    
    directPort.close();
    facePort.close();
    printf("closing timing port \n");
    
    printf("\n \n attPrioritiserThread::threadRelease:successfully closed all the ports \n");

    /*
    delete eyeL;
    delete eyeR;
    printf("attPrioritiserThread::threadRelease:successfully deleted eyes references \n");
 
    
    if(0!=sacPlanner) {
        printf("attPrioritiserThread::threadRelease:deleting the clientPlanner \n");
        sacPlanner->stop();
    }

    if(0 != tracker) {
        printf("attPrioritiserThread::threadRelease:stopping the tracker \n");
        tracker->stop();
    }
    
    if(0!=trajPredictor) {
        printf("attPrioritiserThread::threadRelease:stopping the traject.Predictor \n");
        //trajPredictor->stop();
        printf("attPrioritiserThread::threadRelease:success in stopping traj.Predict \n");
    }
    printf("attPrioritiserThread::threadRelease:corretly stopped the trajPredictor \n");
    
    */

    //delete sacPlanner;
    printf("----------------------------------- attPrioritiserThread::threadRelease:success in releasing all the components \n");
}

void attPrioritiserThread::setDimension(int w, int h) {
    width = w;
    height = h;
}

void attPrioritiserThread::setBlockPitch(double value) {
    blockNeckPitchValue = value;
}

void attPrioritiserThread::setName(string str) {
    this->name=str;
    printf("name: %s \n", name.c_str());
}

std::string attPrioritiserThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void attPrioritiserThread::setFacialExpression(std::string command) {
    if(facePort.getOutputCount()) {
        Bottle& value = facePort.prepare();
        value.clear();
        value.addString(command.c_str());
        facePort.write();
    }    
}

void attPrioritiserThread::setRobotName(string str) {
    this->robot = str;
    printf("name: %s \n", name.c_str());
}

void attPrioritiserThread::init(const int x, const int y) {
    point.x = x;
    point.y = y;
    template_roi.width = template_roi.height = template_size;
    search_roi.width = search_roi.height = search_size;
}

void attPrioritiserThread::getPoint(CvPoint& p) {
    //tracker->getPoint(p);
}


void attPrioritiserThread::sendPendingCommand() {
    //highLevelLoopPort.prepare() = *pendingCommand;
    //highLevelLoopPort.write();
    printf("sending pending %s \n", pendingCommand->toString().c_str());
    update(this, pendingCommand);
}

void attPrioritiserThread::run() {
    //Bottle& status = feedbackPort.prepare();
    Bottle& timing = timingPort.prepare();
    //double start = Time::now();
    //printf("stateRequest: %s \n", stateRequest.toString().c_str());
    //mutex.wait();
    
    // checking for missed commands
    double timeoutResponse = Time::now() - timeoutResponseStart;
    

    // checking for pending communication
    if(isPendingCommand) {
        printf ("!!!!!!!!!!!! PENDING COMMAND !!!!!!! \n");
        sendPendingCommand();
        isPendingCommand = false;
        printf("sent the command \n");
        printf("___________________________________ \n");
        return;
    }
    //Vector-vector element-wise product operator between stateRequest possible transitions
    else if ((stateRequest(0) != 0) || (stateRequest(1) != 0) || (stateRequest(2) != 0) ||
             (stateRequest(3) != 0) || (stateRequest(4) != 0) || (stateRequest(5) != 0) ||
             (stateRequest(6) != 0)) {
        printf("time of inactivity %f \n",Time::now() - timeoutResponseStart );
        printf("#### stateRequest      : %s \n", stateRequest.toString().c_str());
        printf("#### state             : %s \n", state.toString().c_str());
        Vector c(6);
        c = stateRequest * (state * stateTransition);
        allowedTransitions = orVector(allowedTransitions ,c);
        // resetting the requests
        stateRequest(0) = 0; stateRequest(1) = 0; stateRequest(2) = 0; stateRequest(3) = 0; stateRequest(4) = 0; stateRequest(5) = 0; stateRequest(6) = 0;
        printf("#### allowedTransitions: %s \n", allowedTransitions.toString().c_str());
        
        // notify observer concerning the state in which the prioritiser sets in
        Bottle notif;
        notif.addVocab(COMMAND_VOCAB_ACT);
        notif.addDouble(allowedTransitions(0));  // reset
        notif.addDouble(allowedTransitions(1));  // wait
        notif.addDouble(allowedTransitions(2));  // vergence 
        notif.addDouble(allowedTransitions(3));  // smooth pursuit
        notif.addDouble(allowedTransitions(4));  // planned saccade
        notif.addDouble(allowedTransitions(5));  // express saccade
        notif.addDouble(allowedTransitions(6));  // trajectory prediction
        setChanged();
        notifyObservers(&notif);

        startAction = Time::now(); //start the time counter for the activated action
        
        if(!allowedTransitions(0)) {
            firstNull = true;
        }
        
        setFacialExpression("M08");
        setFacialExpression("L01");
        
    }
    else if(timeoutResponse > 10.0) {
        printf("TIMEOUT %f \n %d %d %d %d %d %d \n", timeoutResponse,
               waitResponse[0],
               waitResponse[1],
               waitResponse[2],
               waitResponse[3],
               waitResponse[4],
               waitResponse[5],
               waitResponse[6]);
        pendingCommand->clear();
        isPendingCommand = true;
        if(waitResponse[0]) {
            printf("TIMEOUT RESPONSE in WAITING\n");
            pendingCommand->addString("WAIT_ACC");
        }
        else if(waitResponse[1]) {
            printf("TIMEOUT RESPONSE in WAITING\n");
            pendingCommand->addString("WAIT_ACC");
        }
        else if(waitResponse[2]) {
            printf("TIMEOUT RESPONSE in VERGENCE\n");
            pendingCommand->addString("VER_REF");
        }
        else if(waitResponse[3]) {
            printf("TIMEOUT RESPONSE in SMPURSUIT \n");
            pendingCommand->addString("SM_ACC");
        }
        else if(waitResponse[4]) {
            printf("TIMEOUT RESPONSE in SACCADE \n");
            pendingCommand->addString("SAC_FAIL");
        }
        else if(waitResponse[5]) {
            printf("TIMEOUT RESPONSE in \n");
            pendingCommand->addString("SAC_FAIL");
        }
        else if(waitResponse[6]) {
            printf("TIMEOUT RESPONSE in SACCADE\n");
            pendingCommand->addString("PRED_FAIL");
        }
        else {
            isPendingCommand = false;  
             timeoutResponseStart = Time::now(); //starting the timer for a control on responses
        }
    }


    //mutex.post();
    //double end = Time::now();
    //double interval = end - start;
    //printf("interval: %f", interval);

    //printf("state: %s \n", state.toString().c_str());
    //printf("allowedTransitions: %s \n", allowedTransitions.toString().c_str());

    if(allowedTransitions(6)>0) {
        // ----------------  Trajectory Prediction  -----------------------
        state(6) = 1; state(5) = 0; state(4) = 0 ; state(3) = 0; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        waitResponse[6] = true;
        timeoutResponseStart = Time::now(); //starting the timer for a control on responses
        printf("resetting response timer \n");
        
        
        printf("\n \n ---------------- Trajectory prediction --------------------- \n \n");
        
        /*
        // nofiying action            
        Bottle notif;
        notif.clear();
        notif.addVocab(COMMAND_VOCAB_ACT);
        // code for prediction accomplished
        notif.addDouble(states(0));  // null
        notif.addDouble(states(1));  // vergence 
        notif.addDouble(states(2));  // smooth pursuit
        notif.addDouble(states(3));  // planned saccade
        notif.addDouble(states(4));  // express saccade
        notif.addDouble(states(5));  // trajectory prediction
        setChanged();
        notifyObservers(&notif);
        */
        
        //double predVx = 0.0, predVy = 0.0;
        //bool predictionSuccess = false;
        amplitude = 0;
        tracker->init(u,v);
        tracker->waitInitTracker();

        bool predictionSuccess = trajPredictor->estimateVelocity(u, v, predVx, predVy, predXpos, predYpos, predTime, predDistance);
        amplitude = 0; // null amplitude in prediction )no action involved)
        
        printf("after trajectory prediction %f %f (land: %f, %f) in %f \n", predVx, predVy, predXpos, predYpos, predTime);
        

        // nofiying state transition            
        //Bottle notif;
        //notif.clear();
        //notif.addVocab(COMMAND_VOCAB_STAT);
        //notif.addDouble(1);                  // code for prediction accomplished
        //setChanged();
        //notifyObservers(&notif); 

        printf("just notified observers \n");

        //predictionSuccess = false; // forcing the prediction failed
        if(predictionSuccess) {
            printf("prediction success: velocity(%f, %f) time( %f) \n", predVx, predVy, predTime);
            
            // action after prediction 
            //Bottle& sent     = highLevelLoopPort.prepare();            
            //sent.clear();
            //sent.addString("PRED_ACC");
            //highLevelLoopPort.write();                        

            pendingCommand->clear();
            pendingCommand->addString("PRED_ACC");
            isPendingCommand = true;     
            
        }
        else {
            printf("prediction failed \n");
            
            // nofiying state transition            
            //notif.clear();
            //notif.addVocab(COMMAND_VOCAB_STAT);
            //notif.addDouble(3);                  // code for prediction accomplished
            //setChanged();
            //notifyObservers(&notif);            

            pendingCommand->clear();
            pendingCommand->addString("PRED_FAIL");
            isPendingCommand = true;  
        }
        printf("_________________ Trajectory prediction  _____________________\n\n");
    }
    else if(allowedTransitions(5)>0) {
        // ----------------  Express Saccade  -----------------------
        state(6) = 0; state(5) = 1 ; state(4) = 0; state(3) = 0; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        waitResponse[5] = true;            // waitResponse[4] because action is planned saccade
        timeoutResponseStart = Time::now(); //starting the timer for a control on responses
        printf("resetting response timer \n");
        
        // forcing in idle early processes during oculomotor actions
        // not postsaccadic correction
        printf("------------------ Express Saccade --------------- \n");
        amplitude = sqrt( (u - 160) * (u - 160) + (v - 120) * (v - 120));
        tracker->init(u,v);
        tracker->waitInitTracker();
            
        if(feedbackPort.getOutputCount()) {
            Bottle* sent     = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_INH);
            feedbackPort.write(*sent, *received);
            delete sent;
            delete received;
            Time::delay(0.01);
        }

        if(!executing) {                       
            
            correcting = false;
            executing  = true;
            collectionLocation[0 + 0] = u;
            collectionLocation[0 * 2 + 1] = v;
            printf("express saccade in position %d %d \n", u,v);
            
            int centroid_x = u;
            int centroid_y = v;
            //Bottle& commandBottleOFF = outputPort.prepare();
            //commandBottleOFF.clear();
            //commandBottleOFF.addString("COR_OFF");
            //outputPort.write();
            //Time::delay(0.5);
            bool port_is_writing;
            Bottle& commandBottle = outputPort.prepare();
            commandBottle.clear();
            commandBottle.addString("SAC_EXPR");
            commandBottle.addInt(centroid_x);
            commandBottle.addInt(centroid_y);
            zDistance = 0.6;
            commandBottle.addDouble(zDistance);
            outputPort.write();
            
            //Bottle& commandBottleON = outputPort.prepare();
            //commandBottleON.clear();
            //commandBottleON.addString("COR_ON");
            //outputPort.write();

            // Bottle notif;
            // notif.clear();
            // notif.addVocab(COMMAND_VOCAB_STAT);
            // notif.addDouble(6);                  // code for fixStableKO
            // setChanged();
            // notifyObservers(&notif);

            // immediate accomplish command for express saccade
            pendingCommand->clear();
            pendingCommand->addString("SAC_ACC_HIGH");
            isPendingCommand = true; 
            
            /*
            //correcting flag is set when the saccade is accomplished
            timeout = 0;
            timeoutStart = Time::now();
            while((!correcting)&&(timeout < 2.0)) {
                timeoutStop = Time::now();
                timeout = timeoutStop - timeoutStart;
                Time::delay(0.01);
            }
            if(timeout >= 2.0) {
                printf("Express Saccade timed out \n");
                
                // nofiying state transition            
                //Bottle notif;
                //notif.clear();
                //notif.addVocab(COMMAND_VOCAB_STAT);
                //notif.addDouble(3);                  // code for fixStableKO
                //setChanged();
                //notifyObservers(&notif);

            }
            else {
                printf("Express Saccade  accomplished \n");
                // nofiying state transition            
                
                //Bottle notif;
                //notif.clear();
                //notif.addVocab(COMMAND_VOCAB_STAT);
                //notif.addDouble(2);                  // code for fixStableKO
                //setChanged();
                //notifyObservers(&notif);
            } 
            */

            Time::delay(0.01);
            
            /*
            timeoutStop = Time::now();
            timeout = timeoutStop - timeoutStart;
            if(timeout < time) {
                timeoutStop = Time::now();
                timeout = timeoutStop - timeoutStart;
                printf("es \n");
            }
            else {    
                executing = true;
                //execution here
                int centroid_x, centroid_y;                
                
                printf("executing the centroid of the group of saccades \n");
                int j = 0;
                for(j = 0; j < 1; j++) {
                    centroid_x += collectionLocation[j * 2];
                    centroid_y += collectionLocation[j * 2 + 1];
                }
                centroid_x =(int) floor(centroid_x / j);
                centroid_y =(int) floor(centroid_y / j);

                Bottle& commandBottle=outputPort.prepare();
                commandBottle.clear();
                commandBottle.addString("SAC_MON");
                commandBottle.addInt(centroid_x);
                commandBottle.addInt(centroid_y);
                commandBottle.addDouble(zDistance);
                outputPort.write();
                
                //allowedTransitions(2) = 0;
                //executing = false;
            }
            */
        }

        //resume early processes
        //printf("          SENDING COMMAND OF RESUME      \n");
        if(feedbackPort.getOutputCount()) {
            //printf("feedback resetting \n");
            Bottle* sent     = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_NINH);
            feedbackPort.write(*sent, *received);
            delete sent;
            delete received;
        }
        Time::delay(0.01);
    }    
    else if(allowedTransitions(4)>0) {
        // ----------------  Planned Saccade  -----------------------
        state(6) = 0; state(5) = 0 ; state(4) = 1 ; state(3) = 0; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        waitResponse[4] = true;
        timeoutResponseStart = Time::now(); //starting the timer for a control on responses
        printf("resetting response timer %d %d \n", u,v);
        amplitude = sqrt( (u - 160) * (u - 160) + (v - 120) * (v - 120));
        //initialising the tracker
        if(visualFeedback) {
            printf("tracker enable for planned saccade \n");
            tracker->init(u,v);
            tracker->waitInitTracker();
        }
        else {
            printf("tracker disabled due user`s request \n");
        }
        
        //forcing in idle early processes during oculomotor actions
        if(feedbackPort.getOutputCount()) {
            //printf("feedback resetting \n");
            Bottle* sent     = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_NINH);
            feedbackPort.write(*sent, *received);
            delete sent;
            delete received;
        }

        if(!executing) {                       
            //printf("\n \n ____________________ Planned Saccade ___________________ \n");
            
            if((u==-1)||(v==-1)) {
                printf(" \n ----------- Stereo Planned Saccade ------------  \n");
                executing = true;
                // executing the stereo saccade without postsaccadic correction
                Bottle& commandBottle=outputPort.prepare();
                commandBottle.clear();
                commandBottle.addString("SAC_ABS");
                commandBottle.addDouble(xObject);
                commandBottle.addDouble(yObject);
                commandBottle.addDouble(zObject);
                outputPort.write();
                
            }
            else {

                printf(" \n ------- Monocular Planned Saccade -------------  \n");
                /*
                // nofiying state transition            
                Bottle notif;
                notif.clear();
                notif.addVocab(COMMAND_VOCAB_STAT);
                notif.addDouble(3);                  // code for fixStableKO
                setChanged();
                notifyObservers(&notif);
                */
                
                printf("initialising the planner thread %f \n", time);
                /*
                sacPlanner->setSaccadicTarget(u,v);
                timeoutStart = Time::now();
                executing = true;       
                timeoutStop = Time::now();
                timeout = timeoutStop - timeoutStart;
                printf("waiting for planned saccade \n");
                while(timeout < time) {
                    timeoutStop = Time::now();
                    timeout = timeoutStop - timeoutStart;
                    //printf("ps \n");
                }
                */

                // activating the sacPlanner
                sacPlanner->setSaccadicTarget(u,v);
                sacPlanner->wakeup();
                Time::delay(0.01);
                
            
                // executing the saccade
                Bottle& commandBottle=outputPort.prepare();
                commandBottle.clear();
                commandBottle.addString("SAC_MONO");
                commandBottle.addInt(u);
                commandBottle.addInt(v);
                commandBottle.addDouble(zDistance);
                outputPort.write();
                
                // post-saccadic connection
                if(postSaccCorrection) {
                    //printf("waiting for saccade accomplished in postSaccadic correction \n");
                    // wait for accomplished saccadic event
                    timeout = 0;
                    timeoutStart = Time::now();
                    while((!correcting)&&(timeout < 5.0)) {
                        timeoutStop = Time::now();
                        timeout = timeoutStop - timeoutStart;
                        Time::delay(0.005);
                    }
                    if(timeout > 5.0) {
                        printf("Saccade accomplished timeout \n");
                        
                        // nofiying state transition            
                        //Bottle notif;
                        //notif.clear();
                        //notif.addVocab(COMMAND_VOCAB_STAT);
                        //notif.addDouble(3);                  // code for fixStableKO
                        //setChanged();
                        //notifyObservers(&notif);

                        // immediate accomplish command for express saccade
                        pendingCommand->clear();
                        pendingCommand->addString("SAC_FAIL_HIGH");
                        isPendingCommand = true; 
                    }
                    else {
                        printf("Saccade accomplished command received \n");

                        // nofiying state transition            
                        //Bottle notif;
                        //notif.clear();
                        //notif.addVocab(COMMAND_VOCAB_STAT);
                        //notif.addDouble(2);                  // code for fixStableKO
                        //setChanged();
                        //notifyObservers(&notif);
                        //printf("stopping vergence \n");
                        //stopVergence = false;
                        
                        // immediate accomplish command for express saccade
                        pendingCommand->clear();
                        pendingCommand->addString("SAC_ACC_HIGH");
                        isPendingCommand = true; 
                    }


                    correcting = false;   // resetting the correction flag
                    sacPlanner->setCompare(true);
                    sacPlanner->wakeup();
                    Time::delay(0.05);
                    
                    /*
                    // correction or second saccade??
                    double corr = sacPlanner->getCorrValue();
                    printf("received the response from the planner %f \n", corr);
                    if(corr < THCORR) {
                    //getDirection and calculating the pixel dimension of the correction
                    double dirRad = (sacPlanner->getDirection() * PI) / 180.0;
                    printf("direction of the correction in degrees %f \n",sacPlanner->getDirection() );
                    int xVar = (int) floor(cos(dirRad) * corrStep);
                    int yVar = (int) floor(sin(dirRad) * corrStep);
                    Bottle& commandBottle=outputPort.prepare();
                    commandBottle.clear();
                    commandBottle.addString("SAC_MONO");
                    commandBottle.addInt(160 + xVar);
                    commandBottle.addInt(120 + yVar);
                    commandBottle.addDouble(zDistance);
                    outputPort.write();
                    } 
                    */
                    
                    Time::delay(0.05);
                    
                } //end of postsaccadic correction                
            }
            printf("-------------------------------------------------- \n");
        }
        
        //resume early processes
        //printf("SENDING COMMAND OF RESUME      \n");
        if(feedbackPort.getOutputCount()) {
            //printf("feedback resetting \n");
            Bottle* sent     = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_NINH);
            feedbackPort.write(*sent, *received);
            delete sent;
            delete received;
        }
        //printf("AFTER COMMAND OF RESUME \n");
        Time::delay(0.005);
    }
    else if(allowedTransitions(3)>0) {
        // ----------------  Smooth Pursuit  -----------------------
        state(6) = 0 ; state(5) = 0 ; state(4) = 0 ; state(3) = 1 ; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        waitResponse[3] = true;
        timeoutResponseStart = Time::now();
        printf("resetting response timer \n");
        amplitude = sqrt(Vx * Vx + Vy * Vy) / 100.0;

        // smooth pursuit does not initialise the tracker because the pred does it

        if(!executing) {                       
            printf("---------------- Smooth Pursuit --------------\n");
            printf("SM_PUR %f %f %f \n", Vx, Vy, time);
            
            // executing the saccade
            if(outputPort.getOutputCount()) {                
                Bottle& commandBottle=outputPort.prepare();
                commandBottle.clear();
                commandBottle.addString("SM_PUR");
                commandBottle.addDouble(Vx);
                commandBottle.addDouble(Vy);
                commandBottle.addDouble(time);
                outputPort.write();
            }
            
            printf("--------------------------------------------------\n");
        }
    }
    else if(allowedTransitions(2)>0) {
        // ----------------  Vergence  -----------------------
        state(6) = 0; state(5) = 0 ; state(4) = 0 ; state(3) = 0 ; state(2) = 1; state(1) = 0 ; state(0) = 0;
        waitResponse[2] = true;
        timeoutResponseStart = Time::now();
        printf("resetting response timer \n");
        // initialising the tracker not necessary because it started before
        // the risk is that it redefines tracking for every vergence command
        //tracker->init(160,120);
        //tracker->waitInitTracker();
        amplitude = 1;
        

        printf(" __________________ Vergence __________________ \n");
        /*    
        if((feedbackPort.getOutputCount())&&(firstVergence)) {
            printf("vergence: sending suspend command \n");
            Bottle* sent     = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SUSPEND);
            feedbackPort.write(*sent, *received);
            delete sent;
            delete received;
            Time::delay(0.05);
            firstVergence =  false;
            timeoutStart = Time::now();
        }
        */

        if(!executing) {
 
            //correcting =  false;
            //collectionLocation[0 + 0] = u;
            //collectionLocation[0 * 2 + 1] = v;
            //printf("vergence in relative angle %f \n", phi);
            
            int centroid_x = u;
            int centroid_y = v;
            //Bottle& commandBottleOFF = outputPort.prepare();
            //commandBottleOFF.clear();
            //commandBottleOFF.addString("COR_OFF");
            //outputPort.write();
            //Time::delay(0.5);

            timeoutStop = Time::now();
            timeout = timeoutStop - timeoutStart;

            if((ver_accomplished)/*||(timeout>5.0)*/) {
                //resume early processes
                //printf("vergence: accomplished sending resume command \n");

                // nofiying state transition            
                //Bottle notif;
                //notif.clear();
                //notif.addVocab(COMMAND_VOCAB_STAT);
                //notif.addDouble(10);                  // code for vergence accomplished
                //setChanged();
                //notifyObservers(&notif);

                stopVergence = true;

                if(feedbackPort.getOutputCount()) {
                    //printf("feedback resetting \n");
                    Bottle* sent     = new Bottle();
                    Bottle* received = new Bottle();    
                    sent->clear();
                    sent->addVocab(COMMAND_VOCAB_RESUME);
                    feedbackPort.write(*sent, *received);
                    delete sent;
                    delete received;
                }
                firstVergence = true;
            }
            else {
                // nofiying state transition            
                //Bottle notif;
                //notif.clear();
                //notif.addVocab(COMMAND_VOCAB_STAT);
                //notif.addDouble(8);                  // code for vergence angle under correction
                //setChanged();
                //notifyObservers(&notif);
                
                //printf("vergence: sending relative angle to the gazeArbiter \n");
                bool port_is_writing;
                Bottle& commandBottle = outputPort.prepare();
                
                printf("VER@%f %f %f \n", phi, phi2, phi3);
                commandBottle.clear();
                commandBottle.addString("VER_REL");
                commandBottle.addDouble(phi);
                commandBottle.addDouble(phi2);
                commandBottle.addDouble(phi3);
                outputPort.write();
                Time::delay(0.1);
                
            }                        
        }
    
    }
    else if(allowedTransitions(1)>0) {
        //--------------- wait --------------------------------
        state(6) = 0 ; state(5) = 0; state(4) = 0 ; state(3) = 0 ; state(2) = 0 ; state(1) = 1 ; state(0) = 0;
        waitResponse[1] = true;
        timeoutResponseStart = Time::now();
        printf("resetting response timer \n");
        amplitude = 0;
        //tracker->init(u,v);
        //tracker->waitInitTracker();
        Bottle& commandBottle = outputPort.prepare();
        commandBottle.clear();
        commandBottle.addString("WAIT");
        commandBottle.addDouble(u);
        commandBottle.addDouble(v);
        commandBottle.addDouble(0.5);
        outputPort.write();
        
        printf("--------------------- Wait -------------------- \n");
        printf("Standby in Wait ....%s \n", waitType.c_str());
        /*
        double tstart = Time::now();
        double tdiff = 0;
        while( tdiff < waitTime) {
            Time::delay(0.01);
            tdiff = Time::now() - tstart;
        }

        pendingCommand->clear();
        pendingCommand->addString("WAIT_ACC");
        isPendingCommand = true;    
        */
        
        printf("____________________   Wait ____________________ \n");
        
    }
    else if(allowedTransitions(0)>0) {
        // ----------------  reset  -----------------------
        state(6) = 0 ; state(5) = 0; state(4) = 0 ; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;
        waitResponse[0] = true;
        timeoutResponseStart = Time::now();
        amplitude = 0;
        //tracker->init(u,v);
        //tracker->waitInitTracker();

        
        // nofiying state transition            
        Bottle notif;
        notif.clear();
        notif.addVocab(COMMAND_VOCAB_STAT);
        notif.addDouble(0);                  // code for prediction accomplished
        setChanged();
        notifyObservers(&notif);
        firstNull = false;
        waitResponse[0] = false;
        
    }
    else {
        //printf("No transition \n");
    }

    //------------------------------------------------------------------------------
    //printf("--------------------------------------------------------->%d \n",done);
    
    if(allowedTransitions(6)>0) { //prediction
        mutex.wait();
        state(6) = 0; state(5) = 0; state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(6) = 0;
        executing = false;  //executing=false allows new action commands
        // execution = false moved to after the SAC_ACC is received
        //printf ("Transition request 4 reset \n");
        mutex.post();
    }
    else if(allowedTransitions(5)>0) { //express saccade
        mutex.wait();
        state(6) = 0; state(5) = 0; state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(5) = 0;
        executing = false;  //executing=false allows new action commands
        // execution = false moved to after the SAC_ACC is received
        //printf ("Transition request 4 reset \n");
        mutex.post();
    }
    else if(allowedTransitions(4)>0) { //planned saccade
        mutex.wait();
        state(6) = 0; state(5) = 0; state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(4) = 0;
        executing = false;  //executing=false allows new action commands
        // execution = false moved to after the SAC_ACC is received
        //printf ("Transition request 4 reset \n");
        mutex.post();
    }
    else if(allowedTransitions(3)>0) { //smooth pursuit
        mutex.wait();
        state(6) = 0; state(5) = 0; state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(3) = 0;
        executing = false;
        //printf ("Transition request 3 reset \n");
        mutex.post();
    }
    else if(allowedTransitions(2)>0) { //vergence
        mutex.wait();
        state(6) = 0; state(5) = 0; state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(2) = 0;
        executing = false;
        //printf ("Transition request 2 reset \n");
        mutex.post();
    }
    else if(allowedTransitions(1)>0) { //wait
        mutex.wait();
        state(6) = 0; state(5) = 0; state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(1) = 0;
        executing = false;
        //printf ("Transition request 1 reset \n");
        mutex.post();
    }
    else if(allowedTransitions(0)>0) { //reset action
        mutex.wait();
        state(6) = 0; state(5) = 0; state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(0) = 0;
        executing = false;
        //printf ("Transition request 1 reset \n");
        mutex.post();
    }
    
}

void attPrioritiserThread::printCommandBuffer() {
    for (int i = 0; i < NUMSTATES; i++) {
        if(bufCommand[i]!=NULL)
            printf("%d >>>>> %s \n",i,bufCommand[i].toString().c_str());
        else
            printf("%d >>>>> NULL \n",i);
    }
}

void attPrioritiserThread::setAllowStateRequest(int _pos, int value) {
    int pos;
    //mapping of the state
    if((_pos >= 4) && (_pos <= 6)) pos = 4;    
    else if (_pos > 6)             pos = _pos - 2;
    else                           pos = _pos; 
    allowStateRequest[pos] = value; 
}

void attPrioritiserThread::executeClone(int _pos) {
    int pos;
    //mapping of the state

    if((_pos >= 4) && (_pos <= 6)) pos = 4;    
    else if (_pos > 6)             pos = _pos - 2;
    else                           pos = _pos;

    switch(pos) {
    case 0: {
        // reset
        stateRequest[pos] = 1.0;
        Bottle b;
        b.addString("RESET");
        bufCommand[pos] = b;
    } break;
    case 1: {
        //vergence
        printf("executeClone::vergence \n");
        stateRequest[pos] = 1.0;
        Bottle b;
        b.addString("VER");
        b.addDouble(1.0);
        b.addDouble(1.0);
        b.addDouble(1.0);
        bufCommand[pos] = b;
    }break;
    case 2: {
        //sm_pursuit
        stateRequest[pos] = 1.0;
        Bottle b;
        b.addString("SM_PUR");
        b.addInt(0);
        b.addInt(0);
        b.addDouble(0.5);
        bufCommand[pos] = b;
    }
    case 3 : {
        //mono saccade
        stateRequest[pos] = 1.0;
        Bottle b;
        b.addString("SAC_MONO");
        b.addInt(160);
        b.addInt(120);
        b.addDouble(0.5);
        b.addDouble(1.0);
        bufCommand[pos] = b;
    }break;
    case 4 : {
        //express saccade
        stateRequest[pos] = 1.0;
        Bottle b;
        b.addString("SAC_EXP");
        b.addInt(160);
        b.addInt(120);
        b.addDouble(0.5);
        b.addDouble(0.1);
        bufCommand[pos] = b;
    }break;
    case 5 : {
        //express saccade
        stateRequest[pos] = 1.0;
        Bottle b;
        b.addString("PRED");
        b.addInt(160);
        b.addInt(120);
        bufCommand[pos] = b;
    }break;
    }
}

bool attPrioritiserThread::executeCommandBuffer(int _pos) {
    int pos;
    //mapping of the state
    if((_pos >= 4) && (_pos <= 6)) pos = 4;    
    else if (_pos > 6)             pos = _pos - 2;
    else                           pos = _pos;
    
    
    printf("executing a command saved in the buffer pos %d translated in position %d \n",_pos,pos);
    if (bufCommand[pos] == NULL) {
        printf("no action in the buffer for pos:%d \n", pos);        
        if(isLearning()) {
            printf("using default value when in Learning \n");
            switch (pos) {
            case 0: {
                printf("default RESET action \n");
                stateRequest[pos] = 1.0;
                return true;
            }break;
            case 1: {
                printf("default WAIT action \n");
                stateRequest[pos] = 1.0;
                u = 160;
                v = 120;
                return true;
            }break;
            case 2: {
                printf("default VERG action \n");
                stateRequest[pos] = 1.0;
                phi  = 1.0;
                phi2 = 1.0;
                phi3 = 1.0;
                return true;
            }break;
            case 3: {
                printf("default SMP action \n");
                stateRequest[pos] = 1.0;                
                return true;
            }break;                
            case 4: {
                printf("default SAC action \n");
                stateRequest[pos] = 1.0;
                u = 160;
                v = 120;
                return true;
            }break;
            case 5: {
                printf("default EXPR_SAC action \n");
                stateRequest[pos] = 1.0;
                u = 160;
                v = 120;
                return true;
            }break;
            case 6: {
                printf("default PRED action \n");
                stateRequest[pos] = 1.0;
                u = 160;
                v = 120;
                return true;
            }break;    
            }
        }
        printCommandBuffer();
        return false;
    }   
    else {        
        printf("found action \n");
        printf("Bottle: %s \n", bufCommand[pos].toString().c_str());
        stateRequest[pos] = 1.0;
        bufCommand[pos] = NULL;
        return true;
    }
    
}


//*********************** LIST OF BEHAVIOURS ****************************

void attPrioritiserThread::fixCenter(int elapseTime) {
    
    printf("FIX CENTER \n");
    printf("FIX CENTER \n");
    printf("FIX CENTER \n");
    printf("FIX CENTER \n");
    printf("FIX CENTER \n");

    Bottle& commandBottle=outputPort.prepare();
    commandBottle.clear();
    commandBottle.addString("STOP");
    outputPort.write();
    
    Time::delay(0.5);

    mutex.wait();
    xObject = -0.9;
    yObject = 0.0;
    zObject = 0.5;
    u = -1;
    v = -1;
    stateRequest[3] = 1;
    mono = false;   
    mutex.post();
}

void attPrioritiserThread::sendColorCommand(int redValue, int greenValue, int blueValue){
    // setting TopDown state
    topDownState[0] = 0;
    topDownState[1] = 1;
    topDownState[2] = 0;
    topDownState[3] = 0;
    
    Bottle* sent     = new Bottle();
    Bottle* received = new Bottle();
    printf("Seeking rgb coloured objects \n");
    //setting selective attention
    //map1
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K1);
    sent->addDouble(kColor[0]);
    feedbackSelective.write(*sent, *received);
    //map2
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K2);
    sent->addDouble(kColor[1]);
    feedbackSelective.write(*sent, *received);
    //map3
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K3);
    sent->addDouble(kColor[2]);
    feedbackSelective.write(*sent, *received);
    //map4
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K4);
    sent->addDouble(kColor[3]);
    feedbackSelective.write(*sent, *received);
    //map5
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K5);
    sent->addDouble(kColor[4]);
    feedbackSelective.write(*sent, *received);
    //map6
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K6);
    sent->addDouble(kColor[5]);
    feedbackSelective.write(*sent, *received);
    
    //timing of the saccades
    //sent->clear();
    //sent->addVocab(COMMAND_VOCAB_SET);
    //sent->addVocab(COMMAND_VOCAB_TIME);
    //sent->addDouble(tNull);
    //feedbackSelective.write(*sent, *received);
    
    //setting saliencyBlobFinder
    //weight BU
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_WBU);
    sent->addDouble(0.05);
    feedbackProtoObject.write(*sent, *received);
    //weight TD           
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_WTD);
    sent->addDouble(0.95);
    feedbackProtoObject.write(*sent, *received);
    //colour red
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_TRED);
    sent->addInt(redValue);
    feedbackProtoObject.write(*sent, *received);
    //colour green
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_TGRE);
    sent->addInt(greenValue);
    feedbackProtoObject.write(*sent, *received);
    //colour blue
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_TBLU);
    sent->addInt(blueValue);
    feedbackProtoObject.write(*sent, *received);
    
    delete sent;
    delete received;
}


void attPrioritiserThread::seek(Bottle command) {
    int voc = command.get(1).asVocab();
    printf("seeking subject to %s \n", command.get(1).asString().c_str());
    switch (voc) {
    case COMMAND_VOCAB_RED:
        {
            sendColorCommand(255,0,0);
        }
        break;
    case COMMAND_VOCAB_GRE:
        {
            sendColorCommand(0,255,0);
        }
        break;
    case COMMAND_VOCAB_BLU:
        {
            sendColorCommand(0,0,255);
        }
        break;
    case COMMAND_VOCAB_RGB:
        {
            int redValue   = command.get(2).asInt();
            int greenValue = command.get(3).asInt();
            int blueValue  = command.get(4).asInt();
            printf("looking for color %d, %d, %d \n", redValue, greenValue, blueValue);            
            sendColorCommand(redValue, greenValue, blueValue);
        }
        break;
    case COMMAND_VOCAB_NULL:
        {
            // setting TopDown state
            topDownState[0] = 1;
            topDownState[1] = 0;
            topDownState[2] = 0;
            topDownState[3] = 0;

            Bottle* sent     = new Bottle();
            Bottle* received = new Bottle();
            printf("Seeking Reset \n");
            //setting selective attention
            //map1
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SET);
            sent->addVocab(COMMAND_VOCAB_K1);
            sent->addDouble(kNull[0]);
            feedbackSelective.write(*sent, *received);
            //map2
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SET);
            sent->addVocab(COMMAND_VOCAB_K2);
            sent->addDouble(kNull[1]);
            feedbackSelective.write(*sent, *received);
            //map3
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SET);
            sent->addVocab(COMMAND_VOCAB_K3);
            sent->addDouble(kNull[2]);
            feedbackSelective.write(*sent, *received);
            //map4
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SET);
            sent->addVocab(COMMAND_VOCAB_K4);
            sent->addDouble(kNull[3]);
            feedbackSelective.write(*sent, *received);
            //map5
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SET);
            sent->addVocab(COMMAND_VOCAB_K5);
            sent->addDouble(kNull[4]);
            feedbackSelective.write(*sent, *received);
            //map6
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SET);
            sent->addVocab(COMMAND_VOCAB_K6);
            sent->addDouble(kNull[5]);
            feedbackSelective.write(*sent, *received);
            
            //timing of the saccades
            //sent->clear();
            //sent->addVocab(COMMAND_VOCAB_SET);
            //sent->addVocab(COMMAND_VOCAB_TIME);
            //sent->addDouble(tNull);
            //feedbackSelective.write(*sent, *received);
            
            //setting saliencyBlobFinder
            //weight BU
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SET);
            sent->addVocab(COMMAND_VOCAB_K6);
            sent->addDouble(0.95);
            feedbackProtoObject.write(*sent, *received);

            delete sent;
            delete received;
        }
        break;
    default: {
        
    }
        break;    
    }  

}

void attPrioritiserThread::reinforceFootprint() {
    reinfFootprint = false;

    printf("SELF REINFORCEMENT \n");

    // setting TopDown state
    topDownState[0] = 0;
    topDownState[1] = 1; // color top-down
    topDownState[2] = 1; // orientation top-down
    topDownState[3] = 0;
    
    Bottle* sent     = new Bottle();
    Bottle* received = new Bottle();
    printf("Seeking rgb coloured objects \n");
    //setting selective attention
    //map1
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K1);
    sent->addDouble(kColOri[0]);
    feedbackSelective.write(*sent, *received);
    //map2
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K2);
    sent->addDouble(kColOri[1]);
    feedbackSelective.write(*sent, *received);
    //map3
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K3);
    sent->addDouble(kColOri[2]);
    feedbackSelective.write(*sent, *received);
    //map4
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K4);
    sent->addDouble(kColOri[3]);
    feedbackSelective.write(*sent, *received);
    //map5
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K5);
    sent->addDouble(kColOri[4]);
    feedbackSelective.write(*sent, *received);
    //map6
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_K6);
    sent->addDouble(kColOri[5]);
    feedbackSelective.write(*sent, *received);
    
    //timing of the saccades
    //sent->clear();
    //sent->addVocab(COMMAND_VOCAB_SET);
    //sent->addVocab(COMMAND_VOCAB_TIME);
    //sent->addDouble(tColOri);
    //feedbackSelective.write(*sent, *received);
    
    //setting saliencyBlobFinder
    //weight BU
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_WBU);
    sent->addDouble(0.05);
    feedbackProtoObject.write(*sent, *received);
    //weight TD           
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_WTD);
    sent->addDouble(0.95);
    feedbackProtoObject.write(*sent, *received);
    //colour red
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_TRED);
    sent->addInt(feedbackBlobRed);
    feedbackProtoObject.write(*sent, *received);
    //colour green
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_TGRE);
    sent->addInt(feedbackBlobGreen);
    feedbackProtoObject.write(*sent, *received);
    //colour blue
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_TBLU);
    sent->addInt(feedbackBlobBlue);
    feedbackProtoObject.write(*sent, *received);

    //feedback feedbackEarlyVision
    //feedbackEarlyVision.write(*sent, *received);
    
    delete sent;
    delete received;

}

void attPrioritiserThread::update(observable* o, Bottle * arg) {
    cUpdate++;
    //printf("ACK. Aware of observable asking for attention \n");
    if (arg != 0) {
        //printf("bottle: %s ", arg->toString().c_str());
        int size = arg->size();
        ConstString name = arg->get(0).asString();
        
        if(!strcmp(name.c_str(),"SAC_MONO")) {
                        
            u = arg->get(1).asInt();
            v = arg->get(2).asInt();                      

            
            //--------------------------------------------------------------------------
            // prediction attempt after triggering stimulus
            if(visualFeedback) {
                mutex.wait();
                if(allowStateRequest[6]) {
                    printf("setting stateRequest[6] \n");
                    reinfFootprint = true;
                    stateRequest[6] = 1;
                    timeoutStart = Time::now();
                    //  changing the accomplished flag
                    mutexAcc.wait();
                    accomplFlag[6];
                    mutexAcc.post();
                    
                }
                mutex.post();     
            
                
            
                // activating the predictor if learning is active            
                if((learning) && (highLevelLoopPort.getOutputCount())) {
                    Bottle& sent     = highLevelLoopPort.prepare();
                    sent.clear();
                    sent.addString("PRED");
                    sent.addInt(u);
                    sent.addInt(v);
                    highLevelLoopPort.write();
                }
            }
            
            
            //---------------------------------------------------------------------------
            zDistance = arg->get(3).asDouble();
            time      = arg->get(4).asDouble();
            printf("saccade mono time: %f with allowed %d  \n", time,allowStateRequest[3]);
            //mutex.wait();
            if(time <= 0.5) {

                // saving bottle in the buffer
                bufCommand[5] = *arg;                
                // express saccade
                mutex.wait();
                if(allowStateRequest[5]) {
                    printf("setting stateRequest[5] \n");
                    reinfFootprint = true;
                    stateRequest[5] = 1;
                    timeoutStart = Time::now();
                    //  changing the accomplished flag
                    mutexAcc.wait();
                    accomplFlag[5] = false;
                    validAction = true;
                    mutexAcc.post();
                }
                mutex.post();

            } 
            else {
                // saving bottle in the buffer
                bufCommand[4] = *arg;
                
                // feedback saccade
                // checking first for reinfFootPrint
                if((!reinfFootprint)&&(!idleReinf)) {
                    // reinforceFootprint already happened
                    // and reinforcement enable
                    // feedback value must still be taken into account 
                    printf("Footprint still active \n");
                    printf("Footprint color  : %d-%d-%d \n", feedbackBlobRed, feedbackBlobGreen, feedbackBlobBlue);
                    printf("Footprint orient : %d-%d-%d-%d \n", feedbackOri0, feedbackOri45, feedbackOri90, feedbackOriM45);
                    printf("Footprint chrominance: %d \n");

                    //comparing with fovea current information
                    Bottle rep;
                    Bottle req;
                    
                    req.clear();
                    req.addVocab(COMMAND_VOCAB_GET);
                    req.addVocab(COMMAND_VOCAB_FRGB);
                    feedbackProtoObject.write(req, rep);
                    cout<<"fovrgb:     "<<rep.toString().c_str()<<endl;
                    unsigned char currentBlobRed    = rep.get(0).asInt();
                    unsigned char currentBlobGreen  = rep.get(1).asInt();
                    unsigned char currentBlobBlue   = rep.get(2).asInt();
                    printf("feedback colour value %d %d %d \n",feedbackBlobRed, feedbackBlobGreen, feedbackBlobBlue);
                    printf("current  fovea colour %d %d %d \n",currentBlobRed , currentBlobGreen ,  currentBlobBlue);
                    
                    //calculating the distance between feedback color and current fovea color
                    double colourDistance = sqrt (
                                                  (feedbackBlobRed   - currentBlobRed)   * (feedbackBlobRed   - currentBlobRed)   + 
                                                  (feedbackBlobGreen - currentBlobGreen) * (feedbackBlobGreen - currentBlobGreen) + 
                                                  (feedbackBlobBlue  - currentBlobBlue)  * (feedbackBlobBlue  - currentBlobBlue)   
                                                  );
                    printf("colour distance in reinforcement %f \n", colourDistance);
                    if(colourDistance < 10.0) {
                        printf("saccade discard for top-down call; footprint desired is similar to fovea footprint \n");
                    }
                    else {                        
                        printf("footprint desired is not the footprint in fovea \n");                        
                        mutex.wait();
                        if(allowStateRequest[4]) {
                            //printf("setting stateRequest[4] \n");
                            stateRequest[4] = 1;
                            mutexAcc.wait();
                            accomplFlag[4] = false;
                            validAction = true;
                            mutexAcc.post();
                            reinfFootprint  = true;   // enabling back the control top-down footprint extraction
                        }
                        mutex.post();
                    }
                }
                else {
                    //reinforceFootprint has not happened yet
                    mutex.wait();
                    if(allowStateRequest[4]) {
                        printf("setting stateRequest[4], reinforceFootprint has not happened \n");
                        stateRequest[4] = 1;
                        mutexAcc.wait();
                        accomplFlag[4] = false;
                        validAction = true;
                        mutexAcc.post();
                    }
                    mutex.post();
                }
            }
            //executing = false;
            //mutex.post();
            timetotStart = Time::now();
            mono = true;
            firstVer = true;

            /*
            // null state
            Bottle notif;
            notif.clear();
            notif.addVocab(COMMAND_VOCAB_STAT);
            notif.addDouble(3);                  // code for fixStateKO 
            setChanged();
            notifyObservers(&notif);
            */
            
        }
        
        else if(!strcmp(name.c_str(),"RESET")) {

            // saving bottle in the buffer
            bufCommand[0] = *arg;
            
            // reseting the state action history
            mutex.wait();
            if(allowStateRequest[0]) {
                printf("setting stateRequest[0] \n");
                reinfFootprint = true;
                stateRequest[0] = 1;
                timeoutStart = Time::now();
                //  changing the accomplished flag
                mutexAcc.wait();
                accomplFlag[0] = false;
                validAction = true;
                mutexAcc.post();
                
            }
            mutex.post();           
        }
        
        else if(!strcmp(name.c_str(),"WAIT")) {

            // saving bottle in the buffer
            bufCommand[1] = *arg;
            u             = arg->get(1).asInt();
            v             = arg->get(2).asInt();
            //waitType      = arg->get(3).asString(); // reading the typology of waiting: ant (anticipatory), fix (fixation)
            waitTime      = arg->get(3).asDouble();
            
            // reseting the state action history
            mutex.wait();
            if(allowStateRequest[1]) {
                printf("setting stateRequest[1] \n");
                reinfFootprint = true;
                stateRequest[1] = 1;
                timeoutStart = Time::now();
                //  changing the accomplished flag
                mutexAcc.wait();
                accomplFlag[1] = false;
                validAction = true;
                mutexAcc.post();
                
            }
            mutex.post();           
        }
        
        else if(!strcmp(name.c_str(),"TRACK_REF")) {
            printf("TRACKER REFINEMENT \n\n");
            tracker->init(160,120);
            tracker->waitInitTracker();
        }
        else if(!strcmp(name.c_str(),"PF_REQ")) {
            // particle filter request
            // saving bottle in the buffer
            bufCommand[5] = *arg;
            // position of cartesian input image of the particle filter
            u = arg->get(1).asInt();
            v = arg->get(2).asInt();
            // sending a command to the particle filter
            Bottle& trackReq = desiredTrackPort.prepare();
            trackReq.clear();
            trackReq.addInt(u);
            trackReq.addInt(v);
            desiredTrackPort.write();
            // reading relavant position from the particol filter
            Time::delay(0.5);
            printf("\n");
            int count = 0;

            
            double timestart, timestop;
            double diff = 0.0;
            
            timestart = Time::now();
            while(diff < 30.0) {
                Bottle* posTrack = trackPositionPort.read();
                u = posTrack->get(0).asDouble();
                v = posTrack->get(1).asDouble();
                printf("tracking position %d %d        \n ",u,v);
                count++;
                
                /*
                Bottle& direct = directPort.prepare();
                direct.clear();
                direct.addString("left");
                direct.addInt(u);
                direct.addInt(v);
                direct.addDouble(0.5);
                directPort.write();
                */

                /*
                // reinforce the template with the latest view of the stimulus
                if(count % 15 == 0) {
                    printf("REINFORCING the TEMPLATE in pos %d %d \n", u, v);
                    Bottle& trackReq = desiredTrackPort.prepare();
                    trackReq.clear();
                    trackReq.addInt(u);
                    trackReq.addInt(v);
                    desiredTrackPort.write();
                }
                */
                
                /*
                // sending command of monocular saccade
                zDistance = 0.5;  // default interacting distance
                time = 0.1;       // type of saccede : express = 0.1; normal = 0.5;
                mutex.wait();
                if(allowStateRequest[3]) {
                    //printf("setting stateRequest[3] \n");
                    stateRequest[3] = 1;
                    mutexAcc.wait();
                    accomplFlag[3] = false;
                    validAction = true;
                    mutexAcc.post();
                    //reinfFootprint  = true;   // enabling back the control top-down footprint extraction
                }
                mutex.post();
                */
                Vector px(2);
                px(0) = u;
                px(1) = v;
                int camSel = 0;
                igaze->lookAtMonoPixel(camSel,px,0.5);          
                
                //Time::delay(0.05);
                timestop = Time::now();
                diff = timestop - timestart;
                
            }
            
            printf("end of the 30 seconds of control!");
          
              
        }
        else if(!strcmp(name.c_str(),"PRED")) {
            
            // saving bottle in the buffer
            bufCommand[6] = *arg;

            u = arg->get(1).asInt();
            v = arg->get(2).asInt();
            
            // prediction attemp after triggering stimulus
            mutex.wait();
            if(allowStateRequest[6]) {
                printf("setting stateRequest[6] \n");
                reinfFootprint = true;
                stateRequest[6] = 1;
                timeoutStart = Time::now();
                //  changing the accomplished flag
                mutexAcc.wait();
                accomplFlag[6] = false;
                validAction = true;
                mutexAcc.post();
                
            }
            mutex.post();           
        }
        else if(!strcmp(name.c_str(),"SAC_ABS")) {
            xObject = arg->get(1).asDouble();
            yObject = arg->get(2).asDouble();
            zObject = arg->get(3).asDouble();
            u = -1;
            v = -1;
            mutex.wait();
            //printf("setting stateRequest[3] \n");
            if(allowStateRequest[4]) {
                stateRequest[4] = 1;
                mutexAcc.wait();
                accomplFlag[4] = false;
                validAction = true;
                mutexAcc.post();
                //executing = false;
            }
            mutex.post();
            mono = false;
        }
        else if(!strcmp(name.c_str(),"SM_PUR")) {

            // saving bottle in the buffer
            bufCommand[3] = *arg;
            Vx   = arg->get(1).asDouble();
            Vy   = arg->get(2).asDouble();
            time = arg->get(3).asDouble();
            mutex.wait();
            printf("recognised PUR command \n");
            if(allowStateRequest[3]) {
                printf("setting stateRequest[2] \n");
                stateRequest[3] = 1;
                mutexAcc.wait();
                accomplFlag[3] = false;
                validAction = true;
                mutexAcc.post();
                //executing = false;
            }
            mutex.post();            
        }
        else if(!strcmp(name.c_str(),"VER_REL")) {

            // saving bottle in the buffer
            bufCommand[2] = *arg;
            
            phi  = arg->get(1).asDouble();
            phi2 = arg->get(2).asDouble();
            phi3 = arg->get(3).asDouble();
            
            //printf("\r                                                      \r");
            if(!stopVergence) {
                
                //printf("vergence command received %d \n", firstVergence);
                if(firstVergence){
                    
                    //printf("inside the vergence command \n");
                    mutex.wait();
                    if(allowStateRequest[2]) {
                        //printf("setting stateRequest[1] \n");
                        ver_accomplished = false;
                        stateRequest[2]  = 1;
                        mutexAcc.wait();
                        accomplFlag[2] = false;
                        validAction = true;
                        mutexAcc.post();
                        //executing = false;
                    }
                    mutex.post();
                    
                }
                else {
                    mutex.wait();
                    if(allowStateRequest[2]) {
                        //printf("setting stateRequest[1] \n");
                        ver_accomplished = false;
                        stateRequest[2]  = 1;
                        mutexAcc.wait();
                        accomplFlag[2] = false;
                        validAction = true;
                        mutexAcc.post();
                        //executing = false;
                    }
                    mutex.post();
                }
            } //end if(!vergencestop)
        }

        //****************************** ACTION RESPONSES  ************************************************//
        //**** this section regulates the state transition indicating state of agent after action *********//
        else if(!strcmp(name.c_str(),"SAC_FAIL")) {
            timeoutResponseStart = Time::now(); //starting the timer for a control on responses
            printf("resetting response timer \n");
            printf("reset the correcting flag \n");
            // saccade accomplished flag reset           
            mutex.wait();
            correcting = true;  // flag that indicates the end of the saccade action
            //executing = false;
            mutex.post();
        }
        else if((!strcmp(name.c_str(),"SAC_FAIL_HIGH")) && (waitResponse[4])) {
            waitResponse[4] = false;
            if(facePort.getOutputCount()) {
                Bottle& value = facePort.prepare();
                value.clear();
                value.addString("M38");
                facePort.write();
            }

            //extracting reward measures
            timing    = Time::now() - startAction;
            accuracy  = tracker->getProxMeasure();            
            amplitude = 1.0;
            frequency = frequencyRule[4];

            // nofiying state transition to fixStable ok           
            Bottle notif;
            notif.clear();
            notif.addVocab(COMMAND_VOCAB_STAT);
            notif.addDouble(7);                  // code for fixStableKO saccade not accomplished
            notif.addDouble(timing);
            notif.addDouble(accuracy);
            notif.addDouble(amplitude);
            notif.addDouble(frequency);
            setChanged();
            notifyObservers(&notif);

            /*
            // reset action
            notif.clear();
            printf("notify action reset \n");
            notif.addVocab(COMMAND_VOCAB_ACT);
            // code for reset action
            notif.addDouble(1.0);  // reset
            notif.addDouble(0.0);  // vergence 
            notif.addDouble(0.0);  // smooth pursuit
            notif.addDouble(0.0);  // planned saccade
            notif.addDouble(0.0);  // express saccade
            notif.addDouble(0.0);  // trajectory prediction
            setChanged();
            notifyObservers(&notif);
            
            // null state
            notif.clear();
            notif.addVocab(COMMAND_VOCAB_STAT);
            notif.addDouble(0);                  // code for null state
            setChanged();
            notifyObservers(&notif);
            */
            
        }
        else if(!strcmp(name.c_str(),"SAC_ACC")) {
            printf("changing the correcting status \n");

            // saccade accomplished flag set 
            mutex.wait();
            correcting = true;
            //executing = false;
            mutex.post();
        }
        else if((!strcmp(name.c_str(),"SAC_ACC_HIGH")) && (waitResponse[4])) {
            waitResponse[4] = false;
            timeoutResponseStart = Time::now();
            printf("resetting response timer \n");
            if(facePort.getOutputCount()) {
                Bottle& value = facePort.prepare();
                value.clear();
                value.addString("M0B");
                facePort.write();
            }

            
            //  changing the accomplished flag
            mutexAcc.wait();
            accomplFlag[4] = true;               // action number accomplished
            mutexAcc.post();

            //extracting reward measures
            double timing    = Time::now() - startAction;
            double accuracy  = tracker->getProxMeasure();            
            double amplitude = 1.0;
            double frequency = frequencyRule[4];

            CvPoint t; tracker->getPoint(t);

            double distance = sqrt((t.x - 160) * (t.x - 160) + (t.y - 120) * (t.y - 120));
            
            if(distance < FOVEACONFID) {
                
                // nofiying state transition to fixStable ok           
                Bottle notif;
                notif.clear();
                notif.addVocab(COMMAND_VOCAB_STAT);
                notif.addDouble(6);                  // code for fixStableOK accomplished 
                notif.addDouble(timing);
                notif.addDouble(accuracy);
                notif.addDouble(amplitude);
                notif.addDouble(frequency);
                setChanged();
                notifyObservers(&notif);

            }
            else {
                // nofiying state transition to fixStable ok           
                Bottle notif;
                notif.clear();
                notif.addVocab(COMMAND_VOCAB_STAT);
                notif.addDouble(7);                  // code for fixStableOK accomplished 
                notif.addDouble(timing);
                notif.addDouble(accuracy);
                notif.addDouble(amplitude);
                notif.addDouble(frequency);
                setChanged();
                notifyObservers(&notif);
            }

            /*
            // reset action
            notif.clear();
            printf("notify action reset \n");
            notif.addVocab(COMMAND_VOCAB_ACT);
            // code for reset action
            notif.addDouble(1.0);  // reset
            notif.addDouble(0.0);  // vergence 
            notif.addDouble(0.0);  // smooth pursuit
            notif.addDouble(0.0);  // planned saccade
            notif.addDouble(0.0);  // express saccade
            notif.addDouble(0.0);  // trajectory prediction
            setChanged();
            notifyObservers(&notif);
            
            // null state
            notif.clear();
            notif.addVocab(COMMAND_VOCAB_STAT);
            notif.addDouble(0);                  // code for null state
            setChanged();
            notifyObservers(&notif);
            */

            //gathering information about the feature from the preattentive stage (earlyVision)
            if(feedbackEarlyVision.getOutputCount()) {
                //cout<<"communication activated with the earlyVision: ";
                Bottle rep;
                Bottle req;

                req.clear();
                req.addVocab(COMMAND_VOCAB_GET);
                req.addVocab(COMMAND_VOCAB_ORI);
                req.addVocab(COMMAND_VOCAB_P0);
                feedbackEarlyVision.write(req, rep);
                cout<<"orientation pos.0 : "<<rep.toString().c_str()<<endl;
                feedbackOri0 = rep.get(0).asInt(); 

                req.clear();
                req.addVocab(COMMAND_VOCAB_GET);
                req.addVocab(COMMAND_VOCAB_ORI);
                req.addVocab(COMMAND_VOCAB_P90);
                feedbackEarlyVision.write(req, rep);
                cout<<"orientation pos.90 : "<<rep.toString().c_str()<<endl;
                feedbackOri90 = rep.get(0).asInt();
                
                req.clear();
                req.addVocab(COMMAND_VOCAB_GET);
                req.addVocab(COMMAND_VOCAB_ORI);
                req.addVocab(COMMAND_VOCAB_P45);
                feedbackEarlyVision.write(req, rep);
                cout<<"orientation pos.45 : "<<rep.toString().c_str()<<endl;
                feedbackOri45 = rep.get(0).asInt();
                
                req.clear();
                req.addVocab(COMMAND_VOCAB_GET);
                req.addVocab(COMMAND_VOCAB_ORI);
                req.addVocab(COMMAND_VOCAB_N45);
                feedbackEarlyVision.write(req, rep);
                cout<<"orientation neg.45 : "<<rep.toString().c_str()<<endl;
                feedbackOriM45 = rep.get(0).asInt();
            }
            else {
                cout<<"no active feedback to earlyVision"<<endl;
            }

            //gathering information about the feature from the preattentive stage (protoObject component)
            if(feedbackProtoObject.getOutputCount()) {
                cout<<"communication activated with the protoObject: \n";
                Bottle rep;
                Bottle req;

                req.clear();
                req.addVocab(COMMAND_VOCAB_GET);
                req.addVocab(COMMAND_VOCAB_FRGB);
                feedbackProtoObject.write(req, rep);
                cout<<"fovrgb:     "<<rep.toString().c_str()<<endl;
                feedbackBlobRed   = rep.get(0).asInt();
                feedbackBlobGreen = rep.get(1).asInt();
                feedbackBlobBlue  = rep.get(2).asInt();
                printf("rgb colour value %d %d %d \n",feedbackBlobRed, feedbackBlobGreen, feedbackBlobBlue);
            }
            else {
                cout<<"no active feedback to protoObject"<<endl;
                reinfFootprint = false;
            }

            // reinforcing footprint if allowed
            reinfFootprint = false;
            if(reinfFootprint) {                
                reinforceFootprint();
                Time::delay(0.5);
            }
        }
        else if((!strcmp(name.c_str(),"VER_REF")) && (waitResponse[2])) {
            waitResponse[2] = false;
            timeoutResponseStart = Time::now(); //starting the timer for a control on responses
            printf("resetting response timer \n");
            if(facePort.getOutputCount()) {
                Bottle& value = facePort.prepare();
                value.clear();
                value.addString("M38");
                facePort.write();
            }

            //extracting reward measures
            double timing    = Time::now() - startAction;
            double accuracy  = tracker->getProxMeasure();            
            double amplitude = 1.0;
            double frequency = frequencyRule[2];
            
            // nofiying state transition            
            Bottle notif;
            notif.clear();
            notif.addVocab(COMMAND_VOCAB_STAT);
            notif.addDouble(13);                  // code for vergence accomplished
            notif.addDouble(timing);
            notif.addDouble(accuracy);
            notif.addDouble(amplitude);
            notif.addDouble(frequency);
            setChanged();
            notifyObservers(&notif);            
        }
        else if((!strcmp(name.c_str(),"VER_ACC")) && (waitResponse[2])) {
            waitResponse[2] = false;
            timeoutResponseStart = Time::now();
      
            if(facePort.getOutputCount()) {
                Bottle& value = facePort.prepare();
                value.clear();
                value.addString("M0B");
                facePort.write();
            }
            
            // vergence accomplished        
            waitType = "fix";
            printf("waitAccomplished set waitType=fix \n");
            //printf("Vergence accomplished \n");
            mutex.wait();
            if(allowStateRequest[1]) {
                printf("setting stateRequest[1] \n");
                ver_accomplished = true;
                //stateRequest[1]  = 1;
                //executing = false;
            }
            //  changing the accomplished flag
            mutexAcc.wait();
            accomplFlag[2] = true;                // action number accomplished
            validAction = true;
            mutexAcc.post();
            mutex.post();

            //extracting reward measures
            double timing    = Time::now() - startAction;
            double accuracy  = tracker->getProxMeasure();            
            double frequency = frequencyRule[2];
            double amplitude = 1.0;

            // nofiying state transition            
            Bottle notif;
            notif.clear();
            notif.addVocab(COMMAND_VOCAB_STAT);
            notif.addDouble(12);                  // code for vergence accomplished
            notif.addDouble(timing);
            notif.addDouble(accuracy);
            notif.addDouble(amplitude);
            notif.addDouble(frequency);
            setChanged();
            notifyObservers(&notif); 

            pendingCommand->clear();
            pendingCommand->addString("WAIT");
            pendingCommand->addInt(160);
            pendingCommand->addInt(140);
            pendingCommand->addString("fix");
            pendingCommand->addDouble(0.5);
            isPendingCommand = true;      
            

        }
        else if((!strcmp(name.c_str(),"SM_ACC")) && (waitResponse[3])) {
            timeoutResponseStart = Time::now(); //starting the timer for a control on responses
            printf("resetting response timer \n");
            waitResponse[3] = false;
            if(facePort.getOutputCount()) {
                Bottle& value = facePort.prepare();
                value.clear();
                value.addString("M0B");
                facePort.write();
            }
            
            // smooth pursuit accomplished           
            printf("Smooth Pursuit  Accomplished \n");
            mutex.wait();
            if(allowStateRequest[2]) {
                printf("setting stateRequest[2] \n");
                sp_accomplished = true;
                //stateRequest[2]  = 1;
                //executing = false;
            }
            //  changing the accomplished flag
            mutexAcc.wait();
            accomplFlag[3] = true;              // action number accomplished
            validAction    = false;
            mutexAcc.post();           
            mutex.post();
            
            // gets the proximity measure from the tracker
            double timing    = Time::now() - startAction;
            double accuracy  = tracker->getProxMeasure();            
            double frequency = frequencyRule[3];
            double amplitude = 1.0;

            //action ended look into the visual stimulus
            CvPoint t; tracker->getPoint(t);
            double distance = sqrt((t.x - 160) * (t.x - 160) + (t.y - 120) * (t.y - 120));
            
            if(distance < FOVEACONFID) {

                // nofiying state transition into successful tracking
                Bottle notif;
                notif.clear();
                notif.addVocab(COMMAND_VOCAB_STAT);
                notif.addDouble(8);                  // code for smooth-pursuit accomplished
                notif.addDouble(timing);
                notif.addDouble(accuracy);
                notif.addDouble(amplitude);
                notif.addDouble(frequency);
                setChanged();
                notifyObservers(&notif);
                
            }
            else {
                
                // nofiying state transition into unsuccessful tracking
                Bottle notif;
                notif.clear();
                notif.addVocab(COMMAND_VOCAB_STAT);
                notif.addDouble(9);                  // code for smooth-pursuit not accomplished
                notif.addDouble(timing);
                notif.addDouble(accuracy);
                notif.addDouble(amplitude);
                notif.addDouble(frequency);
                setChanged();
                notifyObservers(&notif);
                
            }
        }
        else if((!strcmp(name.c_str(),"PRED_FAIL")) && (waitResponse[6])) {
            timeoutResponseStart = Time::now(); //starting the timer for a control on responses
            printf("resetting response timer \n");
            waitResponse[6] = false;
            if(facePort.getOutputCount()) {
                Bottle& value = facePort.prepare();
                value.clear();
                value.addString("M38");
                facePort.write();
            }
            
            // gets the proximity measure from the tracker
            double timing    = Time::now() - startAction;
            double accuracy  = tracker->getProxMeasure() + 300;            
            double frequency = frequencyRule[6];
            double amplitude = 1.0;

            Bottle notif;
            notif.clear();
            notif.addVocab(COMMAND_VOCAB_STAT);
            notif.addDouble(0);                  // code for prediction fail goes into the NULL state
            notif.addDouble(timing);
            notif.addDouble(accuracy);
            notif.addDouble(amplitude);
            notif.addDouble(frequency);
            setChanged();
            notifyObservers(&notif); 

        }
        else if((!strcmp(name.c_str(),"PRED_ACC")) && (waitResponse[6])) {
            waitResponse[6] = false;
            timeoutResponseStart = Time::now();
            printf("resetting response timer \n");
            waitType = "ant";
            
            if(facePort.getOutputCount()) {
                Bottle& value = facePort.prepare();
                value.clear();
                value.addString("M0B");
                facePort.write();
            }

            // prediction accomplished           
            printf("Prediction Accomplished %f %f  \n", predVx, predVy);
            mutex.wait();
            if(allowStateRequest[6]) {
                printf("setting stateRequest[6] \n");
                pred_accomplished = true;
                //stateRequest[5] = 1;
                //executing = false;
            }
            //  changing the accomplished flag
            mutexAcc.wait();
            accomplFlag[6] =  true;              // action number accomplished
            validAction    = false;
            mutexAcc.post();            
            mutex.post();

            //extracting reward measures
            double timing    = Time::now() - startAction;
            double accuracy  = tracker->getProxMeasure() + 300;
            double frequency = frequencyRule[6];
            double amplitude = 1.0;

             // nofiying state transition            
            //Bottle notif;
            //notif.clear();
            //notif.addVocab(COMMAND_VOCAB_STAT);
            //notif.addDouble(1);                  // code for prediction accomplished
            //setChanged();
            //notifyObservers(&notif);

            
            //notify correct state andtrigger new behaviours
            // a. stable-> uSaccade
            // b. stable-> mSaccade
            // c. stable-> LSaccade
            // d. move  -> movSaccade
            // e. ant   -> antSaccade
            
            //stable stimulus
            if((predVx == 0) || (predVy == 0)) {

                Bottle notif;
                notif.clear();
                notif.addVocab(COMMAND_VOCAB_STAT);
                notif.addDouble(1);                  // code for prediction accomplished
                notif.addDouble(timing);
                notif.addDouble(accuracy);
                notif.addDouble(amplitude);
                notif.addDouble(frequency);
                setChanged();
                notifyObservers(&notif);
                
                /*
                if(predDistance < 10) {
                    //a. stable-> uSaccade
                    printf("stable stimulus with uSaccade \n");
                    // nofiying state transition            
                    Bottle notif;
                    notif.clear();
                    notif.addVocab(COMMAND_VOCAB_STAT);
                    notif.addDouble(1);                  // code for prediction accomplished
                    notif.addDouble(timing);
                    notif.addDouble(accuracy);
                    notif.addDouble(amplitude);
                    setChanged();
                    notifyObservers(&notif);
                }
                else if((predDistance >= 10) && (predDistance < 80)) {
                    // b. stable-> mSaccade
                    printf("stable stimulus with mSaccade \n");
                    // nofiying state transition            
                    Bottle notif;
                    notif.clear();
                    notif.addVocab(COMMAND_VOCAB_STAT);
                    notif.addDouble(2);                  // code for prediction accomplished
                    notif.addDouble(timing);
                    notif.addDouble(accuracy);
                    notif.addDouble(amplitude);
                    setChanged();
                    notifyObservers(&notif);
                }
                else {
                    printf("stable stimulus with LSaccade \n");
                    // nofiying state transition            
                    Bottle notif;
                    notif.clear();
                    notif.addVocab(COMMAND_VOCAB_STAT);
                    notif.addDouble(3);                  // code for prediction accomplished
                    notif.addDouble(timing);
                    notif.addDouble(accuracy);
                    notif.addDouble(amplitude);
                    setChanged();
                    notifyObservers(&notif);
                } 
                */
            }
            else { //not stable object
                if ((predXpos != -1) && (predYpos != 1)) {
                    // move -> anticipatoryPredictor
                    // nofiying state transition            
                    Bottle notif;
                    notif.clear();
                    notif.addVocab(COMMAND_VOCAB_STAT);
                    notif.addDouble(5);                  // code for prediction accomplished in anticipatory state
                    notif.addDouble(timing);
                    notif.addDouble(accuracy);
                    notif.addDouble(amplitude);
                    notif.addDouble(frequency);
                    setChanged();
                    notifyObservers(&notif);                    

                    /*
                    Bottle& sentPred     = highLevelLoopPort.prepare();
                    Bottle* receivedPred = new Bottle();    
                    sentPred.clear();
                    sentPred.addString("SAC_MONO");
                    sentPred.addInt(predXpos);
                    sentPred.addInt(predYpos);
                    highLevelLoopPort.write();                    
                    delete receivedPred;
                    */
                    
                    pendingCommand->clear();
                    pendingCommand->addString("WAIT");
                    pendingCommand->addInt(predVx);
                    pendingCommand->addInt(predVy);
                    pendingCommand->addString("ant");
                    pendingCommand->addDouble(0.5);
                    isPendingCommand = true;
                    
                }
                // b. smooth pursuit
                else { 
                    //move  -> movSaccade                   
                    // nofiying state transition            
                    Bottle notif;
                    notif.clear();
                    notif.addVocab(COMMAND_VOCAB_STAT);
                    notif.addDouble(4);                  // code for prediction accomplished in motion state
                    notif.addDouble(timing);
                    notif.addDouble(accuracy);
                    notif.addDouble(amplitude);
                    notif.addDouble(frequency);
                    setChanged();
                    notifyObservers(&notif);
                    
                    /*
                      Bottle& sent     = highLevelLoopPort.prepare();                  
                      sent.clear();
                      sent.addString("SM_PUR");
                      sent.addInt(predVx);
                      sent.addInt(predVy);
                      sent.addDouble(predTime);
                      highLevelLoopPort.write();
                    */
                    
                    pendingCommand->clear();
                    pendingCommand->addString("SM_PUR");
                    pendingCommand->addInt(predVx);
                    pendingCommand->addInt(predVy);
                    pendingCommand->addDouble(predTime);
                    isPendingCommand = true;                    
                    
                }
            }

            /*
            // a. predictive saccade
            if ((predXpos != -1) && (predYpos != 1)) {
                Bottle& sentPred     = highLevelLoopPort.prepare();
                Bottle* receivedPred = new Bottle();    
                sentPred.clear();
                sentPred.addString("SAC_MONO");
                sentPred.addInt(predXpos);
                sentPred.addInt(predYpos);
                highLevelLoopPort.write();
                
                delete receivedPred;
            }
            // b. smooth pursuit
            else { 
                if((predVx != 0) || (predVy != 0)) {                    
                                  
                    
                    //Bottle& sent     = highLevelLoopPort.prepare();                  
                    //sent.clear();
                    //sent.addString("SM_PUR");
                    //sent.addInt(predVx);
                    //sent.addInt(predVy);
                    //sent.addDouble(predTime);
                    //highLevelLoopPort.write();
                    

                    pendingCommand->clear();
                    pendingCommand->addString("SM_PUR");
                    pendingCommand->addInt(predVx);
                    pendingCommand->addInt(predVy);
                    pendingCommand->addDouble(predTime);
                    isPendingCommand = true;                    

                    
                }
            } 
            */
        }
        else if((!strcmp(name.c_str(),"WAIT_ACC")) && (waitResponse[1])) {
            waitResponse[1] = false;
            timeoutResponseStart = Time::now(); //starting the timer for a control on responses
            printf("resetting response timer \n");

            if(facePort.getOutputCount()) {
                Bottle& value = facePort.prepare();
                value.clear();
                value.addString("M0B");
                facePort.write();
            }

            // smooth pursuit accomplished           
            printf("Wait Accomplished with waitType %s \n", waitType.c_str());
            mutex.wait();
            if(allowStateRequest[1]) {
                printf("setting stateRequest[0] \n");
                //sp_accomplished = true;
                stateRequest[1] = 1;
                //executing = false;
            }
            //  changing the accomplished flag
            mutexAcc.wait();
            accomplFlag[1] = true;
            validAction    = false;
            mutexAcc.post();            
            mutex.post();

            //extracting reward measures
            double timing    = Time::now() - startAction;
            double accuracy  = tracker->getProxMeasure();            
            double frequency = frequencyRule[1];
            double amplitude = 1.0;

            if(!strcmp(waitType.c_str(),"ant")) {
                CvPoint t; tracker->getPoint(t);
                double distance = sqrt((t.x - 160) * (t.x - 160) + (t.y - 120) * (t.y - 120));
                
                if(distance < FOVEACONFID){
                    
                    // nofiying state transition into successful tracking
                    Bottle notif;
                    notif.clear();
                    notif.addVocab(COMMAND_VOCAB_STAT);
                    notif.addDouble(10);                  // code for smooth-pursuit accomplished
                    notif.addDouble(timing);
                    notif.addDouble(accuracy);
                    notif.addDouble(amplitude);notif.addDouble(frequency);
                    setChanged();
                    notifyObservers(&notif);
                    
                }
                else {
                    
                    // nofiying state transition into unsuccessful tracking
                    Bottle notif;
                    notif.clear();
                    notif.addVocab(COMMAND_VOCAB_STAT);
                    notif.addDouble(11);                  // code for smooth-pursuit not accomplished
                    notif.addDouble(timing);
                    notif.addDouble(accuracy);
                    notif.addDouble(amplitude);notif.addDouble(frequency);
                    setChanged();
                    notifyObservers(&notif);
                
                }
            }
            else {
                printf("IN FIXATING \n");
                printf("IN FIXATING \n");
                // nofiying state transition into unsuccessful tracking
                Bottle notif;
                notif.clear();
                notif.addVocab(COMMAND_VOCAB_STAT);
                notif.addDouble(14);                  // code for smooth-pursuit not accomplished
                notif.addDouble(timing);
                notif.addDouble(accuracy);
                notif.addDouble(amplitude);notif.addDouble(frequency);
                setChanged();
                notifyObservers(&notif);                
            }

        }
        /*
        else if(!strcmp(name.c_str(),"RESET")) {
            // reset accomplished           
            printf("Reset Accomplished \n");
            mutex.wait();
            if(allowStateRequest[0]) {
                printf("setting stateRequest[0] \n");
                //sp_accomplished = true;
                stateRequest[0] = 1;
                //executing = false;
            }
            //  changing the accomplished flag
            mutexAcc.wait();
            accomplFlag[0];
            validAction  = false;
            mutexAcc.post();            
            mutex.post();

             // nofiying state transition            
            Bottle notif;
            notif.clear();
            notif.addVocab(COMMAND_VOCAB_STAT);
            notif.addDouble(0);                  // code for reset accomplished in vergence ok state
            setChanged();
            notifyObservers(&notif);

        }
        */
        else if(!strcmp(name.c_str(),"SIM")) {
            // vergence accomplished           
            printf("Simulate \n");
            if(!strcmp(arg->get(1).asString(),"ACT") ){
                // notify observer concerning the state in which the prioritiser sets in
                printf("action request \n");
                
                Vector actionId(5);
                actionId(0) = 0;
                actionId(1) = 0;
                actionId(2) = 0;
                actionId(3) = 0;
                actionId(4) = 0;
                
                
                switch (arg->get(2).asInt()){
                case 0 : actionId(0) = 1; break;
                case 1 : actionId(1) = 1; break;
                case 2 : actionId(2) = 1; break;    
                case 3 : actionId(3) = 1; break;
                case 4 : actionId(4) = 1; break;
                }
                
                Bottle notif;
                notif.clear();
                notif.addVocab(COMMAND_VOCAB_ACT);
                notif.addDouble(actionId(0));
                notif.addDouble(actionId(1)); 
                notif.addDouble(actionId(2));
                notif.addDouble(actionId(3));
                notif.addDouble(actionId(4));
                setChanged();
                notifyObservers(&notif);
            }
            else if(!strcmp(arg->get(1).asString(),"STAT") ) {
                printf("state request \n");
                
                Vector stateId(5);
                stateId(0) = 0;
                stateId(1) = 0;
                stateId(2) = 0;
                stateId(3) = 0;
                stateId(4) = 0;
                
                switch (arg->get(2).asInt()){
                case 0 : stateId(0) = 1; break;
                case 1 : stateId(1) = 1; break;
                case 2 : stateId(2) = 1; break;    
                case 3 : stateId(3) = 1; break;
                case 4 : stateId(4) = 1; break;
                default: printf("in Default"); break;
                }
                
                Bottle notif;
                notif.clear();
                notif.addVocab(COMMAND_VOCAB_STAT);
                notif.addDouble(arg->get(2).asInt());
                //notif.addDouble(stateId(1)); 
                //notif.addDouble(stateId(2));
                //notif.addDouble(stateId(3));
                //notif.addDouble(stateId(4));
                setChanged();
                notifyObservers(&notif); 
            }
            
        }
        else {
            printf("Command %s has not been recognised \n",name.c_str());
        }
    }
}


