// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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
 * @file velocityController.cpp
 * @brief Implementation of the velocitycontroller thread(see header velocityController.h)
 */

#include <iCub/velocityController.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cstring>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;
using namespace yarp::math;
using namespace iCub::iKin;


#define THRATE 10
#define PI  3.14159265
#define BASELINE 0.068     // distance in meters between eyes
#define TIMEOUT_CONST 5    // time constant after which the motion is considered not-performed    
#define INHIB_WIDTH 320
#define INHIB_HEIGHT 240
 

velocityController::velocityController() : RateThread(THRATE) { 
    u = 0;
    v = 0;
}


velocityController::velocityController(string _configFile) : RateThread(THRATE) {
    numberState  = 4; //null, vergence, smooth pursuit, saccade
    countVerNull = 0; 
    configFile = _configFile;
    u = 0;
    v = 0;
    
    //boolean flag initialisation
    firstVer            = false;
    availableVisualCorr = false;
    visualCorrection    = false;
    isOnWings           = false;
    onDvs               = false;
    
    phiTOT = 0;
    xOffset = yOffset = zOffset = 0;
    blockNeckPitchValue =-1;

    Matrix trans(4,4);
    trans(0,0) = 1.0 ; trans(0,1) = 1.0 ; trans(0,2) = 1.0 ; trans(0,3) = 1.0;
    trans(1,0) = 1.0 ; trans(1,1) = 1.0 ; trans(1,2) = 1.0 ; trans(1,3) = 1.0;
    trans(2,0) = 1.0 ; trans(2,1) = 1.0 ; trans(2,2) = 1.0 ; trans(2,3) = 1.0;
    trans(3,0) = 1.0 ; trans(3,1) = 1.0 ; trans(3,2) = 0.0 ; trans(3,3) = 1.0;
    stateTransition=trans;

    Vector req(4);
    req(0) = 0;
    req(1) = 0;
    req(2) = 0;
    req(3) = 0;
    stateRequest = req;
    allowedTransitions = req;

    Vector s(4);
    s(0) = 1;
    s(1) = 0;
    s(2) = 0;
    s(3) = 0;
    state = s;
    
    Vector t(3);
    t(0) = -0.6;
    t(1) = 0;
    t(2) = 0.6;
    xFix = t;

    printf("extracting kinematic informations \n");
}

velocityController::~velocityController() {
    // MUST BE REMOVED THE RF AND TRACKER ALLOCATED IN THE CONSTRUCTOR
}

bool velocityController::threadInit() {
    done=true;
    executing = false;
    printf("starting the thread.... \n");
    
    eyeL = new iCubEye("left");
    eyeR = new iCubEye("right");    

    // remove constraints on the links
    // we use the chains for logging purpose
    //eyeL->setAllConstraints(false);
    //eyeR->setAllConstraints(false);

    // release links
    eyeL->releaseLink(0);
    eyeR->releaseLink(0);
    eyeL->releaseLink(1);
    eyeR->releaseLink(1);
    eyeL->releaseLink(2);
    eyeR->releaseLink(2);

    // if it isOnWings, move the eyes on top of the head 
    if (isOnWings) {
        printf("changing the structure of the chain \n");
        iKinChain* eyeChain = eyeL->asChain();
        //eyeChain->rmLink(7);
        //eyeChain->rmLink(6); ;
        iKinLink* link = &(eyeChain-> operator ()(5));
        //double d_value = link->getD();
        //printf("d value %f \n", d_value);
        //double a_value = link->getA();
        //printf("a value %f \n", a_value);
        link->setD(0.145);
        link = &(eyeChain-> operator ()(6));
        link->setD(0.0);
        //eyeChain->blockLink(6,0.0);
        //eyeChain->blockLink(7,0.0);
        //link = &(eyeChain-> operator ()(6));
        //link->setA(0.0);
        //link->setD(0.034);
        //link->setAlpha(0.0);
        //double d_value = link->getD();
        //printf("d value %f \n", d_value);
        //iKinLink twistLink(0.0,0.034,M_PI/2.0,0.0,-22.0*CTRL_DEG2RAD,  84.0*CTRL_DEG2RAD);
        //*eyeChain << twistLink;
        //eyeL->releaseLink(6);

    }
    else {
        printf("isOnWing false \n");
    }
    
    /*
    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        Matrix &Prj = *PrjL;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    */
    

    /*
    //initializing gazecontrollerclient
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze/");
    localCon.append(getName(""));
    option.put("local",localCon.c_str());

    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    else
        return false;

    
    igaze->storeContext(&originalContext);
  
    if(blockNeckPitchValue != -1) {
        igaze->blockNeckPitch(blockNeckPitchValue);
        printf("pitch fixed at %f \n",blockNeckPitchValue);
    }
    else {
        printf("pitch free to change\n");
    }

    
    string headPort = "/" + robot + "/head";
    string nameLocal("local");

    //initialising the head polydriver
    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", "/localhead");
    optionsHead.put("remote", headPort.c_str());
    robotHead = new PolyDriver (optionsHead);

    if (!robotHead->isValid()){
        printf("cannot connect to robot head\n");
    }
    robotHead->view(encHead);
    
    //initialising the torso polydriver
    printf("starting the polydrive for the torso.... \n");
    Property optPolyTorso("(device remote_controlboard)");
    optPolyTorso.put("remote",("/"+robot+"/torso").c_str());
    optPolyTorso.put("local",("/"+nameLocal+"/torso/position").c_str());
    polyTorso=new PolyDriver;
    if (!polyTorso->open(optPolyTorso))
    {
        return false;
    }
    polyTorso->view(encTorso);
    */

  
    template_size = 20;
    search_size = 100;
    point.x = INHIB_WIDTH;
    point.y = INHIB_HEIGHT;

    template_roi.width = template_roi.height = template_size;
    search_roi.width   = search_roi.height   = search_size;

    /*
    //opening port section 
    string rootNameStatus("");rootNameStatus.append(getName("/status:o"));
    statusPort.open(rootNameStatus.c_str());
    string rootNameTiming("");rootNameTiming.append(getName("/timing:o"));
    timingPort.open(rootNameTiming.c_str());
    string rootNameTemplate("");rootNameTemplate.append(getName("/template:o"));
    templatePort.open(rootNameTemplate.c_str());
    string rootNameDatabase("");rootNameDatabase.append(getName("/database:o"));
    blobDatabasePort.open(rootNameDatabase.c_str());
    string rootNameInhibition("");rootNameInhibition.append(getName("/inhibition:o"));
    inhibitionPort.open(rootNameInhibition.c_str());
    inLeftPort.open(getName("/gazeArbiter/imgMono:i").c_str());
    //inRightPort.open(getName("/matchTracker/img:o").c_str());
    firstConsistencyCheck=true;
    */

    inhibitionImage = new ImageOf<PixelMono>;
    inhibitionImage->resize(INHIB_WIDTH,INHIB_HEIGHT);
    inhibitionImage->zero();
    unsigned char* pinhi = inhibitionImage->getRawImage();
    int padding          = inhibitionImage->getPadding();
    int rowsizeInhi      = inhibitionImage->getRowSize();
    int ym = INHIB_HEIGHT >> 1;
    int xm = INHIB_WIDTH  >> 1;
    //calculating the peek value
    int dx = 50.0;
    int dy = 50.0;
    double sx = (dx / 2) / 3 ; //0.99 percentile
    double sy = (dy / 2) / 3 ;
    double vx = 9; //sx * sx; // variance          
    double vy = 9; //sy * sy;
    
    double rho = 0;
    
    double a = 0.5 / (3.14159 * vx * vy * sqrt(1-rho * rho));
    double b = -0.5 /(1 - rho * rho);
    double k = 1 / (a * exp (b));      
    
    double f, e, d, z = 1;            
    
    double zmax = 0;
    pinhi +=   ((int)(ym-(dy>>1))) * rowsizeInhi + ((int)(xm-(dx>>1)));
    //for the whole blob in this loop
    for (int r = ym - (dy>>1); r <= ym + (dy>>1); r++) {
        for (int c = xm - (dx>>1); c <= xm + (dx>>1); c++){
            
            if((c == xm)&&(r == ym)) { 
                //z = a * exp (b);
                //z = z * k;
                z = 1;
            }
            else {    
                f = ((c - xm) * (c - xm)) /(vx * vx);
                d = ((r - ym)  * (r - ym)) /(vy * vy);
                //e = (2 * rho* (c - ux) * (r - uy)) / (vx * vy);
                e = 0;
                z = a * exp ( b * (f + d - e) );
                z = z * k;
                if(z>zmax) zmax=z;
                z = (1 / 1.646172) * z;
                //z = 0.5;
            }
            
            // restrincting the z gain between two thresholds
            if (z > 1) {
                z = 1;
            }

            //set the image 
            *pinhi++ = 255 * z;                    
        }
        //pinhi += rowsizeInhi - dx  ; //odd
        pinhi += rowsizeInhi - (dx + 1) ;  //even
    }

    printf("     \n zmax = %f \n", zmax);

    /*printf("starting the tracker.... \n");
    ResourceFinder* rf = new ResourceFinder();
    tracker = new trackerThread(*rf);
    tracker->setName(getName("/matchTracker").c_str());
    tracker->start();
    printf("tracker successfully started \n");
    */

    return true;
}

void velocityController::interrupt() {
    //inCommandPort
    inLeftPort.interrupt();
    inRightPort.interrupt();
    statusPort.interrupt();
    templatePort.interrupt();
    inhibitionPort.interrupt();
    blobDatabasePort.interrupt();
    templatePort.interrupt();
    timingPort.interrupt();
}

void velocityController::setDimension(int w, int h) {
    width = w;
    height = h;
}

void velocityController::setBlockPitch(double value) {
    blockNeckPitchValue = value;
}

void velocityController::setName(string str) {
    this->name=str;
    printf("name: %s \n", name.c_str());
}

std::string velocityController::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void velocityController::setRobotName(string str) {
    this->robot = str;
    printf("name: %s \n", name.c_str());
}

void velocityController::init(const int x, const int y) {
    point.x = x;
    point.y = y;
    template_roi.width = template_roi.height = template_size;
    search_roi.width = search_roi.height = search_size;
}

void velocityController::getPoint(CvPoint& p) {
    //tracker->getPoint(p);
}


void velocityController::run() {

    Bottle& status = statusPort.prepare();
    Bottle& timing = timingPort.prepare();
    
    if((u!=0)&&(v!=0)) {
        printf("u =  %f , v = %f \n", u, v);
    }
    
    
}

void velocityController::threadRelease() {
    inLeftPort.close();
    inRightPort.close();
    statusPort.close();
    templatePort.close();
    blobDatabasePort.close();
    inhibitionPort.close();
    timingPort.close();
    //tracker->stop();
    delete eyeL;
    delete eyeR;
    //igaze->restoreContext(originalContext);
    //delete clientGazeCtrl;
}

/*
void velocityController::update(observable* o, Bottle * arg) {
    //printf("ACK. Aware of observable asking for attention \n");
    if (arg != 0) {
        //printf("bottle: %s ", arg->toString().c_str());
        int size = arg->size();
        ConstString name = arg->get(0).asString();
        
        if(!strcmp(name.c_str(),"SAC_MONO")) {
            // monocular saccades with visualFeedback
            printf("MONO SACCADE request \n");
            u = arg->get(1).asInt();
            v = arg->get(2).asInt();
            zDistance = arg->get(3).asDouble();
            mutex.wait();
            setVisualFeedback(true);
            stateRequest[3] = 1;
            mutex.post();
            timetotStart = Time::now();
            mono = true;
            firstVer = true;
        }
        if(!strcmp(name.c_str(),"SAC_EXPR")) {
            // monocular saccades without visualfeedback
            printf("EXPRESS SACCADE request \n");
            u = arg->get(1).asInt();
            v = arg->get(2).asInt();
            zDistance = arg->get(3).asDouble();
            mutex.wait();
            setVisualFeedback(false);
            stateRequest[3] = 1;
            //executing = false;
            mutex.post();
            timetotStart = Time::now();
            mono = true;
            firstVer = true;
        }
        
        else if(!strcmp(name.c_str(),"SAC_ABS")) {
            xObject = arg->get(1).asDouble();
            yObject = arg->get(2).asDouble();
            zObject = arg->get(3).asDouble();
            printf("received request of abs saccade in position %f %f %f \n", xObject, yObject, zObject);
            mutex.wait();
            stateRequest[3] = 1;
            //executing = false;
            mutex.post();
            mono = false;
        }
        else if(!strcmp(name.c_str(),"PUR")) {
            mutex.wait();
            stateRequest[2] = 1;
            //executing = false;
            mutex.post();
        }
        else if(!strcmp(name.c_str(),"VER_REL")) {
            phi = arg->get(1).asDouble();            
            mutex.wait();
            mono = true;
            stateRequest[1] = 1;
            //executing = false;
            mutex.post();
        }
        else if(!strcmp(name.c_str(),"COR_OFF")) {            
            printf("visual correction disabled \n");
            Time::delay(0.01);
            mutex.wait();
            setVisualFeedback(false);
            mutex.post();
        }
        else if(!strcmp(name.c_str(),"COR_ON")) {   
            printf("visual correction enabled \n");
            Time::delay(0.01);
            mutex.wait();
            setVisualFeedback(true);
            mutex.post();
        }
        else {
            printf("Command has not been recognised \n");
        }
    }
}
*/
