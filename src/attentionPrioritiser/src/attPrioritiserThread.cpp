// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * Public License fo
 r more details
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


#define THRATE 10
#define PI  3.14159265
#define BASELINE 0.068     // distance in meters between eyes
#define TIMEOUT_CONST 5    // time constant after which the motion is considered not-performed    
 
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
    numberState = 4; //null, vergence, smooth pursuit, saccade
    configFile = _configFile;
    firstVer = false;
    visualCorrection = false;
    isOnWings = false;
    onDvs =  false;
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

    printf("starting the tracker.... \n");
    ResourceFinder* rf = new ResourceFinder();
    tracker = new trackerThread(*rf);
    tracker->start();
    printf("tracker successfully started \n");
}

attPrioritiserThread::~attPrioritiserThread() {
    // MUST BE REMOVED THE RF AND TRACKER ALLOCATED IN THE CONSTRUCTOR
}

bool attPrioritiserThread::threadInit() {
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

    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        Matrix &Prj = *PrjL;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    
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

  
    template_size = 20;
    search_size = 100;
    point.x = 320;
    point.y = 240;

    template_roi.width = template_roi.height = template_size;
    search_roi.width   = search_roi.height   = search_size;

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
    inLeftPort.open(getName("/attPrioritiser/imgMono:i").c_str());
    //inRightPort.open(getName("/matchTracker/img:o").c_str());
    firstConsistencyCheck=true;

    inhibitionImage = new ImageOf<PixelMono>;
    inhibitionImage->resize(320,240);
    inhibitionImage->zero();
    unsigned char* pinhi = inhibitionImage->getRawImage();
    int padding = inhibitionImage->getPadding();
    int rowsizeInhi = inhibitionImage->getRowSize();
    int ym = 240>>1;
    int xm = 320>>1;
    //calculating the peek value
    int dx = 30.0;
    int dy = 30.0;
    double sx = (dx / 2) / 3 ; //0.99 percentile
    double sy = (dy / 2) / 3 ;
    double vx = 10; //sx * sx; // variance          
    double vy = 10; //sy * sy;
    
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
                z = (1 / 1.645062) * z;
                //z = 0.5;
            }
            
            // restrincting the z gain between two thresholds
            if (z > 1) {
                z = 1;
            }
            //if (z < 0.3) {
            //    z = 0.3;
            //}
            
            
            //set the image 
            *pinhi++ = 255 * z;                    
        }
        pinhi += rowsizeInhi - (dx + 1) ;
    }

    printf("zmax = %f \n", zmax);

    //pinhi = inhibitionImage->getRawImage();
    //*pinhi = 255;
    //pinhi += rowsizeInhi - 1;
    //*pinhi = 255;
    //pinhi += 1 + rowsizeInhi * 230;
    //*pinhi = 255;
    //pinhi += rowsizeInhi - 1;
    //*pinhi = 255;


    //for(int y = 0; y < 240; y++) {
    //    for(int x = 0;x < 320; x++) {
    //        if((x > 160-20) && (x < 160+20) && (y > 120-20) && (y<120+20))
    //            *pinhi++ = (unsigned char) 255;
    //        else
    //            *pinhi++ = (unsigned char) 0;
    //   }
    //    pinhi += padding;
    //}
    
    return true;
}

void attPrioritiserThread::interrupt() {
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


void attPrioritiserThread::run() {

    Bottle& status = statusPort.prepare();
    Bottle& timing = timingPort.prepare();
    //double start = Time::now();
    //printf("stateRequest: %s \n", stateRequest.toString().c_str());
    //mutex.wait();
    //Vector-vector element-wise product operator between stateRequest possible transitions
    if((stateRequest(0) != 0)||(stateRequest(1)!= 0)||(stateRequest(2) != 0)||(stateRequest(3) != 0)) {
        Vector c(4);
        c = stateRequest * (state * stateTransition);
        allowedTransitions = orVector(allowedTransitions ,c );
        stateRequest(0) = 0; stateRequest(1) = 0; stateRequest(2) = 0; stateRequest(3) = 0;
    }

    if(inLeftPort.getInputCount()){
       imgLeftIn = inLeftPort.read(false);
    }
    
    
    //mutex.post();
    //double end = Time::now();
    //double interval = end - start;
    //printf("interval: %f", interval);

    //printf("state: %s \n", state.toString().c_str());
    //printf("allowedTransitions: %s \n", allowedTransitions.toString().c_str());
    
    if(allowedTransitions(3)>0) {
        state(3) = 1 ; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        // ----------------  SACCADE -----------------------
        if(!executing) {                       

        }
    }
    else if(allowedTransitions(2)>0) {
        state(3) = 0 ; state(2) = 1 ; state(1) = 0 ; state(0) = 0;
    }
    else if(allowedTransitions(1)>0) {
        state(3) = 0 ; state(2) = 0 ; state(1) = 1 ; state(0) = 0;
       
        if(!executing) {
            
        }
    }
    else if(allowedTransitions(0)>0) {
        state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;
    }
    else {
        printf("No transition \n");
    }

    
    //printf("--------------------------------------------------------->%d \n",done);
            
    if(allowedTransitions(3)>0) {      
        mutex.wait();
        allowedTransitions(3) = 0;
        executing = false;  //executing=false allows new action commands
        printf ("\n\n\n\n\n\n\n\n\n");
        mutex.post();        
    }
    if(allowedTransitions(2)>0) {
        mutex.wait();
        allowedTransitions(2) = 0;
        executing = false;
        mutex.post();
    }
    if(allowedTransitions(1)>0) {
        mutex.wait();
        allowedTransitions(1) = 0;
        executing = false;
        //printf ("\n\n\n\n\n\n\n\n\n");
        mutex.post();
    }
}

void attPrioritiserThread::threadRelease() {
    inLeftPort.close();
    inRightPort.close();
    statusPort.close();
    templatePort.close();
    blobDatabasePort.close();
    inhibitionPort.close();
    timingPort.close();
    delete eyeL;
    delete eyeR;
    igaze->restoreContext(originalContext);
    delete clientGazeCtrl;
}

void attPrioritiserThread::update(observable* o, Bottle * arg) {
    //printf("ACK. Aware of observable asking for attention \n");
    if (arg != 0) {
        //printf("bottle: %s ", arg->toString().c_str());
        int size = arg->size();
        ConstString name = arg->get(0).asString();
        
        if(!strcmp(name.c_str(),"SAC_MONO")) {
            u = arg->get(1).asInt();
            v = arg->get(2).asInt();
            zDistance = arg->get(3).asDouble();
            mutex.wait();
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
            stateRequest[1] = 1;
            //executing = false;
            mutex.post();
        }
        else {
            printf("Command has not been recognised \n");
        }
    }
}

 


