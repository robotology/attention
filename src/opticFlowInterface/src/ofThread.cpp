// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file ofThread.cpp
 * @brief Implementation of the optic flow thread(see header ofThread.h)
 */

#include <iCub/ofThread.h>
#include <yarp/math/Math.h>
#include <yarp/math/SVD.h>
#include <cstring>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace std;
using namespace yarp::math;

#define THRATE 10
#define PI  3.14159265
#define BASELINE 0.068     // distance in meters between eyes
#define TIMEOUT_CONST 5    // time constant after which the motion is considered not-performed    
#define INHIB_WIDTH 320
#define INHIB_HEIGHT 240
 
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


ofThread::ofThread(string _configFile) : RateThread(THRATE) {
    numberState  = 4; //null, vergence, smooth pursuit, saccade
    countVerNull = 0;
    configFile = _configFile;
    
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

ofThread::~ofThread() {
    // MUST BE REMOVED THE RF AND TRACKER ALLOCATED IN THE CONSTRUCTOR
}

bool ofThread::threadInit() {
    done=true;
    executing = false;
    printf("starting the thread.... \n");
  
    template_size = 20;
    search_size = 100;
    point.x = INHIB_WIDTH;
    point.y = INHIB_HEIGHT;

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
    inLeftPort.open(getName("/of/imgMono:i").c_str());
    //inRightPort.open(getName("/matchTracker/img:o").c_str());
    firstConsistencyCheck=true;

    inhibitionImage = new ImageOf<PixelMono>;
    inhibitionImage->resize(INHIB_WIDTH,INHIB_HEIGHT);
    inhibitionImage->zero();
    unsigned char* pinhi = inhibitionImage->getRawImage();
    int padding          = inhibitionImage->getPadding();
    int rowsizeInhi      = inhibitionImage->getRowSize();
    int ym = INHIB_HEIGHT >> 1;
    int xm = INHIB_WIDTH  >> 1;
    

    printf("starting the tracker.... \n");
    ResourceFinder* rf = new ResourceFinder();
    tracker = new trackerThread(*rf);
    tracker->setName(getName("/matchTracker").c_str());
    tracker->start();
    printf("tracker successfully started \n");

    return true;
}

void ofThread::interrupt() {
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

void ofThread::setDimension(int w, int h) {
    width = w;
    height = h;
}

void ofThread::setBlockPitch(double value) {
    blockNeckPitchValue = value;
}

void ofThread::setName(string str) {
    this->name=str;
    printf("name: %s \n", name.c_str());
}

std::string ofThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void ofThread::setRobotName(string str) {
    this->robot = str;
    printf("name: %s \n", name.c_str());
}

void ofThread::init(const int x, const int y) {
    point.x = x;
    point.y = y;
    template_roi.width = template_roi.height = template_size;
    search_roi.width = search_roi.height = search_size;
}

void ofThread::getPoint(CvPoint& p) {
    //tracker->getPoint(p);
}


void ofThread::run() {
    Bottle& status = statusPort.prepare();
    Bottle& timing = timingPort.prepare();
    //double start = Time::now();
}

void ofThread::threadRelease() {
    inLeftPort.close();
    inRightPort.close();
    statusPort.close();
    templatePort.close();
    blobDatabasePort.close();
    inhibitionPort.close();
    timingPort.close();
    tracker->stop();
}

void ofThread::update(observable* o, Bottle * arg) {
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

 


