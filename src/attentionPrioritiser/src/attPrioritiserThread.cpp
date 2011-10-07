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


#define THRATE 10
#define PI  3.14159265
#define BASELINE 0.068     // distance in meters between eyes
#define TIMEOUT_CONST 5    // time constant after which the motion is considered not-performed    
#define THCORR 0.99
#define corrStep 10 

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
    collectionLocation = new int[4*2];
    numberState = 4; //null, vergence, smooth pursuit, saccade
    configFile = _configFile;
    firstVer = false;
    visualCorrection = false;
    isOnWings = false;
    onDvs =  false;
    done=true;
    postSaccCorrection = true;
    executing = false;
    correcting = false;
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

    printf("starting the thread.... \n");
    
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
    string rootNameStatus("");rootNameStatus.append(getName("/feedback:o"));
    feedbackPort.open(rootNameStatus.c_str());
    string rootNameOutput("");rootNameOutput.append(getName("/cmd:o"));
    outputPort.open(rootNameOutput.c_str());
    string rootNameTiming("");rootNameTiming.append(getName("/timing:o"));
    timingPort.open(rootNameTiming.c_str());
    string rootNameTemplate("");rootNameTemplate.append(getName("/template:o"));
    templatePort.open(rootNameTemplate.c_str());
    string rootNameDatabase("");rootNameDatabase.append(getName("/database:o"));
    blobDatabasePort.open(rootNameDatabase.c_str());
    string rootNameInhibition("");rootNameInhibition.append(getName("/inhibition:o"));
    inhibitionPort.open(rootNameInhibition.c_str());
    
    inLeftPort.open(getName("/imgMono:i").c_str());
    //inRightPort.open(getName("/matchTracker/img:o").c_str());
    firstConsistencyCheck=true;

    inhibitionImage = new ImageOf<PixelMono>;
    inhibitionImage->resize(320,240);
    inhibitionImage->zero();
    unsigned char* pinhi = inhibitionImage->getRawImage();
    int padding = inhibitionImage->getPadding();
    int rowsizeInhi = inhibitionImage->getRowSize();
    
    string name = getName("");
    sacPlanner = new sacPlannerThread(name);       
    //referencing the image to all the planners
    sacPlanner->referenceRetina(imgLeftIn);
    sacPlanner->start();
     
    return true;
}

void attPrioritiserThread::interrupt() {
    //inCommandPort
    inLeftPort.interrupt();
    inRightPort.interrupt();
    feedbackPort.interrupt();
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
    //Bottle& status = feedbackPort.prepare();
    Bottle& timing = timingPort.prepare();
    //double start = Time::now();
    //printf("stateRequest: %s \n", stateRequest.toString().c_str());
    //mutex.wait();
    //Vector-vector element-wise product operator between stateRequest possible transitions
    if((stateRequest(0) != 0)||(stateRequest(1)!= 0)||(stateRequest(2) != 0)||(stateRequest(3) != 0)) {
        printf("stateRequest: %s \n", stateRequest.toString().c_str());
        printf("state: %s \n", state.toString().c_str());
        Vector c(4);
        c = stateRequest * (state * stateTransition);
        allowedTransitions = orVector(allowedTransitions ,c );
        stateRequest(0) = 0; stateRequest(1) = 0; stateRequest(2) = 0; stateRequest(3) = 0;
        printf("allowedTransitions: %s \n", allowedTransitions.toString().c_str());
    }
    
    /*
    if(inLeftPort.getInputCount()){
       imgLeftIn = inLeftPort.read(false);    
       if(imgLeftIn!=NULL) {
           sacPlanner->referenceRetina(imgLeftIn);
           if(templatePort.getOutputCount()) {
               ImageOf<PixelRgb>& img = templatePort.prepare(); 
               img.resize(252,152);
               img.zero();
               img.copy(*imgLeftIn);
               //copy_8u_C1R(imgLeftIn,&img);
               templatePort.write();
           }
       }       
    }
    */
    
    
    
    //mutex.post();
    //double end = Time::now();
    //double interval = end - start;
    //printf("interval: %f", interval);

    //printf("state: %s \n", state.toString().c_str());
    //printf("allowedTransitions: %s \n", allowedTransitions.toString().c_str());
    
    if(allowedTransitions(3)>0) {
        state(3) = 1 ; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        // ----------------  Express Saccade  -----------------------
        // forcing in idle early processes during oculomotor actions
        // not postsaccadic correction
        printf("------------------ Express Saccade --------------- \n");
            
        if(feedbackPort.getOutputCount()) {
            Bottle* sent = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SUSPEND);
            feedbackPort.write(*sent, *received);
        }

        if(!executing) {                       
            
            collectionLocation[0 + 0] = u;
            collectionLocation[0 * 2 + 1] = v;
            printf("express saccade in position %d %d \n", u,v);
            
            int centroid_x = u;
            int centroid_y = v;
            Bottle& commandBottle=outputPort.prepare();
            commandBottle.clear();
            commandBottle.addString("COR_OFF");
            outputPort.write();
            commandBottle.clear();
            commandBottle.addString("SAC_MONO");
            commandBottle.addInt(centroid_x);
            commandBottle.addInt(centroid_y);
            commandBottle.addDouble(zDistance);
            outputPort.write();
            commandBottle.clear();
            commandBottle.addString("COR_ON");
            outputPort.write();

            //correcting is set when the saccade is accomplished
            while((!correcting)&&(timeout < 2.0)) {
                timeoutStop = Time::now();
                timeout = timeoutStop - timeoutStart;
                Time::delay(0.1);
            }
            if(timeout >= 2.0) {
                printf("Express Saccade timed out \n");
            }
            else {
                printf("Express Saccade  accomplished \n");
            }        

            //Time::delay(3.0);
            
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
                commandBottle.addString("SAC_MONO");
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
            Bottle* sent = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_RESUME);
            feedbackPort.write(*sent, *received);
        }        
    }    
    else if(allowedTransitions(2)>0) {
        state(3) = 0 ; state(2) = 1 ; state(1) = 0 ; state(0) = 0;
        //forcing in idle early processes during oculomotor actions
        if(feedbackPort.getOutputCount()) {
            Bottle* sent = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SUSPEND);
            feedbackPort.write(*sent, *received);
        }

        // ----------------  Planned Saccade  -----------------------
        if(!executing) {                       
            printf("----------------------- Planned Saccade ------------------- \n");
            printf("initialising the planner thread %f \n", time);
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
            // activating the sacPlanner
            sacPlanner->setSaccadicTarget(u,v);
            sacPlanner->wakeup();
            Time::delay(0.1);
            
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
                // wait for accomplished saccadic event
                timeout = 0;
                timeoutStart = Time::now();
                while((!correcting)&&(timeout < 5.0)) {
                    timeoutStop = Time::now();
                    timeout = timeoutStop - timeoutStart;
                    Time::delay(0.1);
                }
                if(timeout > 5.0) {
                    printf("Saccade accomplished timeout \n");
                }
                else {
                    printf("Saccade accomplished command received \n");
                }
                correcting = false;   // resetting the correction flag
                sacPlanner->setCompare(true);
                sacPlanner->wakeup();
                Time::delay(0.1);
                
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

                Time::delay(2.0);
                
            } //end of postsaccadic correction                
        }
        
        //resume early processes
        //printf("          SENDING COMMAND OF RESUME      \n");
        if(feedbackPort.getOutputCount()) {
            //printf("feedback resetting \n");
            Bottle* sent = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_RESUME);
            feedbackPort.write(*sent, *received);
        }
    }
    else if(allowedTransitions(1)>0) {
        state(3) = 0 ; state(2) = 0 ; state(1) = 1 ; state(0) = 0;
        // ----------------  Smooth Pursuit  -----------------------
        if(!executing) {                       
            printf("------------- Smooth Pursuit ---------------------\n");
        }
    }
    else if(allowedTransitions(0)>0) {
        // ----------------  No Action - StandBy  -----------------------
        state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;
    }
    else {
        //printf("No transition \n");
    }
    
    //printf("--------------------------------------------------------->%d \n",done);
            
    if(allowedTransitions(3)>0) {
        mutex.wait();
        state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(3) = 0;
        executing = false;  //executing=false allows new action commands
        // execution = false moved to after the SAC_ACC is received
        printf ("Transition request 3 reset \n");
        mutex.post();
    }
    if(allowedTransitions(2)>0) {
        mutex.wait();
        state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(2) = 0;
        executing = false;
        printf ("Transition request 2 reset \n");
        mutex.post();
    }
    if(allowedTransitions(1)>0) {
        mutex.wait();
        state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(1) = 0;
        executing = false;
        printf ("Transition request 1 reset \n");
        mutex.post();
    }
}

void attPrioritiserThread::update(observable* o, Bottle * arg) {
    printf("ACK. Aware of observable asking for attention \n");
    if (arg != 0) {
        //printf("bottle: %s ", arg->toString().c_str());
        int size = arg->size();
        ConstString name = arg->get(0).asString();
        
        if(!strcmp(name.c_str(),"SAC_MONO")) {
            u = arg->get(1).asInt();
            v = arg->get(2).asInt();
            zDistance = arg->get(3).asDouble();
            time =  arg->get(4).asDouble();
            printf("saccade mono time: %f \n", time);
            mutex.wait();
            if(time <= 0.5) {
                printf("setting stateRequest[3] \n");
                stateRequest[3] = 1;
                timeoutStart = Time::now();
            } 
            else {
                printf("setting stateRequest[2] \n");
                stateRequest[2] = 1;
            }
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
            stateRequest[2] = 1;
            //executing = false;
            mutex.post();
            mono = false;
        }
        else if(!strcmp(name.c_str(),"PUR")) {
            mutex.wait();
            stateRequest[1] = 1;
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
        else if(!strcmp(name.c_str(),"SAC_ACC")) {
            // saccade accomplished           
            mutex.wait();
            correcting = true;
            //executing = false;
            mutex.post();
        }
        else {
            printf("Command has not been recognised \n");
        }
    }
}


void attPrioritiserThread::threadRelease() {
    inLeftPort.close();
    printf("closing right port \n");
    inRightPort.close();
    printf("closing feedback port \n");
    feedbackPort.close();
    printf("closing template port \n");
    templatePort.close();
    printf("closing database port \n");
    blobDatabasePort.close();
    printf("closing inhibition port \n");
    inhibitionPort.close();
    timingPort.close();
    printf("successfully closed all the ports \n");
    delete eyeL;
    delete eyeR;
    printf("successfully deleted eyes references \n");
    //igaze->restoreContext(originalContext);
    printf("successfully restored previous gaze context \n");
    
    //delete clientGazeCtrl;
    printf("deleting the clientPlanner \n");
    sacPlanner->stop();
    delete sacPlanner;
    printf("deleting the sacPlanner \n");
}
