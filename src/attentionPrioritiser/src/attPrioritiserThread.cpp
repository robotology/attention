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
    cUpdate = 0;
    collectionLocation = new int[4*2];
    numberState = 5; //null, vergence, smooth pursuit, saccade
    configFile = _configFile;
    // boolean values
    firstVer           = false;
    firstVergence      = true;
    done               = true;
    reinfFootprint     = true;
    postSaccCorrection = false;
    executing          = false;
    correcting         = false;

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
    kColOri[2] = 0.40;  // chrominance
    kColOri[3] = 0.00;  // orientation
    kColOri[4] = 0.00;  // edges
    kColOri[5] = 0.40;  // proto-objects
    //tColOri    = 5000;

    Matrix trans(5,5);
    trans(0,0) = 1.0 ; trans(0,1) = 1.0 ; trans(0,2) = 1.0 ; trans(0,3) = 1.0; trans(0,4) = 1.0;
    trans(1,0) = 1.0 ; trans(1,1) = 1.0 ; trans(1,2) = 1.0 ; trans(1,3) = 1.0; trans(0,4) = 1.0;
    trans(2,0) = 1.0 ; trans(2,1) = 1.0 ; trans(2,2) = 1.0 ; trans(2,3) = 1.0; trans(0,4) = 1.0;
    trans(3,0) = 1.0 ; trans(3,1) = 1.0 ; trans(3,2) = 0.0 ; trans(3,3) = 1.0; trans(0,4) = 1.0;
    trans(4,0) = 1.0 ; trans(4,1) = 1.0 ; trans(4,2) = 0.0 ; trans(4,3) = 1.0; trans(0,4) = 1.0;
    stateTransition=trans;

    Vector req(5);
    req(0) = 0;
    req(1) = 0;
    req(2) = 0;
    req(3) = 0;
    req(4) = 0;
    stateRequest = req;
    allowedTransitions = req;

    Vector s(5);
    s(0) = 1;
    s(1) = 0;
    s(2) = 0;
    s(3) = 0;
    s(4) = 0;
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
    tracker->stop();
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
    string nameFBEarlyVision("");nameFBEarlyVision.append(getName("/earlyVision:o"));
    feedbackEarlyVision.open(nameFBEarlyVision.c_str());
    string nameFBSelective("");nameFBSelective.append(getName("/selectiveAtt:o"));
    feedbackSelective.open(nameFBSelective.c_str());    
    string nameFBProtoObject("");nameFBProtoObject.append(getName("/protoObject:o"));
    feedbackProtoObject.open(nameFBProtoObject.c_str());

    inLeftPort.open(getName("/imgMono:i").c_str());
    //inRightPort.open(getName("/matchTracker/img:o").c_str());
    //firstConsistencyCheck=true;

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
    //interrupting ports
    inLeftPort.interrupt();
    inRightPort.interrupt();
    feedbackPort.interrupt();
    templatePort.interrupt();
    inhibitionPort.interrupt();
    blobDatabasePort.interrupt();
    templatePort.interrupt();
    timingPort.interrupt();
    feedbackEarlyVision.interrupt();
    feedbackSelective.interrupt();
    feedbackProtoObject.interrupt();
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
    printf("closing feedback ports \n");
    feedbackEarlyVision.close();
    feedbackSelective.close();
    feedbackProtoObject.close();
    
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
    if((stateRequest(0) != 0)||(stateRequest(1)!= 0)||(stateRequest(2) != 0)||(stateRequest(3) != 0)||(stateRequest(4) != 0)) {
        //printf("stateRequest: %s \n", stateRequest.toString().c_str());
        //printf("state: %s \n", state.toString().c_str());
        Vector c(5);
        c = stateRequest * (state * stateTransition);
        allowedTransitions = orVector(allowedTransitions ,c );
        // resetting the requests
        stateRequest(0) = 0; stateRequest(1) = 0; stateRequest(2) = 0; stateRequest(3) = 0; stateRequest(4) = 0;
        //printf("allowedTransitions: %s \n", allowedTransitions.toString().c_str());
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
    
    if(allowedTransitions(4)>0) {
        state(4) = 1 ; state(3) = 0; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        // ----------------  Express Saccade  -----------------------
        // forcing in idle early processes during oculomotor actions
        // not postsaccadic correction
        printf("------------------ Express Saccade --------------- \n");
            
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
            correcting =  false;
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
            }
            else {
                printf("Express Saccade  accomplished \n");
            }        

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
    else if(allowedTransitions(3)>0) {
        state(4) = 0 ; state(3) = 1 ; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        //forcing in idle early processes during oculomotor actions
        if(feedbackPort.getOutputCount()) {
            Bottle* sent = new Bottle();
            Bottle* received = new Bottle();    
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_INH);
            feedbackPort.write(*sent, *received);
        }

        // ----------------  Planned Saccade  -----------------------
        if(!executing) {                       
            printf("\n \n ____________________ Planned Saccade ___________________ \n");
            if((u==-1)||(v==-1)) {
                printf("----------- Stereo Planned Saccade ------------  \n");
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
                printf("------- Monocular Planned Saccade -------------  \n");
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
                    }
                    else {
                        printf("Saccade accomplished command received \n");
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

        Time::delay(0.005);
    }
    else if(allowedTransitions(2)>0) {
        state(4) = 0 ; state(3) = 0 ; state(2) = 1 ; state(1) = 0 ; state(0) = 0;
        // ----------------  Smooth Pursuit  -----------------------
        if(!executing) {                       
            printf("------------- Smooth Pursuit ---------------------\n");
            printf("_____________ Smooth Pursuit _____________________\n");
        }
    }
    else if(allowedTransitions(1)>0) {
        // ----------------  Vergence  -----------------------
        state(4) = 0 ; state(3) = 0 ; state(2) = 0 ; state(1) = 1; state(0) = 0;
        //printf("------------------ Vergence --------------- \n");
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
                //printf("vergence: sending relative angle to the gazeArbiter \n");
                bool port_is_writing;
                Bottle& commandBottle = outputPort.prepare();
                commandBottle.clear();
                commandBottle.addString("VER_REL");
                commandBottle.addDouble(phi);
                commandBottle.addDouble(phi2);
                commandBottle.addDouble(phi3);
                outputPort.write();
            }                        
        }
    
    }
    else {
        //printf("No transition \n");
    }
    
    //printf("--------------------------------------------------------->%d \n",done);
            
    if(allowedTransitions(4)>0) { //express saccade
        mutex.wait();
        state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(4) = 0;
        executing = false;  //executing=false allows new action commands
        // execution = false moved to after the SAC_ACC is received
        //printf ("Transition request 4 reset \n");
        mutex.post();
    }
    if(allowedTransitions(3)>0) { //planned saccade
        mutex.wait();
        state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(3) = 0;
        executing = false;
        //printf ("Transition request 3 reset \n");
        mutex.post();
    }
    if(allowedTransitions(2)>0) { //smooth pursuit
        mutex.wait();
        state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(2) = 0;
        executing = false;
        //printf ("Transition request 2 reset \n");
        mutex.post();
    }
    if(allowedTransitions(1)>0) { //vergence
        mutex.wait();
        state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;   
        allowedTransitions(1) = 0;
        executing = false;
        //printf ("Transition request 1 reset \n");
        mutex.post();
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
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_TIME);
    sent->addDouble(tNull);
    feedbackSelective.write(*sent, *received);
    
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
            sent->clear();
            sent->addVocab(COMMAND_VOCAB_SET);
            sent->addVocab(COMMAND_VOCAB_TIME);
            sent->addDouble(tNull);
            feedbackSelective.write(*sent, *received);
            
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
    printf("SELF REINFORCEMENT \n");
    printf("SELF REINFORCEMENT \n");
    printf("SELF REINFORCEMENT \n");
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
    sent->clear();
    sent->addVocab(COMMAND_VOCAB_SET);
    sent->addVocab(COMMAND_VOCAB_TIME);
    sent->addDouble(tColOri);
    feedbackSelective.write(*sent, *received);
    
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
            zDistance = arg->get(3).asDouble();
            time =  arg->get(4).asDouble();
            printf("saccade mono time: %f \n", time);
            mutex.wait();
            if(time <= 0.5) {
                // express saccade
                printf("setting stateRequest[4] \n");
                reinfFootprint = true;
                stateRequest[4] = 1;
                timeoutStart = Time::now();
            } 
            else {
                // feedback saccade
                // checking first for reinfFootPrint
                if(!reinfFootprint) {
                    // reinforceFootprint already happened
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
                        printf("setting stateRequest[3] \n");
                        Time::delay(1);
                        stateRequest[3] = 1;
                        reinfFootprint  = true;   // enabling back the control top-down footprint extraction
                    }
                }
                else {
                    //reinforceFootprint has not happened yet
                    printf("setting stateRequest[3] \n");
                    stateRequest[3] = 1;
                }
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
            u = -1;
            v = -1;
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
            phi  = arg->get(1).asDouble();
            phi2 = arg->get(2).asDouble();
            phi3 = arg->get(3).asDouble();
            //printf("vergence command received %d \n", firstVergence);
            if(firstVergence){
                //if((phi!=0)){                    
                //printf("inside the vergence command \n");
                mutex.wait();
                ver_accomplished = false;
                stateRequest[1]  = 1;
                //executing = false;
                mutex.post();
                //}
            }
            else {
                mutex.wait();
                ver_accomplished = false;
                stateRequest[1]  = 1;
                //executing = false;
                mutex.post();
            }            
        }
        else if(!strcmp(name.c_str(),"SAC_ACC")) {
            // saccade accomplished           
            mutex.wait();
            correcting = true;
            //executing = false;
            mutex.post();

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
            }

            if(reinfFootprint) {
                
                reinforceFootprint();
                Time::delay(5);
            }
        }
        else if(!strcmp(name.c_str(),"VER_ACC")) {
            // vergence accomplished           
            //printf("Vergence accomplished \n");
            mutex.wait();
            ver_accomplished = true;
            stateRequest[1]  = 1;
            //executing = false;
            mutex.post();
        }
        else {
            printf("Command %s has not been recognised \n",name.c_str());
        }
    }
}


