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
 * @file gazeCollectorThread.cpp
 * @brief Implementation of the gaze arbiter thread(see header gazeArbiterThread.h)
 */

#include <iCub/gazeArbiterThread.h>
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
#define BASELINE 0.068      // distance in meters between eyes
#define TIMEOUT_CONST 3    // time constant after which the motion is considered not-performed    
#define INHIB_WIDTH 320
#define INHIB_HEIGHT 240

#define CONFIGFOVEA
 
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
                //double cx = parType.find("w").asDouble() / 2.0;
                //double cy = parType.find("h").asDouble() / 2.0;
                // we suppose that the centerof ditortion is NOT    compensated
                double cx = parType.find("cx").asDouble();
                double cy = parType.find("cy").asDouble();

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


gazeArbiterThread::gazeArbiterThread(string _configFile) : RateThread(THRATE) {
    numberState  = 4; //null, vergence, smooth pursuit, saccade
    countVerNull = 0;
    camSel       = 0;
    configFile = _configFile;
    
    //boolean flag initialisation
    firstVer            = false;
    availableVisualCorr = false;
    visualCorrection    = true;
    isOnWings           = false;
    onDvs               = false;
    firstVergence       = true;
    
    countRegVerg = 0;
    sumRegVerg   = 0;
    phiTOT       = 0;
    xOffset      = 0;
    yOffset      = 0; 
    zOffset      = 0;
    blockNeckPitchValue =-1;

    Matrix trans(5,5);
    trans(0,0) = 1.0 ; trans(0,1) = 1.0 ; trans(0,2) = 1.0 ; trans(0,3) = 1.0; trans(0,4) = 1.0;
    trans(1,0) = 1.0 ; trans(1,1) = 1.0 ; trans(1,2) = 1.0 ; trans(1,3) = 1.0; trans(1,4) = 1.0;
    trans(2,0) = 1.0 ; trans(2,1) = 1.0 ; trans(2,2) = 1.0 ; trans(2,3) = 1.0; trans(2,4) = 1.0;
    trans(3,0) = 1.0 ; trans(3,1) = 1.0 ; trans(3,2) = 1.0 ; trans(3,3) = 1.0; trans(3,4) = 1.0;
    trans(4,0) = 1.0 ; trans(4,1) = 1.0 ; trans(4,2) = 1.0 ; trans(4,3) = 1.0; trans(4,4) = 1.0;
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
    s(0)  = 1;
    s(1)  = 0;
    s(2)  = 0;
    s(3)  = 0;
    s(4)  = 0;
    state = s;
    
    Vector t(3);
    t(0) = -0.6;
    t(1) = 0;
    t(2) = 0.6;
    xFix = t;

    printf("extracting kinematic informations \n");
}

gazeArbiterThread::~gazeArbiterThread() {
    // MUST BE REMOVED THE RF AND TRACKER ALLOCATED IN THE CONSTRUCTOR
}

bool gazeArbiterThread::threadInit() {
    done=true;
    executing = false;
    printf("starting the thread.... \n");

    //initializing gazecontrollerclient
    printf("initialising gazeControllerClient \n");
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
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

    // -----------------------------------------------------------------

    Bottle info;
    igaze->getInfo(info);
    printf("just got the info \n");    
    int head_version = info.check("head_version", Value(1)).asInt();
    
    printf("head_version extracted from gazeArbiter \n");

    if(head_version == 1) {
        eyeL = new iCubEye("left");
        eyeR = new iCubEye("right");
    }
    else {
        eyeL = new iCubEye("left_v2");
        eyeR = new iCubEye("right_v2");
    }
    printf("correctly istantiated the head \n");

    
    // remove constraints on the links
    // we use the chains for logging purpose
    // eyeL->setAllConstraints(false);
    // eyeR->setAllConstraints(false);

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
    printf("get Camera configuration \n");
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        Matrix &Prj = *PrjL;
        cxl=Prj(0,2);
        cyl=Prj(1,2);
        printf("pixel fovea in the config file %d %d \n", cxl,cyl);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }

    //--------------------------------------------------------------------

    
    string headPort = "/" + robot + "/head";
    string nameLocal("gazeArbiter");

    //initialising the head polydriver
    optionsHead.put("device", "remote_controlboard");
    optionsHead.put("local", ("/"+nameLocal+"/localhead").c_str());
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
    point.x = INHIB_WIDTH;
    point.y = INHIB_HEIGHT;

    template_roi.width = template_roi.height = template_size;
    search_roi.width   = search_roi.height   = search_size;

    //opening port section 
    string rootNameStatus("");rootNameStatus.append(getName("/status:o"));
    statusPort.open(rootNameStatus.c_str());
    string rootNameEgoMotion("");rootNameEgoMotion.append(getName("/egoMotion:o"));
    egoMotionFeedback.open(rootNameEgoMotion.c_str());
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

    printf("starting the trackers.... \n");
    // active agent for countinuos tracking
    ResourceFinder* rf = new ResourceFinder();
    tracker = new trackerThread(*rf);
    tracker->setName(getName("/matchTracker").c_str());
    tracker->start();
    printf("tracker successfully started \n");
    
    //active agent for periodic tracking
    ptracker = new periodicTrackerThread(*rf);
    ptracker->setName(getName("/periodicTracker").c_str());
    ptracker->start();
    printf("periodic tracker successfully started \n");
    ptracker->setUpdate(true);
    ptracker->init(160,120);
    ptracker->setUpdate(false);

    // active agent for velocity control
    printf("starting the velocity controller \n");
    velControl = new velocityController();
    velControl->setName(getName("/velControl").c_str());
    velControl->start();
    printf("velocity controller successfully started \n");

    return true;
}

void gazeArbiterThread::interrupt() {
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

void gazeArbiterThread::setDimension(int w, int h) {
    width = w;
    height = h;
}

void gazeArbiterThread::setBlockPitch(double value) {
    blockNeckPitchValue = value;
}

void gazeArbiterThread::setName(string str) {
    this->name=str;
    printf("name: %s \n", name.c_str());
}

std::string gazeArbiterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void gazeArbiterThread::setRobotName(string str) {
    this->robot = str;
    printf("name: %s \n", name.c_str());
}

void gazeArbiterThread::init(const int x, const int y) {
    point.x = x;
    point.y = y;
    template_roi.width = template_roi.height = template_size;
    search_roi.width = search_roi.height = search_size;
}

void gazeArbiterThread::getPoint(CvPoint& p) {
    //tracker->getPoint(p);
}


void gazeArbiterThread::interfaceIOR(Bottle& timing) {
    Time::delay(3.0);
    
    //code for accomplished vergence
    timetotStop = Time::now();
    timetot = timetotStop - timetotStart;
    timing = timingPort.prepare();
    timing.clear();
    timing.addDouble(timetot);
    timingPort.write();
    
    accomplished_flag = true;
    countVerNull = 0;
        
    //sending an image for inhibition of return                     
    if(imgLeftIn!=NULL){
        unsigned char* pinhi = inhibitionImage->getRawImage();
        inhibitionImage->resize(320,240);
        //inhibitionImage->zero();
        /*
          int padding = inhibitionImage->getPadding();
          int rowsizeInhi = inhibitionImage->getRowSize();
          int ym = 240>>1;
          int xm = 320>>1;
          //calculating the peek value
          int dx = 50.0;
          int dy = 50.0;
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
          z = (1 / 1.62) * z;
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
        */
        
        
        pinhi = inhibitionImage->getRawImage();
        
        //printf("copying the image \n");
        //unsigned char* pinLeft = imgLeftIn->getRawImage();
        //int padding  = inhibitionImage->getPadding();
        //int padding3 = imgLeftIn->getPadding(); 
        //for ( int row = 0 ; row < 240; row++) { 
        //    for (int cols = 0; cols< 320; cols++) {
        //        *pinhi = (unsigned char) floor(0.85 * *pinLeft + 0.15 * *pinhi);  //red
        //        //*portion++ = *mosaic++;  //green
        //*portion++ = *mosaic++;  //blue
        //        pinhi++; pinLeft+=3;
        //    }
        //    pinhi += padding;
        //    pinLeft += padding3;
        //}
    }
    inhibitionPort.prepare() = *inhibitionImage;
    inhibitionPort.write();                    
    
    //calculating the 3d position and sending it to database
    u = 160; 
    v = 120;
    Vector fp(3);
    
    
    Vector torso(3);
    encTorso->getEncoder(0,&torso[0]);
    encTorso->getEncoder(1,&torso[1]);
    encTorso->getEncoder(2,&torso[2]);
    Vector head(5);
    encHead->getEncoder(0,&head[0]);
    encHead->getEncoder(1,&head[1]);
    encHead->getEncoder(2,&head[2]);
    encHead->getEncoder(3,&head[3]);
    encHead->getEncoder(4,&head[4]);
    
    
    Vector q(8);
    double ratio = M_PI /180;
    q[0]=torso[0] * ratio;
    q[1]=torso[1]* ratio;
    q[2]=torso[2]* ratio;
    q[3]=head[0]* ratio;
    q[4]=head[1]* ratio;
    q[5]=head[2]* ratio;
    q[6]=head[3]* ratio;
    q[7]=head[4]* ratio;
    double ver = head[5];                    
                            
    Vector x(3);
    printf("varDistance %f \n", varDistance);
    x[0]=varDistance * u;   //epipolar correction excluded the focal lenght
    x[1]=varDistance * v;
    x[2]=varDistance;
    
    // find the 3D position from the 2D projection,
    // knowing the distance z from the camera
    Vector xe = yarp::math::operator *(*invPrjL, x);
    xe[3]=1.0;  // impose homogeneous coordinates                
    
    // update position wrt the root frame
    Matrix eyeH = eyeL->getH(q);
    //printf(" %f %f %f ", eyeH(0,0), eyeH(0,1), eyeH(0,2));
    Vector xo = yarp::math::operator *(eyeH,xe);

    printf("object %f,%f,%f \n",xo[0],xo[1],xo[2]);    
    
    //adding novel position to the 
    Bottle request, reply;
    request.clear(); reply.clear();
    request.addVocab(VOCAB3('a','d','d'));
    Bottle& listAttr=request.addList();
    
    Bottle& sublistX = listAttr.addList();
    
    sublistX.addString("x");
    sublistX.addDouble(xo[0] * 1000);    
    listAttr.append(sublistX);
    
    Bottle& sublistY = listAttr.addList();
    sublistY.addString("y");
    sublistY.addDouble(xo[1] * 1000);      
    listAttr.append(sublistY);
    
    Bottle& sublistZ = listAttr.addList();            
    sublistZ.addString("z");
    sublistZ.addDouble(xo[2] * 1000);   
    listAttr.append(sublistZ);
    
    Bottle& sublistR = listAttr.addList();
    sublistR.addString("r");
    sublistR.addDouble(255.0);
    listAttr.append(sublistR);
                    
    Bottle& sublistG = listAttr.addList();
    sublistG.addString("g");
    sublistG.addDouble(255.0);
    listAttr.append(sublistG);
                        
    Bottle& sublistB = listAttr.addList();
    sublistB.addString("b");
    sublistB.addDouble(255.0);
    listAttr.append(sublistB);
    
    Bottle& sublistLife = listAttr.addList();
    sublistLife.addString("lifeTimer");
    sublistLife.addDouble(1.0);
    listAttr.append(sublistLife);          
    
    if (templatePort.getInputCount()) {
        Bottle& templateList = listAttr.addList();
        printf("attaching the blob to the item in the list");
        templateImage = templatePort.read(false);
        if(templateImage!=0) {
            int width = templateImage->width();
            int height = templateImage->height();
            printf("template dim %d %d \n", width, height);
            unsigned char* pointerTemplate = templateImage->getRawImage();
            int padding = templateImage->getPadding();
            templateList.addString("texture");
            Bottle& pixelList = templateList.addList();
            pixelList.addInt(width);
            pixelList.addInt(height);
            
            for (int r = 0; r < height ; r++) {
                for (int c = 0; c < width; c++) {
                    pixelList.addInt((unsigned char)*pointerTemplate++);
                    //pixelList.addInt(r + c);
                }
                pointerTemplate += padding;
            }
        }
    }
    
    blobDatabasePort.write(request, reply);                     
    
    //delay after vergence accomplished ... needed to allow other module to call the control
    Time::delay(0.01);
    return;
}


void gazeArbiterThread::run() {
    //visualCorrection =false; // TODO : check why this should be forced in leaning algorithm
    Bottle& status = statusPort.prepare();
    Bottle& timing = timingPort.prepare();
    //double start = Time::now();
    
    //mutex.wait();
    //Vector-vector element-wise product operator between stateRequest possible transitions
    if((stateRequest(0) != 0)||(stateRequest(1)!= 0)||(stateRequest(2) != 0)||(stateRequest(3) != 0)||(stateRequest(4) != 0)) {
        printf("stateRequest: %s \n", stateRequest.toString().c_str());
        Vector c(5);
        c = stateRequest * (state * stateTransition);
        allowedTransitions = orVector(allowedTransitions ,c );
        stateRequest(0) = 0; stateRequest(1) = 0; stateRequest(2) = 0; stateRequest(3) = 0; stateRequest(4) = 0;
        printf("state: %s \n", state.toString().c_str());
        printf("allowedTransitions: %s \n", allowedTransitions.toString().c_str());
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
    
    if(allowedTransitions(4)>0) {
        state(4) = 1; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        // ----------------  SACCADE -----------------------
        if(!executing) {                       
            // starting saccade toward the direction of the required position
            // needed timeout because controller kept stucking whenever a difficult position could not be reached
            
            timeoutStart = Time::now();
            if(mono){
                printf("mono saccade activated \n");
                //calculating where the fixation point would end up
                executing = true;
                bool isLeft = true;  // TODO : the left drive is hardcoded but in the future might be either left or right
                
                Matrix  *invPrj = (isLeft?invPrjL:invPrjR);
                iCubEye *eye = (isLeft?eyeL:eyeR);
                

                //function that calculates the 3DPoint where to redirect saccade and add the offset
                Vector torso(3);
                encTorso->getEncoder(0,&torso[0]);
                encTorso->getEncoder(1,&torso[1]);
                encTorso->getEncoder(2,&torso[2]);
                Vector head(5);
                encHead->getEncoder(0,&head[0]);
                encHead->getEncoder(1,&head[1]);
                encHead->getEncoder(2,&head[2]);
                encHead->getEncoder(3,&head[3]);
                encHead->getEncoder(4,&head[4]);

                Vector x(3);
                x[0] = zDistance * u;
                x[1] = zDistance * v;
                x[2] = zDistance;

                //printf("got the vector x %d %d %d \n",x(0), x(1), x(2));
                
                // find the 3D position from the 2D projection,
                // knowing the distance z from the camera
                //printf( "%s \n", invPrj->toString().c_str());
                Vector xe = yarp::math::operator *(*invPrj, x);
                xe[3] = 1.0;  // impose homogeneous coordinates 
                //printf("imposing homogeneous coordinates \n");
                
                Vector xo;
                if(isOnWings) {
 
                    Vector qw(8);
                    double ratio = M_PI /180; 
                    qw[0]=torso[0] * ratio;
                    qw[1]=torso[1] * ratio;
                    qw[2]=torso[2] * ratio;
                    qw[3]=head[0]  * ratio;
                    qw[4]=head[1]  * ratio;
                    qw[5]=head[2]  * ratio;
                    qw[6]=0.0 * CTRL_DEG2RAD;
                    qw[7]=0.0 * CTRL_DEG2RAD;
                
                    double ver = head[5];
                    xo = yarp::math::operator *(eye->getH(qw),xe);
                    //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
                }
                else {    
 
                    Vector q(8);
                    double ratio = M_PI /180;
                    q[0]=torso[0] * ratio;
                    q[1]=torso[1] * ratio;
                    q[2]=torso[2] * ratio;
                    q[3]=head[0]  * ratio;
                    q[4]=head[1]  * ratio;
                    q[5]=head[2]  * ratio;
                    q[6]=head[3]  * ratio;
                    q[7]=head[4]  * ratio;
                    double ver = head[5];

                    xo = yarp::math::operator *(eye->getH(q),xe);
                    //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
                }
                
                // update position wrt the root frame
                
                //Vector xo = yarp::math::operator *(eye->getH(q),xe);
                printf("fixation point estimated %f %f %f \n",xo[0], xo[1], xo[2]);
                

                if(onDvs){
                    accomplished_flag = false;  
                }
                else if ( (xo[1] > ymax) || (xo[1] < ymin) || (xo[0] < xmin) || (xo[2] < zmin) || (xo[2] > zmax)) {
                    printf("                    OutOfRange ._._._._._.[%f,%f,%f] [%f,%f,%f] [%f,%f,%f] \n",xmin, xo[0], xmax, ymin, xo[1],ymax, zmin, xo[2], zmax);
                    accomplished_flag = true;  //mono = false;     // setting the mono false to inhibith the control of the visual feedback
                    Vector px(3);
                    px[0] = -0.5 + xOffset;
                    px[1] =  0.0 + yOffset;
                    px[2] =  0.3 + zOffset;
                    //igaze->lookAtFixationPoint(px);
                    px[0] = 0; 
                    px[1] = (blockNeckPitchValue == -1)?0:blockNeckPitchValue;
                    px[2] = 0;    
                    igaze->lookAtAbsAngles(px);
                    
                    Time::delay(0.05);
                    printf("waiting for motion done \n");
                    u = width  / 2;  //TO DO: check the division! it would be rather better to use shift
                    v = height / 2;  //TO DO: check the division! it would be rather better to use shift
                    //waitMotionDone();
                    printf("resetting the position: success! \n");

                    // sending error message for saccade SAC_FAIL
                    Bottle& status = statusPort.prepare();
                    status.clear();
                    status.addString("SAC_FAIL");
                    statusPort.write();

                    return;
                }
                else {
                    //sending the timing value -1 when new saccade comes and
                    //vergence action not fineshed
                    if(!accomplished_flag){
                        timing = timingPort.prepare();
                        timing.clear();
                        timing.addDouble(-1);
                        timingPort.write();
                    }
                    else {
                        accomplished_flag = false;
                    }
                }
                
                printf("offset: %f, %f,%f \n", xOffset, yOffset, zOffset );
                printf("visualCorrection %d \n", visualCorrection);
                if (!isOnWings) {
                    printf("starting mono saccade with NO offset \n");
                    //if((tracker->getInputCount()) || (!visualCorrection)) {
                    if((ptracker->getInputCount()) || (!visualCorrection)) {
                        printf("tracker input image ready \n");
                        double dx = 100.0 , dy = 100;
                        double dist = sqrt(dx * dx + dy * dy);
                        if (onDvs) {
                            u = (((((u - 64)/ 128.0) / 7.4) * 4) * 320) + 160;
                            v = (((((v - 64)/ 128.0) / 7.4) * 4) * 240) + 120;
                            printf("onDvs active %d %d \n", u,v);
                        }
                        
                        timeout = 0;
                        timeoutStart = Time::now();
                        while ((dist > 8) && (timeout < TIMEOUT_CONST)) {
                            //printf("inside the while \n");
                            timeoutStop = Time::now();
                            timeout = timeoutStop - timeoutStart;
                            // initialisation of the visual correction 
                            if(visualCorrection){
                                printf("starting visual correction %d %d \n", u, v);
                                //tracker->init(u,v);
                                //tracker->waitInitTracker();
                                ptracker->setUpdate(true);
                                //Time::delay(0.5);
                                //ptracker->setUpdate(false);
                                ptracker->init(u,v);
                                printf("just run into the init \n");
                                ptracker->waitInitTracker();                                
                                //Time::delay(0.1); // TODO : check this delay interval trying to make it shorter
                            }
                            
                            printf("visual correction mechanism correctly initialised \n");
                            Vector px(2);
                            px(0) = u;
                            px(1) = v;
                            //int camSel = 0; //Rea: removed the hardcoded eye drive @ 28/1/13
                            igaze->lookAtMonoPixel(camSel,px,zDistance);
                            igaze->checkMotionDone(&done);
                            printf("first check motion done %d \n", done);
                            dist = 10;
                            Time::delay(0.05);
                            
                            /**** tracking reinforcement only needed for fast saccades ***
                            if(visualCorrection){
                                printf("starting visual correction with tracker refresh\n");
                                // second init of the tracker to remove postsaccadic error
                                tracker->init(160,120);
                                tracker->waitInitTracker();

                                
                                // sending refinement message to the attPrioritiser TRACK
                                Bottle& status = statusPort.prepare();
                                status.clear();
                                status.addString("TRACK_REF");
                                statusPort.write();
                                Time::delay(0.5);
                                
                            } 
                            */
                            
                            //igaze->getFixationPoint(xo);
                            //printf("looking at %f %f %f \n", xo[0], xo[1], xo[2]);
                            
                            
                            // check for offlimits reached                             
                            if ( (xo[1] > ymax) || (xo[1] < ymin) || (xo[0] < xmin) || (xo[2] < zmin) || (xo[2] > zmax)) {
                                printf("                    OutOfRange ._._._._._.[%f,%f,%f] [%f,%f,%f] [%f,%f,%f] \n",xmin, xo[0], xmax, ymin, xo[1],ymax, zmin, xo[2], zmax);
                                accomplished_flag = true;  //mono = false;     // setting the mono false to inhibith the control of the visual feedback
                                Vector px(3);
                                px[0] = -0.5 + xOffset;
                                px[1] =  0.0 + yOffset;
                                px[2] =  0.3 + zOffset;
                                //igaze->lookAtFixationPoint(px);
                                px[0] = 0; 
                                px[1] = (blockNeckPitchValue == -1)?0:blockNeckPitchValue;
                                px[2] = 0;    
                                igaze->lookAtAbsAngles(px);
                                
                                Time::delay(0.1);
                                waitMotionDone();
                                
                                printf("waiting for motion done \n");
                                u = width  / 2;
                                v = height / 2;
                                
                                printf("resetting the position: success! \n");
                                return;
                            }
                            
                            
                            
                            /*if(visualCorrection){
                              tracker->getPoint(point);
                              dx = (double) (point.x - px(0));
                              dy = (double) (point.y - px(1));
                              dist = sqrt(dx * dx + dy * dy);
                              u = width  / 2;
                              v = height / 2;
                              }
                            */
                            dist = 0;
                            
                            printf("correcting distance %f \n", dist);
                        } //end while
                        printf("saccadic event : started %d %d %f \n",u,v,zDistance);
                        
                    }//end if
                }
                else {
                    // monocular with stereo offsets 
                    Vector fp;
                    printf("monocular with stereo offsets \n");
                    fp.resize(3,0.0);
                    fp[0] = xo[0];
                    fp[1] = xo[1];
                    fp[2] = xo[2];
                    igaze->lookAtFixationPoint(fp);
                    visualCorrection = false;
                }
            }
            else {   // for non-mono saccades
                Vector px(3);
                printf("saccadic event to the absolute 3d point with offset %f, %f, %f \n",xOffset, yOffset, zOffset );
                px[0] = xObject;
                px[1] = yObject;
                px[2] = zObject;
                igaze->lookAtFixationPoint(px);
                //igaze->waitMotionDone();
                
                Time::delay(0.1);
                if(visualCorrection){
                    printf("starting visual correction within the stereo saccade\n");
                    tracker->init(160,120);
                    tracker->waitInitTracker();
                    

                    const char BeepChar = '\a'; /* the "alarm" special char */
                    cout << "Maybe you can hear a beep here---" << BeepChar << endl;
                }
                else{
                    goto exiting;
                }
                    
                printf("saccadic event : started %f %f %f \n",xObject,yObject,zObject);
                
            }

            //Time::delay(0.05);
            //Vector px(2);
            //px(0) = 160; px(1) = 120;
            //igaze->lookAtMonoPixel(1,px,0.5);
            //igaze->waitMotionDone();
            timeout = timeoutStop - timeoutStart;
            printf ("checkMotionDone %d \n", done);
            
            
            //***********************************************************************************
            // constant time in sec after which the action is considered not performed
            timeoutStart = Time::now();
            while ((!done)&&(timeout < TIMEOUT_CONST)) {
                // waiting for completion of the gaze action
                while((!done)&&(timeout < TIMEOUT_CONST)) {
                    timeoutStop = Time::now();
                    timeout = timeoutStop - timeoutStart;
                    //printf("saccade timeout %f %d \n", timeout, done);
                    Time::delay(0.005);
                    igaze->checkMotionDone(&done);                    
                }

                if(timeout >= TIMEOUT_CONST) { 
                    // if the gaze controller runs out of time (timeout)
                    Vector v(3);                    
                    timetotStop = Time::now();
                    timetot = timetotStop - timetotStart;
                    printf("TIMEOUT in reaching with a saccade %f <---- \n", timetot);
                    timing = timingPort.prepare();
                    timing.clear();
                    timing.addDouble(-1);
                    timingPort.write();

                    // sending error message for saccade SAC_FAIL
                    Bottle& status = statusPort.prepare();
                    status.clear();
                    status.addString("SAC_FAIL");
                    statusPort.write();
                    
                    v(0)= -0.5; v(1) = 0; v(2) = 0.5;
                    //igaze->stopControl();
                    igaze->lookAtFixationPoint(v);
                    timeoutStart = Time::now();
                    timeout = TIMEOUT_CONST;
                }
                else {
                    printf("checkMotionDone %d \n", done);
                }
            }
                  
            printf("preparing visual correction \n");
            Time::delay(0.5);  // allowing time for preparation
            
            
            // *********************************************************************************************
            // sending the command to the episodic tracker
            ptracker->setCheck(true);
            // the actual visiting of the whole portion of the image is time-demanding
            // the bigger the area controlled the longer the time interval
            // ALLOW necessary time or waitComputation 
            printf("waiting for completion in the periodicTracking.... \n");
            ptracker->waitCorrComputation();
            printf("computation completed \n");
            

            // *********************************************************************************************
            // correcting the macrosaccade using the visual feedback (only if required)
            // saccade correction based of visual feedback (tracker implementation)
            timeoutStart = Time::now();
            timeout = 0;

            if (visualCorrection) {
                printf("Using visual correction \n");
               
                double errorVC        = 100.0;
                double errorVC_pre    = 100.0;
                int countReach        = 0;       // counter of the success in reaching
                int countDecrement    = 0;
                double travelDistance = 0.0;     // distance travel by the tracked point
                float minCumul        = 0;
                double errorx;                      // = (width  >> 1) - point.x;
                double errory;                      // = (height >> 1) - point.y;  
                
                
                Vector px(2);
                timeout = 0;
                point_prev = point;
                // introducing the fast tracker in the system, initialising the fast with the output of slow  
                ptracker->getPoint(point);
                
                
                
                CvPoint testPoint; testPoint.x = -1; testPoint.y = -1;
                while ((testPoint.x != point.x) && (testPoint.y != point.y)) {
                    tracker->init(point.x, point.y);
                    tracker->waitInitTracker();
                    tracker->getPoint(testPoint);
                    printf("tracking %d %d \n",testPoint.x, testPoint.y );
                }                
                printf("-1.just initilisated  with the point %d %d \n", point.x , point.y);

                

                Time::delay(0.1); //extra time necessary for the system to reach steady state
                //igaze->lookAtMonoPixel(camSel,px,0.5);
                
                //initialisation of the loop
                double error_control;

                //Rea : 28/1/13 removed while cycle because it converges in one loop
                //              if the first loop does not suffice increment the number of loops
                //while((countDecrement < 1000) && (countReach < 3)  && (timeout < 1.0) && (ptracker->getInputCount())  ) 
                for(int i = 0 ; i< 1; i++)
                {
                    timeoutStop = Time::now();
                    timeout = timeoutStop - timeoutStart;
                   
                    
                    if(i == 0) {
#ifndef CONFIGFOVEA
                        errorx = (width  >> 1) - point.x;
                        errory = (height >> 1) - point.y;
                        px(0)  = (width  >> 1) - 1 - errorx;    // subtracting 1 for the center of image
                        px(1)  = (height >> 1) - 1 - errory;    // subtracting 1 for the center of image
#else
                        errorx = 160.0 - point.x;
                        errory = 120.0 - point.y;
                        px(0)  = cxl - errorx ;
                        px(1)  = cyl - errory ;
                        //printf("c = (%d,%d) point = (%f,%f) error= (%f, %f) px = (%f,%f) \n",cxl,cyl,point.x, point.y,errorx, errory, px(0), px(1));
#endif

                    
                        errorVC_pre = errorVC;
                        errorVC = sqrt((double) (errorx * errorx + errory * errory));
                        printf("time passed in correcting center =[%f, %f]  %f (%f, %f : %3f) \n",cxl,cyl, timeout, px(0), px(1), errorVC);
                        if(errorVC <= 2) {
                            countReach++;
                        }
                        if ((errorVC > 4 ) ) {
                            //countDecrement++;
                            if(countReach > 0)
                                countReach--;
                        }
                        //else {
                        //    countDecrement = 0;
                        //}
                        printf("norm error %f \n", errorVC);
                    }
                    
                                       
                    //int camSel = 0; 
                    //if(camSel == 0) {
                    //    px(0) = cxl;
                    //    px(1) = cyl;
                    //}
                    igaze->lookAtMonoPixel(camSel,px,0.5);
                    
                    //Vector xo;
                    //igaze->getFixationPoint(xo);
                    //printf("looking at %f %f %f \n", xo[0], xo[1], xo[2]);

                    // check for offlimits by tracking out the stimulus
                            
                    //if ( (xo[1] > ymax) || (xo[1] < ymin) || (xo[0] < xmin) || (xo[2] < zmin) || (xo[2] > zmax)) {
                    //    printf("                    OutOfRange ._._._._._.[%f,%f,%f] [%f,%f,%f] [%f,%f,%f] \n",xmin, xo[0], xmax, ymin, xo[1],ymax, zmin, xo[2], zmax);
                    //    accomplished_flag = true;  //mono = false;     // setting the mono false to inhibith the control of the visual feedback
                    //    Vector px(3);
                    //    px[0] = -0.5 + xOffset;
                    //    px[1] =  0.0 + yOffset;
                    //    px[2] =  0.3 + zOffset;
                        //igaze->lookAtFixationPoint(px);
                    //   px[0] = 0; 
                    //    px[1] = (blockNeckPitchValue == -1)?0:blockNeckPitchValue;
                    //    px[2] = 0;    
                    //    igaze->lookAtAbsAngles(px);
                        
                    //    Time::delay(0.5);
                    //    printf("waiting for motion done \n");
                    //    u = width  / 2;
                    //    v = height / 2;
                        //waitMotionDone();
                    //    printf("resetting the position: success! \n");
                    //   return;
                    //}
                      
                    Time::delay(0.5); //allow time to reach steady state

                    
                    //igaze->waitMotionDone();
                    point_prev = point;        // changing the position of the tracked point in the previous step
                    tracker->getPoint(point);  // have the get point as far as possible from look@mono
                    //ptracker->getPoint(point);
                    travelDistance = sqrt( (double)
                                         (point.x - point_prev.x) * (point.x - point_prev.x) + 
                                         (point.y - point_prev.y) * (point.y - point_prev.y)
                                          );
                                          
                    //minCumul = tracker->getLastMinCumul();
                    minCumul = ptracker->getLastMinCumul();
                    //printf("countReach:%d> the point ended up in (%d,%d) with minCumul %f travelDistance %f \n",countReach,point.x, point.y, minCumul, travelDistance);
                  

                    // *********************************************************************************************
                    // sending the command to the episodic tracker
                    ptracker->setCheck(true);
                    // the actual visiting of the whole portion of the image is time-demanding
                    // the bigger the area controlled the longer the time interval
                    // ALLOW necessary time or waitComputation 
                    printf("waiting for completion in the periodicTracking.... \n");
                    ptracker->waitCorrComputation();
                    printf("computation completed \n");
                    // ******************************************************************************
                    ptracker->getPoint(point);
                    printf("%d.just waiting for initialisation point %d %d... \n",i, point.x , point.y);
                    double errorx_control = 160 - point.x;
                    double errory_control = 120 - point.y;
                    
                    error_control  = sqrt( (double)
                                                  errorx_control * errorx_control + 
                                                  errory_control * errory_control
                                           );

                   
                    px(0)  = cxl - (errorx_control / 10.0) ;
                    px(1)  = cyl - (errory_control / 10.0) ;
                    
                    
                    //if(errorx_control > 0)
                    //    px(0)  = cxl - 1;
                    //else
                    //    px(0)  = cxl + 1;
                    
                    //if (errorx_control > 0)
                    //    px(1)  = cyl - 1;
                    //else
                    //    px(1)  = cyl + 1;
                   
                    //px(0) = cxl;
                    //px(1) = cyl;
                    
                    printf("error = %f preparing saccade:%f %f \n", error_control,px(0),px(1));

                    // ending the cycle
                    // Time::delay(2);
                    // igaze->lookAtMonoPixel(camSel,px,0.5);
                    
                }
             
                timeoutStop = Time::now();
                timeout = timeoutStop - timeoutStart;
                printf("Timeout in saccade execution  %f \n", timeout);
                 
                Time::delay(0.01);
                if (minCumul > 5000000) {
                    printf("Error: out of correlation limits \n");

                    // sending error message for saccade SAC_FAIL
                    Bottle& status = statusPort.prepare();
                    status.clear();
                    status.addString("SAC_FAIL");
                    statusPort.write();
                    
                    px[0] = 0; 
                    px[1] = (blockNeckPitchValue == -1)?0:blockNeckPitchValue;
                    px[2] = 0;    
                    igaze->lookAtAbsAngles(px);
                }
                else if((timeout >= 8.0) || (countDecrement >= 10) || error_control >= 10.0 ) {
                    Time::delay(0.5);

                    Vector px(3);
                    printf("Error in reaching with visualFeedback \n");
                    printf("Error in reaching with visualFeedback \n");
                    printf("Error in reaching with visualFeedback \n");

                    // sending error message for saccade SAC_FAIL
                    Bottle& status = statusPort.prepare();
                    status.clear();
                    status.addString("SAC_FAIL");
                    statusPort.write();
                    
                    timetotStop = Time::now();
                    timetot = timetotStop - timetotStart;
                    timing = timingPort.prepare();
                    timing.clear();
                    timing.addDouble(-1);
                    timingPort.write();

                    px[0] = -0.5 + xOffset;
                    px[1] = 0.0 + yOffset;
                    px[2] = 0.0 + zOffset;
                    //igaze->lookAtFixationPoint(px);

                    px[0] = 0; 
                    px[1] = (blockNeckPitchValue == -1)?0:blockNeckPitchValue;
                    px[2] = 0;    
                    igaze->lookAtAbsAngles(px);
                        
                    /*igaze->checkMotionDone(&done);
                    while((!done)&&(timeout < TIMEOUT_CONST)) {                        
                        timeoutStop = Time::now();
                        timeout =timeoutStop - timeoutStart;
                        Time::delay(0.001);
                        igaze->checkMotionDone(&done);                        
                        }*/
                }
                else {
 
                    printf("saccade accomplished \n");
                    printf("\n \n "); 
           
                    // saccade accomplished
                    //----------------------------------
                    //sending the acknowledgement vergence_accomplished
                    status = statusPort.prepare();
                    status.clear();
                    status.addString("SAC_ACC");
                    statusPort.write();
                }
            }
            
            //-----------------
            //accomplished_flag = true;
            Time::delay(0.05);
        }
    }
    else if(allowedTransitions(3)>0) {
        // ----------------  SMOOTH PURSUIT -----------------------  
        state(4) = 0; state(3) = 1 ; state(2) = 0 ; state(1) = 0 ; state(0) = 0;
        printf(" in RUN of gazeArbiter threadSmooth Pursuit %f %f \n", uVel, vVel);
        printf("velocity profile %f %f \n", uVel, vVel);
        // sending the egoMotion velocity as Feedback
        if(egoMotionFeedback.getOutputCount()) {
            Bottle query;
            query.addVocab(VOCAB3('E','G','O'));
            query.addDouble(uVel);
            query.addDouble(vVel);
            Bottle response;
            egoMotionFeedback.write(query,response);
        }

        //initilisation
        int c = 0;
        int u = 64, v = 64;
        Vector target(2);
        int camSel=0;   // select the image plane: 0 (left), 1 (right)
        Vector px(2);   // specify the pixel where to look
        px[0] = (double) cxl + uVel;
        px[1] = (double) cyl + vVel;        
        double z=1.0;   // distance [m] of the object from the image plane (extended to infinity): yes, you probably need to guess, but it works pretty robustly
        printf("velocity on the retina (%d %d)->(%f %f) \n",cxl,cyl,px[0], px[1]);

        
        //cycle
        double timeend, timeout = 0;
        double timestart = Time::now();
        bool exitloop = false;
        while ((!exitloop)&&(timeout < time)) {
            //px[0] = 1.0;
            //px[1] = 1.0;
            igaze->lookAtMonoPixelWithVergence(camSel,px,4);    // look!
            Time::delay(0.01);
            timeend = Time::now();
            timeout = timeend - timestart;
            c++;
            
            /*
            Vector x(3);
            igaze->getFixationPoint(x); 
            if(x[0] < xmin) {
                exitloop = true;
                printf("exiting because x< xmin \n");
            }
            else if(x[0] > xmax) {
                exitloop = true;
                printf("exiting because x> xmax \n");
            }
            else if(x[1] < ymin) {
                exitloop = true;
                printf("exiting because y< ymin \n");
            }
            else if(x[1] > ymax) {
                exitloop = true;
                printf("exiting because y> ymax \n");
            }
            else if(x[2] < zmin) {
                exitloop = true;
                printf("exiting because z< zmin \n");
            }
            else if(x[2] > zmax) {
                exitloop = true;
                printf("exiting because z< zmax \n");
            }
            */
        }
        
        if(timeout >= time) {
            printf("timeout occurred, ");
        }
        
        printf("position reached in %f \n \n \n", timeout);
        printf("SM_PUR ACCOMPLISHED \n");
        printf("SM_PUR ACCOMPLISHED \n");
        printf("SM_PUR ACCOMPLISHED \n");
        printf("\n");

        //sending the acknowledgement vergence_accomplished
        Bottle& status2 = statusPort.prepare();
        status2.clear();
        status2.addString("SM_ACC");
        statusPort.write();
        //delete &status2;                           

        // sending the egoMotion velocity null as feedback of SM_ACC
        if(egoMotionFeedback.getOutputCount()) {
            Bottle query;
            query.addVocab(VOCAB3('E','G','O'));
            query.addDouble(0);
            query.addDouble(0);
            Bottle response;
            egoMotionFeedback.write(query,response);
        }

    }
    else if(allowedTransitions(2)>0) {
        state(4) = 0; state(3) = 0 ; state(2) = 1 ; state(1) = 0 ; state(0) = 0;
        // ----------------  VERGENCE -----------------------     
        //printf("vergence_accomplished : %d \n",accomplished_flag);
        printf("----------  VERGENCE ----------------------- \n");
        if(!executing) {
            Vector gazeVect(3);
            Vector objectVect(3);
            
            Vector o(4);
            Vector x(3);
            Vector l(3);
            double theta = 0;
                                   
            if((mono)) { 
                printf("phi: %f phi2: %f phi3 : %f  \n", phi, phi2, phi3);
                if((abs(phi) < 0.15) &&(!accomplished_flag) && (!firstVergence))  {                    
                    //if(abs(phi2) < 0.15) {
                        countVerNull += 3;
                        printf("CountVerNull %d \n", countVerNull);
                        //}
                        //else {
                        phi = phi2;
                        //}
                }
                if((countVerNull >= 3) && (!accomplished_flag)) {
                    printf("\n");
                    printf("VERGENCE ACCOMPLISHED \n");
                    printf("VERGENCE ACCOMPLISHED \n");
                    printf("VERGENCE ACCOMPLISHED \n");
                    printf("\n");
                    //sending the acknowledgement vergence_accomplished
                    Bottle& status2 = statusPort.prepare();
                    status2.clear();
                    status2.addString("VER_ACC");
                    statusPort.write();
                    //delete &status2;                    
                    interfaceIOR(timing);
                    firstVergence = true;
                    Time::delay(5.0);

                    goto exiting;
                }
                

                /*
                 *section for revising the vergence
                 */
                /*
                if(accomplished_flag){
                    if((phi > 0.2) || (phi < -0.2)) {
                        printf("the vergence is asking to revise its previouos measure ........ \n");
                        //accomplished_flag = false;
                    }
                    else {
                        return;
                    }
                }
                */
                
                //vergenceInDepth();                
                vergenceInAngle();
               
                /*
                  printf("x1 %f y1 %f z1 %f", x1, y1, z1);
                  double s = sin(theta);
                  double c = cos(theta);
                  double t = 1 - c;
                  //  if axis is not already normalised then uncomment this
                  double magnitude = Math.sqrt(x*x + y*y + z*z);
                  // if (magnitude==0) throw error;
                  // x /= magnitude;
                  // y /= magnitude;
                  // z /= magnitude;
                  
                  double heading; //Roll - φ: rotation about the X-axis
                  double attitude; // Pitch - θ: rotation about the Y-axis
                  double bank;  // Yaw - ψ: rotation about the Z-axis 
                  if ((x1 * y1 * t + z1 * s) > 0.998) { // north pole singularity detected
                  heading = 2 * atan2(x1 * sin(theta / 2), cos(theta / 2));
                  attitude = PI / 2;
                  bank = 0;
                  printf("north pole singularity detected \n");
                  return;
                  }
                  if ((x1*y1*t + z1*s) < -0.998) { // south pole singularity detected
                  heading = -2*atan2(x1*sin(theta/2),cos(theta/2));
                  attitude = -PI/2;
                  bank = 0;
                  printf("south pole singularity detected \n");
                  return;
                  }
                  heading = atan2(y1 * s- x1 * z1 * t , 1 - (y1*y1+ z1*z1 ) * t);
                  attitude = asin(x1 * y1 * t + z1 * s) ;
                  bank = atan2(x1 * s - y1 * z1 * t , 1 - (x1*x1 + z1*z1) * t);
                  printf("headPose heading  %f attitude %f bank %f \n",heading,attitude,bank);
                */

                // in the status port, notification of vergence refinement in action
                printf("Vergence Refinement \n \n");
                Bottle& status2 = statusPort.prepare();
                status2.clear();
                status2.addString("VER_REF");
                statusPort.write();
                
            
            }
            else {   //else of the MONO branch
                //printf("------------- ANGULAR VERGENCE  ----------------- \n \n");
                
                
                //  double elev = (anglesVect[1] * PI) / 180;
                // (anglesVect[0]-(180 - (anglesVect[2] + phi)/2 - anglesVect[2]/2 - beta))/20
                //  gazeVect[0] =(- anglesVect[2] / 80) * cos(elev) ; //version (- anglesVect[2] / 80) * cos(elev) *  -o[1];
                //  gazeVect[1] =(anglesVect[2] / 80) * sin(elev) ;   //tilt
                //  gazeVect[2] = phi ;                              //vergence  
                //  igaze->lookAtRelAngles(gazeVect);
                
            
                gazeVect[0] = 0 ;                //version (- anglesVect[2] / 80) * cos(elev) *  -o[1];
                gazeVect[1] = 0 ;                //tilt
                gazeVect[2] = phi;               //vergence  
                //igaze->lookAtRelAngles(gazeVect);
                executing = true;
                //status = statusPort.prepare();
                //status.clear();
                //status.addString("vergence_accomplished");
                //statusPort.write();
            }
        }
    }
    else if(allowedTransitions(1)>0) {
        // ----------------  WAIT -----------------------  
        state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 1 ; state(0) = 0;
        if((160 != u) && (120 != v)) {
            // performing saccade
            Vector px(2);
            printf("saccadic event before the waiting %d %d \n", u,v );
            px[0] = u;
            px[1] = v;
            
            igaze->lookAtMonoPixel(camSel,px,0.5);
            igaze->waitMotionDone();
        }
        //cycle for the wait of the action
        double timeend, timeout = 0;
        double timestart = Time::now();
        bool exitloop = false;
        while (timeout < time) {
            timeout = Time::now() - timestart;
            Time::delay(0.05);
        }
        
        if(timeout >= time) {
            printf("timeout occurred \n");
        }
        
        printf("position reached in %f \n \n \n", timeout);
        printf("WAIT ACCOMPLISHED \n");
        
        printf("\n");
        //sending the acknowledgement vergence_accomplished
        Bottle& status2 = statusPort.prepare();
        status2.clear();
        status2.addString("WAIT_ACC");
        statusPort.write();
        //delete &status2;                           
    }
    else if(allowedTransitions(0)>0) {
        //-------------------- NULL -------------------------------------------
        state(4) = 0; state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;
    }
    else {
        //printf("No transition \n");
    }
    
    //printf("--------------------------------------------------------->%d \n",done);
 exiting:
    if(allowedTransitions(4)>0) { //SACCADE
        mutex.wait();
        allowedTransitions(4) = 0;
        executing = false;
        mutex.post();
    }
    if(allowedTransitions(3)>0) {   //SMP
        mutex.wait();
        allowedTransitions(3) = 0;
        executing = false;  //executing=false allows new action commands
        mutex.post();              
    }
    if(allowedTransitions(2)>0) { //vergence
        mutex.wait();
        allowedTransitions(2) = 0;
        executing = false;
        mutex.post();
    }
    if(allowedTransitions(1)>0) {   // wait
        mutex.wait();
        allowedTransitions(1) = 0;
        executing = false;
        mutex.post();
        // printf("vergence command : done \n");
    }    
}


void gazeArbiterThread::vergenceInAngle() {
    double phiRel; // vergence relative angle to be commanded to the controller
    printf("Vergence in Angle    ");
    timeoutStart = Time::now();
    timeout = 0;
    Vector anglesVect(3);
    //calculating the magnitude of the 3d vector
    igaze->getAngles(anglesVect);

    if (firstVergence) {
        printf("firstVergence \n             ");
        // first vergence command after the vergence accomplished
        // the first train of vergence command is critical.
        // it has to move the system from an eventual local minima
        // the choosen relative angle is derived from the maximum of the collection shifts
        if((abs(phi) > abs(phi2)) && (abs(phi)>abs(phi3)))
            phiRel = phi;
        else if((abs(phi2) > abs(phi)) && (abs(phi2)>abs(phi3)))
            phiRel = phi2;
        else if((abs(phi3) > abs(phi)) && (abs(phi3)>abs(phi2)))
            phiRel = phi3;
        firstVergence =  false;
    }
    else {
        // the standard descent the angle is the best choice of vergence model.
        // this means that always phi is selected
        phiRel = phi;
    }


    //phiTOT = ((anglesVect[2] + phi)  * PI) / 180;
    phiTOT = anglesVect[2] + phiRel  ; //phiTOT must be in grads
    printf("phiTOT %f \n", phiTOT);    
    tracker->getPoint(point);
    double errorx; // = (width  >> 1) - point.x;
    double errory; // = (height >> 1) - point.y;                    
    double error;
    Vector px(2);

    if (visualCorrection) {
#ifdef MEANVERGENCE        
        if (countRegVerg == 1){
            
            meanRegVerg = sumRegVerg / 1.0;
            
            errorx = 160 - point.x;
            errory = 120 - point.y;
            px(0) = 182;
            px(1) = 113;
            
            printf("norm error in mean %f \n", error);
            //int camSel = 0; //Rea: removed the hardcoded eye drive @ 28/1/13
            igaze->lookAtMonoPixelWithVergence(camSel,px,meanRegVerg);
            //tracker->getPoint(point);
            Time::delay(1.5);

            countRegVerg = 0;
            sumRegVerg = 0;
        }
        else {
            sumRegVerg += phiTOT;
            countRegVerg++;
        }
#else  
        timeoutStart = Time::now();
        error = 2000;
        timeout = 0;
        // the vergence cannot be initialised if the feedback point hasn`t been defined
        if((0 == point.x) && (0 == point.y)) {
            timeout = TIMEOUT_CONST;
        }
        while((error > 5.0)&&(timeout < TIMEOUT_CONST)) {
            timeoutStop = Time::now();
            timeout = timeoutStop - timeoutStart;
            
            Vector pa(3);
            pa[0] = 0; pa[1] = 0; pa[2] = phiRel;
            igaze->lookAtRelAngles(pa);
            
            igaze->getAngles(anglesVect);
            printf("                     phiReached %f \n", anglesVect[2]);
            
            double Ke = 2.0;
            errorx = 160 - point.x;
            errory = 120 - point.y;
            px(0)  = 182 - Ke * errorx;
            px(1)  = 113 - Ke * errory;
            
            error = sqrt(errorx * errorx + errory * errory);
            printf("norm error %f vergence %f \n", error, phiRel);
            if(error >30.0) {
                timeout = TIMEOUT_CONST;
            }

            //Time::delay(0.1);
            //int camSel = 0; //Rea: removed the hardcoded eye drive @ 28/1/13
            //igaze->lookAtMonoPixelWithVergence(camSel,px,phiTOT);
            //double varDistance = BASELINE / (2 * sin (phiTOT / 2)); 
            //printf("varDistance %f \n", varDistance);
            //igaze->getAngles(anglesVect);
            igaze->lookAtMonoPixelWithVergence(camSel, px, phiTOT);
            //igaze->waitMotionDone();
            //Time::delay(0.1);
            tracker->getPoint(point);            

            error = 5.0;
            
        } // while
#endif
        
        
    } // end if (visualCorrection)
    else {  // else if (visualCorrection)
        Vector pa(3);
        printf("Vergence : look at relative angle with no feedback; phiRel:%f \n", phiRel);
        pa[0] = 0; pa[1] = 0; pa[2] = phiRel;
        igaze->lookAtRelAngles(pa);
    }
}

void gazeArbiterThread::vergenceInDepth(){
    //printf("------------- VERGENCE   ----------------- \n");            
    Vector anglesVect(3);
    //calculating the magnitude of the 3d vector
    igaze->getAngles(anglesVect);
    phiTOT = ((anglesVect[2] + phi)  * PI) / 180;
    printf("phiTOT %f \n", phiTOT);
    //phiTOT = phiTOT + phi;
    //double magnitude = sqrt ( x1 * x1 + y1 * y1 + z1 * z1);
    varDistance = BASELINE / (2 * sin (phiTOT / 2));     //in m after it is fixation state
    
    //double varDistance = (BASELINE /  sin (phiTOT / 2)) * sin (leftHat);     //in m after it is fixation state
    //double varDistance = h * 1.5 * sqrt(1 + tan(elevation) * tan(elevation)) ;
    //double varDistance = sqrt (h * h + (BASELINE + b) * (BASELINE + b)) ;
    //double varDistance = ipLeft;
    //printf("varDistance %f distance %f of vergence angle tot %f enc %f \n",varDistance,distance, (phiTOT * 180)/PI, _head(5));
    
    
    
    //tracker->getPoint(point);
    double error = 1000;                                         
    timeoutStart=Time::now();
    //while ((error > 2.0)&&(timeout < 10)) {
    
    timeoutStop = Time::now();
    timeout =timeoutStop - timeoutStart;
    //corrected the error
    tracker->getPoint(point);
    Vector px(2);
    //int camSel = 0; //Rea: removed the hardcoded eye drive @ 28/1/13
    
    if(!visualCorrection){
        px(0) = 183;
        px(1) = 113; 
        printf("no visual correction initialised %f %f \n", px(0), px(1));
        igaze->lookAtMonoPixel(camSel,px,varDistance);                       
    }
    else {
        double errorx = 160 - point.x;
        double errory = 120 - point.y;
        //tracker->init(u,v);
        //tracker->waitInitTracker();
        //printf ("error %f,%f \n",errorx, errory);
        
        error = sqrt(errorx * errorx + errory * errory);
        //printf("norm error %f \n", error);
        
        //px(0) = (width>>1)  - 1 - errorx;   //159 - errorx
        //px(1) = (height>>1) - 1 - errory;   //119 - errory
        px(0) = 182 - errorx;   //159 - errorx
        px(1) = 113 - errory;   //119 - errory
        
        igaze->lookAtMonoPixel(camSel,px,varDistance);
    }
    
    printf("-------------- %d ------------------varDistance %f,%f->%f->%f \n",countVerNull, phi, anglesVect[2], phiTOT,varDistance);
    
    Time::delay(0.05);                                        
    
}

void gazeArbiterThread::threadRelease() {
    inLeftPort.close();
    inRightPort.close();
    statusPort.close();
    templatePort.close();
    blobDatabasePort.close();
    inhibitionPort.close();
    timingPort.close();
    if(tracker!=0) {
        tracker->stop();
    }
    //if(ptracker!=0) {
    //    ptracker->stop();
    //}
    printf("trying to release the velControl \n");
    if(velControl!=0) {
        velControl->stop();
    }
    delete eyeL;
    delete eyeR;
    igaze->restoreContext(originalContext);
    delete clientGazeCtrl;
}

void gazeArbiterThread::update(observable* o, Bottle * arg) {
    //printf("ACK. Aware of observable asking for attention \n");
    if (arg != 0) {
        //printf("bottle: %s ", arg->toString().c_str());
        int size = arg->size();
        ConstString name = arg->get(0).asString();
        if(!strcmp(name.c_str(),"STOP")) {
            printf("STOP request!!!!!! \n");
            timeout = 100;
            suspend();
            resume();
        }
        else if(!strcmp(name.c_str(),"WAIT")) {
            // monocular saccades with visualFeedback
            printf("WAIT request \n");
            u = arg->get(1).asInt();
            v = arg->get(2).asInt();
            zDistance = 0.5;
            time = arg->get(3).asDouble();
            mutex.wait();
            //setVisualFeedback(true); <<-------- TODO: remove because does not make any sense
            stateRequest[1] = 1;
            mutex.post();
            timetotStart = Time::now();
            mono = true;
            firstVer = true;
            firstVergence = true;
        }
        else if(!strcmp(name.c_str(),"SAC_MONO")) {
            // monocular saccades with visualFeedback
            printf("MONO SACCADE request \n");
            u = arg->get(1).asInt();
            v = arg->get(2).asInt();
            zDistance = arg->get(3).asDouble();
            mutex.wait();
            //setVisualFeedback(true); <<-------- TODO: remove because does not make any sense
            stateRequest[4] = 1;
            mutex.post();
            timetotStart = Time::now();
            mono = true;
            firstVer = true;
            firstVergence = true;
        }
        else if(!strcmp(name.c_str(),"SAC_EXPR")) {
            // monocular saccades without visualfeedback
            printf("EXPRESS SACCADE request \n");
            u = arg->get(1).asInt();
            v = arg->get(2).asInt();
            zDistance = arg->get(3).asDouble();
            mutex.wait();
            //setVisualFeedback(false);
            stateRequest[4] = 1;
            //executing = false;
            mutex.post();
            timetotStart = Time::now();
            mono = true;
            firstVer = true;
            firstVergence = true;
        }
        else if(!strcmp(name.c_str(),"SAC_ABS")) {
            xObject = arg->get(1).asDouble();
            yObject = arg->get(2).asDouble();
            zObject = arg->get(3).asDouble();
            printf("received request of abs saccade in position %f %f %f \n", xObject, yObject, zObject);
            mutex.wait();
            stateRequest[4] = 1;
            //executing = false;
            mutex.post();
            mono = false;
            firstVergence = true;
        }
        else if(!strcmp(name.c_str(),"SM_PUR")) {
            printf("received a command of smooth pursuit ");
            uVel = arg->get(1).asDouble();
            vVel = arg->get(2).asDouble();
            time = arg->get(3).asDouble();
            printf("velocity %f %f %f \n", uVel, vVel,time);
            mutex.wait();
            stateRequest[3] = 1;
            //velControl->setVelocityComponents(uVel, vVel);
            //executing = false;
            mutex.post();
            firstVergence = true;
        }
        else if(!strcmp(name.c_str(),"VER_REL")) {
            phi  = arg->get(1).asDouble();            
            phi2 = arg->get(2).asDouble();
            phi3 = arg->get(3).asDouble();
            mutex.wait();
            mono = true;
            stateRequest[2] = 1;
            //executing = false;
            mutex.post();
        }
        else if(!strcmp(name.c_str(),"VER_ABS")) {
            phi = -1;
            phiTOT1 = arg->get(1).asDouble();            
            phiTOT2 = arg->get(2).asDouble();
            phiTOT3 = arg->get(3).asDouble();
            
            mutex.wait();
            mono = true;
            stateRequest[2] = 1;
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

void gazeArbiterThread::calculateDistance() {
    //printf("leftAngle:%f  ,  rightAngle:%f \n", (leftAngle*180)/PI, (rightAngle*180)/PI);
    double h,vergence,v,ipLeft, rightHat, rightAngle, leftHat, alfa,b, leftAngle;
    if(leftAngle >= 0) {
        if(rightAngle >= 0) {
            rightHat = PI / 2 - rightAngle;
            leftHat = PI / 2 - leftAngle;
            alfa = PI / 2 - rightHat;
            
            h = BASELINE * (
                            (sin(leftHat) * sin(rightHat)) / 
                            (sin(rightHat) * sin(vergence + alfa) - sin(alfa) * sin(leftHat))
                            );
            b = h * (sin(alfa) / sin(rightHat));
            ipLeft = sqrt ( h * h + (BASELINE + b) * (BASELINE + b));
        }
        else {
            if(rightAngle >= leftAngle) {
                rightHat = PI / 2 + rightAngle;
                leftHat = PI / 2 - leftAngle;
                alfa = PI / 2 - leftHat;
                h = BASELINE * (
                                (sin(leftHat) * sin(rightHat)) / 
                                (sin(leftHat) * sin(vergence + alfa) + sin(alfa) * sin(rightHat))
                                );
                b = h * (sin(alfa) / sin(leftHat));
                ipLeft = sqrt ( h * h + b * b);
            }
            else {
                rightHat = PI / 2 + rightAngle;
                leftHat = PI / 2 - leftAngle;
                alfa = PI / 2 - rightHat;
                h = BASELINE * (
                                (sin(leftHat) * sin(rightHat)) / 
                                (sin(rightHat) * sin(vergence + alfa) + sin(alfa) * sin(leftHat))
                                );
                b = h * (sin(alfa) / sin(rightHat));
                ipLeft = sqrt ( h * h + (BASELINE + b) * (BASELINE + b));
            }
        }
    }
    else {
        rightHat = PI /2  + rightAngle;
        leftHat = PI / 2 + leftAngle;
        alfa = PI / 2 - leftHat;
        h = BASELINE * (
                        (sin(leftHat) * sin(rightHat)) / 
                        (sin(leftHat) * sin(vergence + alfa) - sin(alfa) * sin(rightHat))
                        );
        b = h * (sin(alfa) / sin(leftHat));
                ipLeft = sqrt ( h * h + b * b);
    }
}
        


