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
 * Public License fo
 r more details
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
#define BASELINE 0.068     // distance in meters between eyes
 
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


gazeArbiterThread::gazeArbiterThread(string _configFile) : RateThread(THRATE) {
    numberState = 4; //null, vergence, smooth pursuit, saccade
    configFile = _configFile;
    firstVer = false;
    phiTOT = 0;
    xOffset = yOffset = zOffset = 0;

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

gazeArbiterThread::~gazeArbiterThread() {

}

bool gazeArbiterThread::threadInit() {
    done=true;
    executing = false;
    printf("starting the thread.... \n");

    blobDatabasePort.open(getName("/database").c_str());

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
    igaze->blockNeckPitch(-40.0);
    
    string headPort = "/icub/head";//<<--------- hard coded here remove asap
    string robot("icub");
    string name("local");

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
    optPolyTorso.put("local",("/"+name+"/torso/position").c_str());
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

    template_roi.width=template_roi.height=template_size;
    search_roi.width=search_roi.height=search_size;

    //inLeftPort.open(getName("/matchTracker/img:i").c_str());
    //inRightPort.open(getName("/matchTracker/img:o").c_str());
    statusPort.open("/gazeArbiter/status:o");
    firstConsistencyCheck=true;

    return true;
}

void gazeArbiterThread::interrupt() {
    //inCommandPort
    inLeftPort.interrupt();
    inRightPort.interrupt();
    statusPort.interrupt();
    blobDatabasePort.interrupt();
}

void gazeArbiterThread::setDimension(int w, int h) {
    width = w;
    height = h;
}

void gazeArbiterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string gazeArbiterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
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


void gazeArbiterThread::run() {
    Bottle& status = statusPort.prepare();
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
            //calculating where the fixation point would end up
            
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

            
            //if (isLeft)
            //    q[7]=head[4]+head[5]/2.0;
            //else
            //    q[7]=head[4]-head[5]/2.0;

            Vector q(8);
            double ratio = M_PI /180;
            q[0]=torso[0] * ratio;
            q[1]=torso[1] * ratio;
            q[2]=torso[2] * ratio;
            q[3]=head[0] * ratio;
            q[4]=head[1] * ratio;
            q[5]=head[2] * ratio;
            q[6]=head[3] * ratio;
            q[7]=head[4] * ratio;
            double ver = head[5];
            //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);

            Vector x(3);
            x[0] = zDistance * u;
            x[1] = zDistance * v;
            x[2] = zDistance;

            // find the 3D position from the 2D projection,
            // knowing the distance z from the camera
            Vector xe = yarp::math::operator *(*invPrj, x);
            xe[3]=1.0;  // impose homogeneous coordinates                

            // update position wrt the root frame
            Vector xo = yarp::math::operator *(eye->getH(q),xe);
            printf("fixation point estimated %f %f %f",xo[0], xo[1], xo[2]);


            if ((xo[1]> 0.3)||(xo[1]< -0.3)) {
                printf("           OutOfRange ........... \n");
                Vector px(3);
                px[0] = -0.5 + xOffset;
                px[1] = 0.0 + yOffset;
                px[2] = 0.1 + zOffset;
                igaze->lookAtFixationPoint(px);
                return;
            }
            
            // starting saccade toward the direction of the required position
            // needed timeout because controller kept stucking whenever a difficult position could not be reached
            timeoutStart=Time::now();
            if(mono){
                //printf("offset: %d, %d,%d \n", xOffset, yOffset, zOffset );
                if ((xOffset == 0) && (yOffset == 0) && (zOffset == 0)) {
                    printf("starting mono saccade with NO offset \n");
                    if(tracker->getInputCount()) {
                        double dx = 100.0 , dy = 100;
                        double dist = sqrt(dx * dx + dy * dy);
                        while (dist > 10) {
                            tracker->init(u,v);
                            tracker->waitInitTracker();
                            Vector px(2);
                            px(0) = u;
                            px(1) = v;
                            int camSel = 0;
                            igaze->lookAtMonoPixel(camSel,px,zDistance);
                            tracker->getPoint(point);
                            dx = (double) (point.x - px(0));
                            dy = (double) (point.y - px(1));
                            dist = sqrt(dx * dx + dy * dy);
                            u = width / 2;
                            v = height / 2;
                        }
                        printf("saccadic event : started \n",u,v,zDistance);
                    }
                }
                else {
                    // monocular with stereo offsets 
                    Vector fp;
                    fp.resize(3,0.0);
                    fp[0]=xo[0] + xOffset;
                    fp[1]=xo[1] + yOffset;
                    fp[2]=xo[2] + zOffset;
                }
            }
            else {
                Vector px(3);
                px[0] = xObject + xOffset;
                px[1] = yObject + yOffset;
                px[2] = zObject + zOffset;
                igaze->lookAtFixationPoint(px);
                printf("saccadic event : started \n",xObject,yObject,zObject);
            }

            executing = true;
            Time::delay(0.05);
            igaze->checkMotionDone(&done);
            timeout =timeoutStop - timeoutStart;
            //printf ("timeout %d \n", timeout);

            //constant time of 10 sec after which the action is considered not performed
            timeoutStart=Time::now();
            while ((!done)&&(timeout < 10.0)) {
                while((!done)&&(timeout < 10.0)) {
                    timeoutStop = Time::now();
                    timeout =timeoutStop - timeoutStart;
                    Time::delay(0.005);
                    igaze->checkMotionDone(&done);
                    
                }
                if(timeout >= 10.0) {
                    Vector v(3);
                    v(0)= -0.5; v(1) = 0; v(2) = 0.5;
                    igaze->stopControl();
                    igaze->lookAtFixationPoint(v);
                    timeoutStart = Time::now();
                    timeout = 0;
                }
                else {
                    printf("saccade_accomplished \n");
                }
            }
            //correcting the macrosaccade using the visual feedback
            double error = 1000.0;
            while(( error > 1)&&(timeout < 10.0)&&(tracker->getInputCount())) {
                timeoutStop = Time::now();
                timeout =timeoutStop - timeoutStart;
                //printf("timeout in correcting  %d \n", timeout);
                //corrected the error
                double errorx = (width / 2.0)  - point.x;
                double errory = (height / 2.0) - point.y;
                Vector px(2);
                px(0) = (width / 2.0) - errorx;
                px(1) = (height / 2.0) - errory;
                error = sqrt(errorx * errorx + errory * errory);
                //printf("norm error %f \n", error);
                int camSel = 0;
                igaze->lookAtMonoPixel(camSel,px,zDistance);
                tracker->getPoint(point);
                //printf("the point ended up in %d  %d \n",point.x, point.y);
            }
            Time::delay(0.05);
            if(timeout >= 10.0) {
                Vector px(3);
                px[0] = -0.5 + xOffset;
                px[1] = 0.0 + yOffset;
                px[2] = 0.1 + zOffset;
                igaze->lookAtFixationPoint(px);
                igaze->checkMotionDone(&done);
                while((!done)&&(timeout < 10.0)) {
                    
                    timeoutStop = Time::now();
                    timeout =timeoutStop - timeoutStart;
                    printf("f");
                    Time::delay(0.005);
                    igaze->checkMotionDone(&done);
                    
                }
            }
        }
    }
    else if(allowedTransitions(2)>0) {
        state(3) = 0 ; state(2) = 1 ; state(1) = 0 ; state(0) = 0;
    }
    else if(allowedTransitions(1)>0) {
        state(3) = 0 ; state(2) = 0 ; state(1) = 1 ; state(0) = 0;
        // ----------------  VERGENCE -----------------------     
        
        //printf("Entering in VERGENCE \n");
        if(!executing) {
            Vector gazeVect(3);
            Vector objectVect(3);
            Vector anglesVect(3);
            Vector o(4);
            Vector x(3);
            Vector l(3);
            double theta = 0;
            
            //printf("leftAngle:%f  ,  rightAngle:%f \n", (leftAngle*180)/PI, (rightAngle*180)/PI);
            /*
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
            */
            

            if((mono)) {
                
                if((phi < 0.1)&&(phi>-0.1)&&(!accomplished_flag)) {
                    status = statusPort.prepare();
                    status.clear();
                    status.addString("vergence_accomplished");
                    statusPort.write();
                    printf(" sending location !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! \n");
                    accomplished_flag = true;
                    


                    //calculating the 3d position and sending it to database
                    u = 160; 
                    v = 120;
                    zDistance = 0.5;
                    Vector fp(3);

                    if (invPrjL) {
                      
                        /*
                        
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
                        printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0]/ratio,q[1]/ratio,q[2]/ratio,q[3]/ratio,q[4]/ratio,q[5]/ratio,q[6]/ratio,q[7]/ratio);

                        
                        
                            
                        Vector x(3);
                        x[0]=z * u;   //epipolar correction excluded the focal lenght
                        x[1]=z * v;
                        x[2]=z;
                        */

                        /*
                        
                        // find the 3D position from the 2D projection,
                        // knowing the distance z from the camera
                        Vector xe = yarp::math::operator *(*invPrjL, x);
                        xe[3]=1.0;  // impose homogeneous coordinates                
                        
                        // update position wrt the root frame
                        Matrix eyeH = eyeL->getH(q);
                        //printf(" %f %f %f ", eyeH(0,0), eyeH(0,1), eyeH(0,2));
                        Vector xo = yarp::math::operator *(eyeH,xe);
                        
                        //fp.resize(3,0.0);
                        //fp[0]=xo[0];
                        //fp[1]=xo[1];
                        //fp[2]=xo[2];
                        printf("object %f,%f,%f \n",xo[0],xo[1],xo[2]);
                        */

                        /*

                        //adding novel position to the GUI
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
                        sublistR.addDouble(0.0);
                        listAttr.append(sublistR);
                        
                        Bottle& sublistG = listAttr.addList();
                        sublistG.addString("g");
                        sublistG.addDouble(0.0);
                        listAttr.append(sublistG);
                        
                        Bottle& sublistB = listAttr.addList();
                        sublistB.addString("b");
                        sublistB.addDouble(0.0);
                        listAttr.append(sublistB);
                        
                        Bottle& sublistLife = listAttr.addList();
                        sublistLife.addString("lifeTimer");
                        sublistLife.addDouble(10.0);
                        listAttr.append(sublistLife);          
                        
                        
                        blobDatabasePort.write(request, reply);

                        */
                        
                        
                    }
                    
                    Time::delay(1);
                    return;
                }
                
                if(accomplished_flag){
                    if((phi>0.4)||(phi<-0.4))
                        accomplished_flag = false;
                    else
                        return;
                }
                

                //printf("------------- VERGENCE   ----------------- \n");
            
            
                //calculating the magnitude of the 3d vector
                igaze->getAngles(anglesVect);
                phiTOT = ((anglesVect[2] + phi)  * PI) / 180;
                //phiTOT = phiTOT + phi;
                //double magnitude = sqrt ( x1 * x1 + y1 * y1 + z1 * z1);
                double varDistance = BASELINE / (2 * sin (phiTOT / 2));     //in m after it is fixation state
                
                //double varDistance = (BASELINE /  sin (phiTOT / 2)) * sin (leftHat);     //in m after it is fixation state
                //double varDistance = h * 1.5 * sqrt(1 + tan(elevation) * tan(elevation)) ;
                //double varDistance = sqrt (h * h + (BASELINE + b) * (BASELINE + b)) ;
                //double varDistance = ipLeft;
                //printf("varDistance %f distance %f of vergence angle tot %f enc %f \n",varDistance,distance, (phiTOT * 180)/PI, _head(5));
            
            
            
                //tracker->getPoint(point);
                double error = 1000;                                         
                timeoutStart=Time::now();
                //while ((error > 2.0)&&(timeout < 10)) {
                
                //timeoutStop = Time::now();
                // timeout =timeoutStop - timeoutStart;
                //corrected the error
                //double errorx = 160 - point.x;
                //double errory = 120 - point.y;
                //printf ("error %f,%f \n",errorx, errory);
                Vector px(2);
                //error = sqrt(errorx * errorx + errory * errory);
                //printf("norm error %f \n", error);
                int camSel = 0;
                
                px(0) = (width / 2.0);
                px(1) = (height / 2.0);
                
                printf("----------------------------------varDistance %f \n", varDistance);
                igaze->lookAtMonoPixel(camSel,px,varDistance);
                
                //tracker->getPoint(point);
                //}
            
               
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
            
            
            }
            else {
                //printf("------------- ANGULAR VERGENCE  ----------------- \n \n");
                
                /*
                  double elev = (anglesVect[1] * PI) / 180;
                  //(anglesVect[0]-(180 - (anglesVect[2] + phi)/2 - anglesVect[2]/2 - beta))/20
                  gazeVect[0] =(- anglesVect[2] / 80) * cos(elev) ; //version (- anglesVect[2] / 80) * cos(elev) *  -o[1];
                  gazeVect[1] =(anglesVect[2] / 80) * sin(elev) ;   //tilt
                  gazeVect[2] = phi ;                              //vergence  
                  igaze->lookAtRelAngles(gazeVect);
                */
            
                gazeVect[0] = 0 ;                //version (- anglesVect[2] / 80) * cos(elev) *  -o[1];
                gazeVect[1] = 0 ;                //tilt
                gazeVect[2] = phi;              //vergence  
                //igaze->lookAtRelAngles(gazeVect);
                executing = true;
                //status = statusPort.prepare();
                //status.clear();
                //status.addString("vergence_accomplished");
                //statusPort.write();
            }
        }
    }
    else if(allowedTransitions(0)>0) {
        state(3) = 0 ; state(2) = 0 ; state(1) = 0 ; state(0) = 1;
    }
    else {
        //printf("No transition \n");
    }

    
    //printf("--------------------------------------------------------->%d \n",done);
            
        if(allowedTransitions(3)>0) {
            //igaze->checkMotionDone(&done);  // the only action that should not be tracking therefore it make wait till the end
            //if (done) {
            //    mutex.wait();
                allowedTransitions(3) = 0;
                executing = false;  //executing=false allows new action commands
                printf ("\n\n\n\n\n\n\n\n\n");
                //    mutex.post();
                // printf("saccadic event : done \n");
                //}
            
        }
        if(allowedTransitions(2)>0) {
            mutex.wait();
            allowedTransitions(2) = 0;
            executing = false;
            mutex.post();
        }
        if(allowedTransitions(1)>0) {
            //mutex.wait();
            allowedTransitions(1) = 0;
            executing = false;
            //printf ("\n\n\n\n\n\n\n\n\n");
            //mutex.post();
            printf("vergence command : done \n");
        }
    
}

void gazeArbiterThread::threadRelease() {
    inLeftPort.close();
    inRightPort.close();
    statusPort.close();
    blobDatabasePort.close();
    delete clientGazeCtrl;
}

void gazeArbiterThread::update(observable* o, Bottle * arg) {
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

 


