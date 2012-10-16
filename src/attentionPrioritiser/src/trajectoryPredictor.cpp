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
 * @file trajectoryPredictor.cpp
 * @brief Implementation of the thread of trajectory predictor(see header trajectoryPredictor.h)
 */


#include <iCub/trajectoryPredictor.h>

#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace attention::predictor;
using namespace attention::evaluator;
using namespace std;

#define THRATE 10

trajectoryPredictor::trajectoryPredictor() {
    tracker = 0;
    blockNeckPitchValue = -1;
    eQueue = new evalQueue();
}

trajectoryPredictor::~trajectoryPredictor() {
    delete eQueue;
}

bool trajectoryPredictor::threadInit() {
    printf("-------------------------------trajectoryPredictor::threadInit:starting the thread.... \n");
    
    // open files
    fout = fopen("./attPrioritiser.trajectoryPredictor.3Dtraj.txt","w+");

    // open ports 
    string rootName("");
    rootName.append(getName("/blobImage:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inImagePort.open(rootName.c_str()); 

    // --------------------------------------------------------------------------------------
    //initializing gazecontrollerclient
    printf("initialising gazeControllerClient \n");
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
    localCon.append("testWings");
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
  
    blockNeckPitchValue = -1;
    if(blockNeckPitchValue != -1) {
        igaze->blockNeckPitch(blockNeckPitchValue);
        printf("pitch fixed at %f \n",blockNeckPitchValue);
    }
    else {
        printf("pitch free to change\n");
    }
    
    
    printf(" \n \n ----------------- trajectoryPredictor::threadInit --------------------- \n");
    evalQueue::iterator it;
    evalThread* tmp;
    it = eQueue->begin();
    printf("got the pointer to the evalThread %08x \n", (*it));
   
    //Vector xCheck = (*it)->getX();
    //printf(" xCheck = \n %s \n", xCheck.toString().c_str());
    printf("---------------------------------------------------------------------------------\n");
    
    /* _old
    minJerkModel* modelC = new minJerkModel();
    modelC->init(1, 1);
    printf("modelC\n %s \n %s \n", modelC->getA().toString().c_str(), modelC->getB().toString().c_str());
    genPredModel* mC = dynamic_cast<genPredModel*>(modelC);
    evalThread etC(*mC);
    evalMJ1_T1 = etC;
    evalMJ1_T1.start();
    eQueue->push_back(&evalMJ1_T1);  
    */

    // ---------------------------------------------------------------------------
    
    linAccModel* modelB = new linAccModel();
    
    int rowA = modelB->getA().rows();
    int colA = modelB->getA().cols();
    Vector z0(rowA);
    Vector x0(rowA);
    x0.zero();z0.zero();
    x0(0) = 1.0; 
    Matrix P0(rowA,colA);
    printf("initialisation of P0 %d %d \n", rowA, colA);
    for (int i = 0; i < rowA; i++) {
        for (int j = 0; j < colA; j++) { 
            P0(i,j) += 0.01;
        }      
    }
    printf("modelB\n %s \n %s \n", modelB->getA().toString().c_str(),modelB->getB().toString().c_str());    
    printf("P0\n %s \n", P0.toString().c_str());    

    //---------------------------------------------------------------------------
    
    printf(" creating eval thread \n");
    modelB->init(1.0);
    genPredModel* mB = dynamic_cast<genPredModel*>(modelB);
    eval = new evalThread(*mB);
    eval->init(z0,x0,P0);
    printf("genPred model A \n %s \n",mB    ->getA().toString().c_str());
    printf("lin acc model A \n %s \n",modelB->getA().toString().c_str());
    printf("just initialised genPredModel %08X \n",&eval);
    eval->start();
    eQueue->push_back(eval); 
    
    

    //------------------------------------------------------------------------------
    
    printf("moving to the next predictor \n");
    modelB = new linAccModel();
    modelB->init(2.0);
    mB = dynamic_cast<genPredModel*>(modelB);
    eval = new evalThread(*mB);
    eval->init(z0,x0,P0);
    eval->start();
    eQueue->push_back(eval); 
    
    
    printf("------------------- trajectoryPredictor::threadInit: success in the initialisation \n");
        
    return true;
}

void trajectoryPredictor::interrupt() {
    inImagePort.interrupt();
}

void trajectoryPredictor::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string trajectoryPredictor::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void trajectoryPredictor::isPredict(bool& value) {
    mutex.wait();
    value = predictionAccompl;
    mutex.post();
}


void trajectoryPredictor::extractCentroid(yarp::sig::ImageOf<yarp::sig::PixelMono>* image, int& x, int& y) {
    x = 10.0;
    y = 10.0;
}

Vector trajectoryPredictor::projectOnPlane(int a, int b, int c , int d, int u, int v) {
     Vector plane(4);        // specify the plane in the root reference frame as ax+by+cz+d=0; z=-0.12 in this case
     plane[0]=a;   // a
     plane[1]=b;   // b
     plane[2]=c;   // c
     plane[3]=d;   // d
     
     //printf("using tableHeight %f \n", tableHeight);
     
     //Vector x;
     if (plane.length() < 4) {
         fprintf(stdout,"Not enough values given for the projection plane!\n");
     }
     
     //printf("defining the point p0 belonging to the plane \n");
     mutexP0.wait();
     p0(3);
     p0.zero();
     
     if (plane[0]!=0.0)
         p0[0]=-plane[3]/plane[0];
     else if (plane[1]!=0.0)
         p0[1]=-plane[3]/plane[1];
     else if (plane[2]!=0.0)
         p0[2]=-plane[3]/plane[2];
     else  {
         fprintf(stdout,"Error while specifying projection plane!\n");
     }
     mutexP0.post();
        
     
     // take a vector orthogonal to the plane
     
     //Vector n(3);
     mutexN.wait();
     n[0]=plane[0];
     n[1]=plane[1];
     n[2]=plane[2];
     mutexN.post();
     
     //printf("p0 = %s ; n = %s \n", p0.toString().c_str(), n.toString().c_str());
     
     Time::delay(0.1);
}



bool trajectoryPredictor::estimateVelocity(int x, int y, double& Vx, double& Vy, double& xPos, double& yPos, double& time, double& distance) {
    printf(" trajectoryPredictor::estimateVelocity in pos.%d,%d  \n", Vx, Vy);
    
    CvPoint p_curr, p_prev;

    double timeStart = Time::now();
    double timeStop, timeDiff;
    double dist_prev;
    double dist;
    double vel_prev;
    double vel ;
    double acc ;
    double velX      = 0;
    double velY      = 0;
    double velX_prev = 0;
    double velY_prev = 0;
    double accX      = 0;
    double accY      = 0;
    double maxAccX   = 0;
    double maxAccY   = 0;
    double maxAcc    = 0;
    double meanVelX;
    double meanVelY;
    double distX;
    double distY;
    double distX_prev;
    double distY_prev;
    
    int nIter = 20;
    
    // //for n times records the position of the object and extract an estimate
    // // extracting the velocity of the stimulus; saving it into a vector 
    Matrix zMeasurements2D(nIter,2);
    Matrix zMeasurements3D(nIter,3);
    Matrix uMeasurements(2.0, nIter);
    for (int i = 0; i < 2; i++) {
        for (int j =0 ; j < nIter; j++) {
            uMeasurements(i, j) = 1.0;
        }
    }
    
    //passing from the 2D image plane to the 3D real location using homography
    //projectOnPlane(0,0,0,1,0,0);
    //get3DPointOnPlane (const int camSel, const yarp::sig::Vector &px, const yarp::sig::Vector &plane, yarp::sig::Vector &x)=0
    int camSel = 1; //left camera
    Vector px(2);
    px(0) = 160; 
    px(1) = 120;
    Vector plane(4);
    plane(0) = 1;
    plane(1) = 0;
    plane(2) = 0;
    plane(3) = 0.35;
    Vector x3D(3);
    igaze->get3DPointOnPlane(camSel,px,plane,x3D);
    printf("3dposition on the plane extract by gazeController %s \n ", x3D.toString().c_str());
        

    double x0, y0, z0;
    double z = 0.35;
    double theta;

    // filling the zMeasure matrix with position on the plane of homography
    printf("entering the loop for necessary to perform high level tracking \n");
    for (short n = 0; n < nIter; n++) {
        
        tracker->getPoint(p_curr);
        timeStop = Time::now();
        if (n == 0) {
            // initialisation of the starting point of the traj.
            p_prev =  p_curr; 
            Vector px(2);
            px(0) = p_curr.x; 
            px(1) = p_curr.y;
            igaze->get3DPointOnPlane(camSel,px,plane,x3D);
            //igaze->get3DPoint(camSel,px,z, x3D);
            x0 = x3D(0);
            y0 = x3D(1);
            z0 = x3D(2);
        }
        else {
            timeDiff = timeStop - timeStart;
            //printf("----------------- \n timeDiff %f \n", timeDiff );
            distX = p_curr.x - p_prev.x;
            distY = p_curr.y - p_prev.y;
            Vector px(2);
            px(0) = p_curr.x; 
            px(1) = p_curr.y;
            igaze->get3DPointOnPlane(camSel,px,plane,x3D);
            //igaze->get3DPoint(camSel,px,z,x3D);
            printf (     "%f %f %f\n", x3D(0) - x0, x3D(1) - y0, x3D(2) - z0);
            fprintf(fout,"%f %f %f\n", x3D(0) - x0, x3D(1) - y0, x3D(2) - z0);


            distX =  x3D(1) - y0;
            distY =  x3D(2) - z0;
            dist_prev = dist;
            dist  = sqrt((double)distX * distX + distY * distY);
            theta = atan2(distY, distX);
            printf("travelled distance %f angle %f \n", dist, theta);
            
            zMeasurements2D(n - 1, 0) = dist;
            zMeasurements3D(n - 1, 0) = dist;

            //velX_prev = velX;
            //velY_prev = velY;
            //velX = (distX - distX_prev) / timeDiff;
            //velY = (distY - distY_prev) / timeDiff;
            //vel = sqrt( velX * velX + velY * velY);

            vel_prev = vel;
            vel = (dist - dist_prev) / timeDiff;
            zMeasurements2D(n - 1, 1) = vel;
            zMeasurements3D(n - 1, 1) = vel;

            //accX = (velX - velX_prev) / timeDiff;
            //accY = (velY - velY_prev) / timeDiff;
            //acc  = sqrt( accX * accX + accY * accY);
             
            acc  = (vel - vel_prev) / timeDiff;
            zMeasurements3D(n - 1, 2) = acc;

            //if(accY > maxAccY) { 
            //    maxAccY = accY;
            //}
            //if(accX > maxAccX) {
            //    maxAccX = accX;
            //}
            
            if(acc > maxAcc) {
                maxAcc = acc;
            }
            
            //meanVelX += velX;
            //meanVelY += velY;
        }
        timeStart = Time::now();
        Time::delay(0.05);
    }
    
    meanVelX /= nIter;
    meanVelY /= nIter;

    //uMeasurements(1,0) = 2.0; uMeasurements(0,0) = 4.0;
    //Measurements(0,0) = 3.0; zMeasurements(1,1) = 1.0;
    
    //estimate the predictor model that best fits the velocity measured
    //printf("setting measurements \n z = \n %s \n", zMeasurements.toString().c_str());
    //printf("u = \n %s \n", uMeasurements.toString().c_str());
    
    //eval->setMeasurements(uMeasurements,zMeasurements);     // alternative for debug

    // pointer to the beginning of the evalQueue
    evalQueue::iterator it;
    evalThread* tmp;
    it = eQueue->begin();
    //printf("got the pointer to the evalThread %08x \n", (*it));
    //Vector xCheck = (*it)->getX();
    //printf(" xCheck = \n %s \n", xCheck.toString().c_str());
    
    
    //tmp = *it;  // copy using pointer to the thread
    //tmp->setMeasurements(uMeasurements,zMeasurements);
    //printf("entering the loop for %08X with getdatReady %d \n",tmp, tmp->getDataReady());

    
    //starting different evalution threads
    while(it != eQueue->end() ) { 
        printf("____________________________________________________________________________________________________\n");
        tmp = *it;  // pointer to the thread
        printf("reading evalThread reference from the queue it = %08X \n", tmp);
        tmp->setMeasurements(uMeasurements,zMeasurements2D);
        printf("entering the loop with getdatReady %d \n", tmp->getDataReady());
        printf("getEvalFineshed value %d \n", tmp->getEvalFinished());
        it++;   
        printf("____________________________________________________________________________________________________\n \n \n");
    }
    printf("out of the loop that starts the predicotrs \n");


    // waiting for the evaluation already started
    printf("entering the loop with eval \n");
    printf("---------------------------- GETEVALFINISHED %d \n",eval->getEvalFinished() );
    it = eQueue->begin();
    int finished  = 0 ;
    double minMSE = 1000000;
    evalThread* minPredictor = null;
    
    while(finished < eQueue->size()) {
        // printf("eval evaluation %d < %d \n",finished, eQueue->size() );
        Time::delay(0.1);
        while(it != eQueue->end() ) {
            if((*it)->getEvalFinished()){
                finished++;
                printf(" predictor ends estimation.state %s \n", (*it)->getX().toString().c_str());
                double currentMSE = (*it)->getMSE();
                printf(" predictor ends with error %f       \n", currentMSE);
                
                if( currentMSE <  minMSE) {
                    minMSE = currentMSE;
                    minPredictor = (*it);
                }
            }
            it++;
        }
        it = eQueue->begin();
    }
    printf("eval evaluatio ended. fineshed=%d >= size=%d \n",finished, eQueue->size() );
    //printf("---------------------------- GETEVALFINISHED %d \n",eval->getEvalFinished() );
    
    if(minPredictor == 0) {
        printf("no predictor found \n");
    }
    else {
        printf("found the predictor %08X that minimises the MSE \n", minPredictor);
    }

    
    tracker->getPoint(p_curr);
    distance = std::sqrt((double)(p_curr.x - 160) * (p_curr.x - 160) + (p_curr.y - 120) * (p_curr.y - 120));

    bool predictionAccompl = true;
    Vx = meanVelX;
    Vy = meanVelY;
    xPos = -1;
    yPos = -1;
   
    double maxAccCart = maxAccX > maxAccY?maxAccX:maxAccY;
    //time = maxAcc / 5000; 
    time = 0.18;
    return predictionAccompl;
}

void trajectoryPredictor::run() {
    printf(" trajectoryPredictor::run %d %d \n", numIter, numEvalVel); 

    //Time::delay(5.0);
    
    
    
    //it2 = eQueue->begin();
    //printf("got the pointer to the evalThread %08x \n", (*it2));
    //Vector xCheck2 = (*it2)->getX();
    //printf(" xCheck2 = \n %s \n", xCheck2.toString().c_str());
    

    // trajectory predictor does not need active run anymore.
    // estimateVelocity function is called from the att.Prioritiser
    
    while(!isStopping()){

        /*
        it = eQueue->begin();
        printf("got the pointer %d  to the evalThread in run %08x \n",eQueue->size(),(*it));
        Vector xCheck = (*it)->getX();
        printf(" xCheck = \n %s \n", xCheck.toString().c_str());

        
        //ImageOf<PixelMono>* b = inImagePort.read(true);
        //  printf("after the imagePort \n");
        */
        
        /*
        printf("trajectoryPreditctor in run %d %d \n", numIter, numEvalVel);
        evalQueue::iterator it, it2;
        //it = eQueue->begin();
        //printf("got the pointer to the evalThread %08x \n", (*it));
        //Vector xCheck = (*it)->getX();
        //printf(" xCheck = \n %s \n", xCheck.toString().c_str());

        Vector xTraj = eval->getX();
        printf(" xTraj = \n %s \n", xTraj.toString().c_str());
        */
        
        //evalQueue::iterator it;
        //evalThread* tmp;
        //it = eQueue->begin();
        //printf("got the pointer to the evalThread %08x \n", (*it));
        //Vector xCheck = (*it)->getX();
        //printf(" xCheck = \n %s \n", xCheck.toString().c_str());
        
        
        /*
        // estimating velocity
        int x,y;
        double Vx, Vy, xPos, yPos, time, distance;
        //extractCentroid(b, x, y);
        estimateVelocity(x, y, Vx, Vy, xPos, yPos, time, distance);
        printf("estimateVelocity %f %f \n",Vx,Vy );
        */
        
        Time::delay(5.0);
    
    }

    
    
}

void trajectoryPredictor::onStop() {
    printf("trajectoryPredictor::onStop() : closing ports \n");
    inImagePort.interrupt();
    inImagePort.close();
    printf("trajectoryPredictor::onStop() : success in closing ports \n");
}

void trajectoryPredictor::threadRelease() {
    printf("trajectoryPredictor::threadRelease() : \n");
    //inImagePort.close();

    //if(0 != tracker) {
    //    printf("trajectoryPredictor::threadRelease:stopping the tracker \n");
    //    tracker->stop();
    //}

    //evalVel1.stop();
    //evalAcc1.stop();
    //evalMJ1_T1.stop();
    
}
