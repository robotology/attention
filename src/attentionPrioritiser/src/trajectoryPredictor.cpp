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
    fout      = fopen("./attPrioritiser.trajectoryPredictor.3Dtraj.txt","w+");
    fMeasure  = fopen("./attPrioritiser.trajectoryPredictor.measure.txt","w+");
    

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

    // _______________________ LINEAR ACCELERATION MODELS _______________________________________
    // ------------------------------------------------------------------------------------------
    
    linAccModel* modelB = new linAccModel();
    
    int rowB = modelB->getA().rows();
    int colB = modelB->getA().cols();
    Vector z0(rowB);
    Vector x0(rowB);
    x0.zero();z0.zero();
    x0(0) = 1.0; 
    Matrix P0(rowB,colB);
    //printf("initialisation of P0 %d %d \n", rowA, colA);
    for (int i = 0; i < rowB; i++) {
        for (int j = 0; j < colB; j++) { 
            P0(i,j) += 0.01;
        }      
    }
    //printf("lin.accModel : modelB\n %s \n %s \n", modelB->getA().toString().c_str(),modelB->getB().toString().c_str());    
    //printf("P0\n %s \n", P0.toString().c_str());    

    //---------------------------------------------------------------------------
    
    
    printf(" creating eval thread \n");
    modelB->init(9.8);
    genPredModel* mB = dynamic_cast<genPredModel*>(modelB);
    eval = new evalThread(*mB);
    eval->init(z0,x0,P0);
    printf("genPred model A \n %s \n",mB    ->getA().toString().c_str());
    printf("lin acc model A \n %s \n",modelB->getA().toString().c_str());
    printf("just initialised genPredModel %08X \n",&eval);
    eval->start();
    eQueue->push_back(eval); 


    // _______________________ LINEAR VELOCITY MODELS _______________________________________
    // ------------------------------------------------------------------------------------------
    
    linVelModel* modelA = new linVelModel();
    
    int rowA = modelA->getA().rows();
    int colA = modelA->getA().cols();
    Vector z0a(rowA);
    Vector x0a(rowA);
    x0a.zero();z0a.zero();
    x0a(0) = 1.0; 
    Matrix P0a(rowA,colA);
    //printf("initialisation of P0 %d %d \n", rowA, colA);
    for (int i = 0; i < rowA; i++) {
        for (int j = 0; j < colA; j++) { 
            P0a(i,j) += 0.01;
        }      
    }
   

    //---------------------------------------------------------------------------
    
    
    printf(" creating evalThread constant velocity \n");
    modelA->init(0.1);
    genPredModel* mA = dynamic_cast<genPredModel*>(modelA);
    eval = new evalThread(*mA);
    eval->init(z0a,x0a,P0a);
    printf("genPred model A \n %s \n",mA    ->getA().toString().c_str());
    printf("lin acc model A \n %s \n",modelA->getA().toString().c_str());
    printf("just initialised genPredModel %08X \n",&eval);
    eval->start();
    eQueue->push_back(eval); 
    
    

    //------------------------------------------------------------------------------
    /*
    printf("moving to the next predictor \n");
    modelB = new linAccModel();
    modelB->init(2.0);
    mB = dynamic_cast<genPredModel*>(modelB);
    eval = new evalThread(*mB);
    eval->init(z0,x0,P0);
    //eval->start();
    //eQueue->push_back(eval); 
    */

    // _______________________ MINIMUM JERK MODELS  _______________________________________
    // ------------------------------------------------------------------------------------
    
    minJerkModel* modelC = new minJerkModel();
    modelC->init(0.5, 1.0);
    
    int rowC = modelC->getA().rows();
    int colC = modelC->getA().cols();
    Vector z0c(rowC);
    Vector x0c(rowC);
    x0c.zero();
    z0c.zero();
    //x0c(0) = 1.0; 
    Matrix P0c(rowC,colC);
    for (int i = 0; i < rowC; i++) {
        for (int j = 0; j < colC; j++) { 
            P0c(i,j) += 0.01;
        }      
    }

    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    genPredModel* mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 
    
    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.5,2.0);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 
    
    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.5,0.5);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval);

    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.5,1.5);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval);

    //------------------------------------------------------------------------------
    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.4,0.5);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 

    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.4,1.0);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 

    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.4,1.5);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 

    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.4,2.0);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 
    
    //------------------------------------------------------------------------------
    //------------------------------------------------------------------------------

    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.3,0.5);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 

    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.3,1.0);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval);

    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.3,1.5);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval);

    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.3,2.0);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 
    
    //------------------------------------------------------------------------------
    //------------------------------------------------------------------------------

    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.2,0.5);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 

    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.2,1.0);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 
    
    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.2,1.5);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 

    //------------------------------------------------------------------------------
    
    printf("----------------------------------- \n MinJerk Model : initialisation of model %d \n", eQueue->size());
    modelC = new minJerkModel();
    modelC->init(0.2,2.0);
    mC = dynamic_cast<genPredModel*>(modelC);
    eval = new evalThread(*mC);
    eval->init(z0c,x0c,P0c);
    eval->start();
    eQueue->push_back(eval); 
    
    //------------------------------------------------------------------------------
    
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



bool trajectoryPredictor::estimateVelocity(int x, int y, double& Vx, double& Vy, double& xPos, double& yPos, double& zPos, double& time, double& distance) {
    printf(" trajectoryPredictor::estimateVelocity in pos.%d,%d  \n", Vx, Vy);
    
    CvPoint p_curr, p_prev;

    double timeStart   = Time::now();
    double timeInitial = timeStart;
    double timeMeas;
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
    
    int nIter = 40;


    // hard coded prediction
    Vx = Vy = 0;
    xPos = yPos = zPos = -1;   

    
    // //for n times records the position of the object and extract an estimate
    // // extracting the velocity of the stimulus; saving it into a vector 
    Matrix zMeasurements2D(nIter,2);
    Matrix zMeasurements3D(nIter,3);
    Matrix uMeasurements(2.0, nIter);
    
    
    //passing from the 2D image plane to the 3D real location using homography
    //projectOnPlane(0,0,0,1,0,0);
    //get3DPointOnPlane (const int camSel, const yarp::sig::Vector &px, const yarp::sig::Vector &plane, yarp::sig::Vector &x)=0
    int camSel = 0; //left camera 0-left 1-right
    Vector px(2);
    px(0) = 160; 
    px(1) = 120;
    //representing on the plane 1x + 0y + 0z = -0.5
    Vector plane(4);
    plane(0) = 1;
    plane(1) = 0;
    plane(2) = 0;
    plane(3) = 0.5;

    
    Vector x3D(3);
    igaze->get3DPointOnPlane(camSel,px,plane,x3D);
    printf("3dposition on the plane extract by gazeController %s \n ", x3D.toString().c_str());
        

    double x0, y0, z0;
    double z = 0.35;
    double theta;


    fprintf(fMeasure,"pCurr.x pCurr.y timeMeas dist theta timeDiff vel acc \n");

    // filling the zMeasure matrix with position on the plane of homography
    printf("entering the loop for necessary to perform high level tracking \n");
    for (short n = 0; n < nIter; n++) {
        
        tracker->getPoint(p_curr);
        
        timeStop = Time::now();
        if (n == 0) {
            // initialisation of the starting point of the traj.
            p_prev =  p_curr; 
            
            px(0) = p_curr.x; 
            px(1) = p_curr.y;
            igaze->get3DPointOnPlane(camSel,px,plane,x3D);
            //igaze->get3DPoint(camSel,px,z, x3D);
            x0 = x3D(0);
            y0 = x3D(1);
            z0 = x3D(2);
        }
        else {
            fprintf(fMeasure,"%d %d ",p_curr.x, p_curr.y); 

            timeDiff = timeStop - timeStart;
            timeMeas = Time::now() - timeInitial;
            //printf("----------------- \n timeDiff %f \n", timeDiff );
            //distX = p_curr.x - p_prev.x;
            //distY = p_curr.y - p_prev.y;
            
            px(0) = p_curr.x; 
            px(1) = p_curr.y;
            igaze->get3DPointOnPlane(camSel,px,plane,x3D);
            //igaze->get3DPoint(camSel,px,z,x3D);
            //printf (     "%f %f %f\n", x3D(0) - x0, x3D(1) - y0, x3D(2) - z0);
            fprintf(fout,"%f %f %f\n", x3D(0) , x3D(1) , x3D(2));


            distX =  x3D(1) - y0;
            distY =  x3D(2) - z0;
            dist_prev = dist;
            dist  = sqrt((double)distX * distX + distY * distY);
            theta = atan2(distY, distX);
            //printf("travelled distance %f angle %f \n", dist, theta);
            
            fprintf(fMeasure,"%f ",timeMeas); 
            fprintf(fMeasure,"%f ",dist);
            fprintf(fMeasure,"%f ",theta);
            
            //calculating space
            zMeasurements2D(n - 1, 0) = dist;
            zMeasurements3D(n - 1, 0) = dist;

            //velX_prev = velX;
            //velY_prev = velY;
            //velX = (distX - distX_prev) / timeDiff;
            //velY = (distY - distY_prev) / timeDiff;
            //vel = sqrt( velX * velX + velY * velY);
            
            // calculating velocity
            vel_prev = vel;
            vel = (dist - dist_prev) / 0.033;
            zMeasurements2D(n - 1, 1) = vel;
            zMeasurements3D(n - 1, 1) = vel;
            
            fprintf(fMeasure,"%f ",timeDiff);
            fprintf(fMeasure,"%f ",vel);

            //accX = (velX - velX_prev) / timeDiff;
            //accY = (velY - velY_prev) / timeDiff;
            //acc  = sqrt( accX * accX + accY * accY);
             
            // calculating acceleration
            acc  = (vel - vel_prev) / timeDiff;
            zMeasurements3D(n - 1, 2) = acc;
            
            fprintf(fMeasure,"%f \n",acc);

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
        Time::delay(0.033);
    }
    
    meanVelX /= nIter;
    meanVelY /= nIter;
    
    printf("ready to save the measure matrix\n %s \n",zMeasurements3D.toString().c_str() );
    //fprintf(fMeasure, "%s\n",zMeasurements3D.toString().c_str());
    
    //estimate the predictor model that best fits the velocity measured
    //printf("setting measurements \n ");
    //printf("u = \n %s \n", uMeasurements.toString().c_str());

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

        
        //creating the uMeasurement vector
        double paramA = tmp->getParamA();
        for (int i = 0; i < 2; i++) {
            for (int j =0 ; j < nIter; j++) {
                uMeasurements(i, j) = paramA;
            }
        }

        if(tmp->getRowA() == 2) {
            printf("dimension of the measure %d %f \n",tmp->getRowA(), paramA );
            tmp->setMeasurements(uMeasurements,zMeasurements2D);
        }
        else {
            printf("dimension of the measure %d %f \n",tmp->getRowA(), paramA );
            tmp->setMeasurements(uMeasurements,zMeasurements3D);
        }
        printf("entering the loop with getdatReady %d \n", tmp->getDataReady());
        printf("getEvalFineshed value %d \n", tmp->getEvalFinished());
        it++;   
        printf("____________________________________________________________________________________________________\n \n \n");
    }
    printf("out of the loop that starts the predictors \n");
    fprintf(fMeasure,"--------\n "); 


    // waiting for the evaluation already started
    printf("entering the loop with eval \n");
    printf("---------------------------- GETEVALFINISHED %d \n",eval->getEvalFinished() );
    it = eQueue->begin();
    int finished  = 0 ;
    minMSE = 1;                      // minMSE = 1 is the lower limit beyond the which any predictor is not considered successful
    evalThread* minPredictor = 0;    // if any predictor fails the pointer to the predictor remains null
    
    while(finished < eQueue->size()) {
        printf("eval evaluation %d < %d \n",finished, eQueue->size() );
        Time::delay(0.1);
        while(it != eQueue->end() ) {
            if( (*it)->getEvalFinished()) {
                (*it)->setEvalFinished(false);    // setting back to false the variable otherwise multiple detection can happen 
                finished++;
                printf(" predictor %s %f %f ends estimation.state %s \n",
 (*it)->getType().c_str(), (*it)->getParamA(), (*it)->getParamB(),(*it)->getX().toString().c_str());
                
                double currentMSE = (*it)->getMSE();
                fprintf(fMeasure," predictor %s distance:%f period:%f ends; estimation.state:%s MSE:%f ",
                        (*it)->getType().c_str(), (*it)->getParamA(), (*it)->getParamB(),(*it)->getX().toString().c_str(), currentMSE);
                printf("  error: %f       \n", currentMSE);
                
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
        predictionAccompl = false;
        return predictionAccompl;
    }
    else {
        printf("found the predictor %s %f %f  that minimises the MSE %f \n",minPredictor->getType().c_str(), minPredictor->getParamA(), minPredictor->getParamB(), minMSE);
        predictionAccompl = true;
    }


    // preparing the output of the method : either presumed 3d position or velocity profile

    if(!strcmp(minPredictor->getType().c_str(), "constVelocity")) {
        tracker->getPoint(p_curr);
        distance = std::sqrt((double)(p_curr.x - 160) * (p_curr.x - 160) + (p_curr.y - 120) * (p_curr.y - 120));
        bool predictionAccompl = true;
        Vx = meanVelX;
        Vy = meanVelY;

        double vel = minPredictor->getParamA();
        Vx = vel * cos(theta);
        Vy = vel * sin(theta);

        printf("comparison estimated velocity: Vx = %f (mean: %f)  Vy = %f (mean: %f)", Vx, meanVelX, Vy, meanVelY );
        
        xPos = -1;
        yPos = -1;
        double maxAccCart = maxAccX > maxAccY?maxAccX:maxAccY;
        //time = maxAcc / 5000; 
        time = 0.18;
    }
    else {
        double distance = minPredictor->getParamA();
        Vx = -1;
        Vy = -1;
        xPos = x0;
        yPos = y0 + distance * cos(theta);
        zPos = z0 + distance * sin(theta);
        printf("target %f %f : %f %f %f \n",distance, theta, xPos, yPos, zPos);
        fprintf(fMeasure,"target %f %f : %f %f %f \n",distance, theta, xPos, yPos, zPos);
    }

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
    fclose(fout);
    fclose(fMeasure);
   
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
