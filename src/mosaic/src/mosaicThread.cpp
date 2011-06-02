// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea, Shashank Pathak
 * email:   francesco.rea@iit.it, Shashank.pathak@iit.it
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
 * @file mosaicThread.cpp
 * @brief Implementation of the mosaic thread (see mosaicThread.h).
 */

#include <iCub/mosaicThread.h>
#include <yarp/math/SVD.h>
#include <cv.h>
#include <highgui.h>
#include <cstring>

#define MAXMEMORY 100
#define LEFT_EYE 0;
#define RIGHT_EYE 0;

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace yarp::math;
using namespace iCub::iKin;

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
                parType.check("fx") && parType.check("fy")) {
                // we suppose that the center distorsion is already compensated
                double cx = parType.find("w").asDouble() / 2.0;
                double cy = parType.find("h").asDouble() / 2.0;
                double fx = parType.find("fx").asDouble(); printf("fx %f \n", fx);
                double fy = parType.find("fy").asDouble(); printf("fy %f \n", fy);

                Matrix K=eye(3,3);
                Matrix Pi=zeros(3,4);

                K(0,0)=fx; K(1,1)=fy;
                K(0,2)=cx; K(1,2)=cy; 
                
                Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0; 

                *Prj=new Matrix;
                **Prj=K*Pi;

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

/**************************************************************************/
mosaicThread::mosaicThread() {
    inputImageLeft = new ImageOf<PixelRgb>;
    inputImageRight = new ImageOf<PixelRgb>;
    outputImageMosaic = new ImageOf<PixelRgb>;
    warpImLeft = new ImageOf<PixelRgb>;
    warpImRight = new ImageOf<PixelRgb>;
   
    robot = "icub"; 
    resized = false;
    memory = (float*) malloc(MAXMEMORY);
    memset(memory, 0, MAXMEMORY);
    countMemory = 0;
    azimuth = 0.0;
    elevation = 0.0;
    rectified = false;
}

mosaicThread::mosaicThread(string _robot, string _configFile) {
    //initialisation of variables
    countMemory = 0;
    elevation = 0.0;
    azimuth = 0.0;
    robot = _robot;
    configFile = _configFile;
    resized = false;
    rectified = false;
    //allocating memory
    inputImageLeft    = new ImageOf<PixelRgb>;
    inputImageRight   = new ImageOf<PixelRgb>;
    outputImageMosaic = new ImageOf<PixelRgb>;
    warpImLeft        = new ImageOf<PixelRgb>;
    warpImRight       = new ImageOf<PixelRgb>;
    memory = (float*) malloc(MAXMEMORY);
    memset(memory, 0, MAXMEMORY);
}

mosaicThread::~mosaicThread() {
    // freeing memory
    delete inputImageLeft;
    delete inputImageRight;
    delete outputImageMosaic;
    delete warpImLeft;
    delete warpImRight;
    free(memory);   
}

bool mosaicThread::threadInit() {
    /* open ports */ 
    if (!imagePortIn.open(getName("/image:i").c_str())) {
        cout <<": unable to open port for camera  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!imagePortInRight.open(getName("/right:i").c_str())) {
        cout <<": unable to open port for camera  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!imagePortOut.open(getName("/image:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!portionPort.open(getName("/portion:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    } 

    printf("\n robotname: %s \n",robot.c_str());

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
    
    
    //initialising the head polydriver
    printf("starting the polydrive for the head.... of the robot %s \n", robot.c_str());
    Property optHead("(device remote_controlboard)");
    string remoteHeadName="/" + robot + "/head";
    string localHeadName = name + "/head";
    optHead.put("remote",remoteHeadName.c_str());
    optHead.put("local",localHeadName.c_str());
    drvHead =new PolyDriver(optHead);
    if (!drvHead->isValid()) {
        fprintf(stdout,"Head device driver not available!\n");        
        delete drvHead;
        return false;
    }
    drvHead->view(encHead);

    //initialising the torso polydriver
    printf("starting the polydrive for the torso.... \n");
    Property optPolyTorso("(device remote_controlboard)");
    
    optPolyTorso.put("remote",("/" + robot + "/torso").c_str());
    optPolyTorso.put("local",(name + "/torso/position").c_str());
    
    printf("creating a new polydriver \n");
    polyTorso=new PolyDriver;
    if (!polyTorso->open(optPolyTorso))
    {
        printf("polydriver of the torso did not start correctly \n");
        return false;
    }
    polyTorso->view(encTorso);

    eyeL=new iCubEye("left");
    eyeR=new iCubEye("right");

    // release links
    eyeL->releaseLink(0);
    eyeR->releaseLink(0);
    eyeL->releaseLink(1);
    eyeR->releaseLink(1);
    eyeL->releaseLink(2);
    eyeR->releaseLink(2);

    printf("trying to CAMERA projection from %s.......... ", configFile.c_str());
    // get left camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        printf("SUCCESS in finding configuration of camera param \n");
        Matrix &Prj=*PrjL;
        cxl = Prj(0,2);
        cyl = Prj(1,2);
        fxl = Prj(0,0);
        fyl = Prj(1,1);
        printf("configuration param of the left camera %f %f %f %f \n",cxl,cyl,fxl,fyl);          
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    else { 
        printf("did not find the configuration file for the camera \n");
        return false; //PrjL=invPrjL=NULL;
    }

    // get right camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_RIGHT",&PrjR)) {
        printf("SUCCESS in finding configuration of camera param \n");
        Matrix &Prj=*PrjR;
        cxr = Prj(0,2);
        cyr = Prj(1,2);
        fxr = Prj(0,0);
        fyr = Prj(1,1);
        printf("configuration param of the right camera %f %f %f %f \n",cxr,cyr,fxr,fyr);                
        invPrjR=new Matrix(pinv(Prj.transposed()).transposed());
    }
    else { 
        printf("did not find the configuration file for the camera \n");
        return false; 
    }

    //initilization of the backprojection to the cyclopic plane
    Vector q(8);
    double ratio = M_PI /180;
    q[0]=0 * ratio;
    q[1]=0 * ratio;
    q[2]=0 * ratio;
    q[3]=0  * ratio;
    q[4]=0  * ratio;
    q[5]=0  * ratio;
    q[6]=0  * ratio;
    q[7]=0  * ratio;
    
    eyeH0 = new Matrix(4,4);
    eyeCyclopic = new iCubEye(*eyeL);
    iKinChain* chainCyclopic  = eyeCyclopic->asChain();
    iKinLink* link = &(chainCyclopic-> operator ()(6));
    link->setD(0.0);
    double dleft = link->getD();
    *eyeH0 = eyeCyclopic->getH(q);  
    //*eyeH0 = eyeL->getH(q);
    Matrix& eyeH_ref = *eyeH0;
    
    inveyeH0 = new Matrix(pinv(eyeH_ref.transposed()).transposed());

    printf("initilisation successfully ended \n");
    return true;
}


void mosaicThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string mosaicThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void mosaicThread::setMosaicDim(int w, int h) {
    printf("setting mosaic dimension \n");
    //mosaic image default size
    width = w;
    height = h;
    //default position of input image's center
    xcoord = floor(height / 2);
    ycoord = floor(width / 2);    
}

void mosaicThread::resize(int width_orig,int height_orig) {        
    this->width_orig = width_orig;
    this->height_orig = height_orig;
    printf("resizing using %d %d \n",width_orig,height_orig);
    inputImageLeft->resize(width_orig,height_orig);
    inputImageRight->resize(width_orig,height_orig);
    warpImLeft->resize(width_orig,height_orig);
    warpImRight->resize(width_orig,height_orig);
    
    printf("successfully resized \n");
}

bool mosaicThread::placeInpImage(int X, int Y) {
    if(X > width || X < 0 || Y > height || Y < 0) return false;
    this->xcoord = X;
    this->ycoord = Y;
    return true;
}

void mosaicThread::setInputDim(int w, int h) {
    //input image default size
    width_orig = w;
    height_orig = h;
}

void mosaicThread::plotObject(float x,float y,float z) {
    printf("saving in memory the 3dlocation \n");
    float* pointer = memory ;
    printf("countMemory %d \n", countMemory);
    pointer += countMemory * 3;
    *pointer = x; pointer++;
    *pointer = y; pointer++;
    *pointer = z;
    printf("saved the position %f,%f,%f \n",x,y,z);
    countMemory++;
}


void mosaicThread::setFetchPortion(float a, float e) {
    elevation = e;
    azimuth   = a;
}

void mosaicThread::fetchPortion(ImageOf<PixelRgb> *image) {
    //    printf("trying to fetch a particular position on the mosaic %f %f \n", azimuth, elevation);
    //determine the shift based on the focal length

    int shiftx =(int) floor(  fxl  * ((azimuth   * 3.14) / 180));
    int shifty =(int) floor( -fyl  * ((elevation * 3.14) / 180));
    
    unsigned char* mosaic = outputImageMosaic->getRawImage();
    unsigned char* portion =  image->getRawImage();
    int mosaicRowSize = outputImageMosaic->getRowSize();
    int mosaicPadding = outputImageMosaic->getPadding();
    int portionPadding = image->getPadding();
    mosaic += shiftx * 3 + shifty * mosaicRowSize; // + (width>>1) * 3 + (height>>1) * mosaicRowSize;
    //printf("height_orig width_orig %d %d mosaicRowSize %d \n", height_orig, width_orig,mosaicRowSize );
    for ( int row = 0 ; row < height_orig; row++) { 
        for (int cols = 0; cols< width_orig; cols++) {
            *portion++ = *mosaic++;  //red
            *portion++ = *mosaic++;  //green
            *portion++ = *mosaic++;  //blue
        }
        portion += portionPadding;
        mosaic  += mosaicRowSize - width_orig * 3;
    }
}

bool mosaicThread::setMosaicSize(int width=DEFAULT_WIDTH, int height=DEFAULT_HEIGHT) {
    if(width > MAX_WIDTH || width < width_orig || width <= 0 
       || height > MAX_HEIGHT || height < height_orig || height <= 0 ) {
        printf("setMosaicSize returned FALSE \n");
        return false;
    }
    else{
        printf("setMosaicSize returned TRUE \n");
    }
    this->width = width;
    this->height = height;

    //extraction the projection for the cyclopic plane
    double cx = width / 2; 
    double cy = height / 2; 
    double fx = fxl;        
    double fy = fyl;        
    
    
    Matrix K=eye(3,3);
    Matrix Pi=zeros(3,4);
    
    K(0,0)=fx; K(1,1)=fy;
    K(0,2)=cx; K(1,2)=cy; 
    
    Pi(0,0)=Pi(1,1)=Pi(2,2)=1.0; 
    
    cyclopicPrj  = new Matrix;
    *cyclopicPrj = K * Pi;

    printf("resizing the image with dimension %d %d \n", width, height);
    outputImageMosaic->resize(width,height);
    outputImageMosaic->zero();
    return true;    
}

void mosaicThread::run() {
    printf("Initialization of the run function %d %d .... \n", width, height);
    count++;
    outputImageMosaic->resize(width,height);
    outputImageMosaic->zero();
    while (isStopping() != true)  {                
        inputImageLeft = imagePortIn.read(false); // do not wait                          
        if (inputImageLeft != NULL ) {            
            if(!resized) {
                resize(inputImageLeft->width(),inputImageLeft->height());
                resized = true;
            }
            if(imagePortInRight.getInputCount()) {
                inputImageRight = imagePortInRight.read(false);                
                if(inputImageRight!=NULL) {
                    makeMosaic(inputImageLeft, inputImageRight); 
                }
            }
            if(imagePortOut.getOutputCount()) {
                imagePortOut.prepare() = *outputImageMosaic;
                imagePortOut.write();
            }
            if(portionPort.getOutputCount()) {
                ImageOf<PixelRgb>& portionImage = portionPort.prepare();
                portionImage.resize(320,240);
                fetchPortion(&portionImage);
                portionPort.write();
            }
        }                      
    }
}


void mosaicThread::makeMosaic(ImageOf<PixelRgb>* inputImageLeft, ImageOf<PixelRgb>* inputImageRight) {
    //recalculing the position in the space
    double u = 160;
    double v = 120;
    double z = 0.5;
    
    bool isLeft = true;
    Vector fp(3);
    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);
    Vector prec;
    double shift_prev;

    CvPoint2D32f *c1 = new CvPoint2D32f[4];
    CvPoint2D32f *c2 = new CvPoint2D32f[4];
    CvPoint2D32f *cr = new CvPoint2D32f[4];
    
    
    if ((invPrj)&&(rectified)) {
        
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
        q[1]=torso[1] * ratio;
        q[2]=torso[2] * ratio;
        q[3]=head[0]  * ratio;
        q[4]=head[1]  * ratio;
        q[5]=head[2]  * ratio;
        q[6]=head[3]  * ratio;
        q[7]=head[4]  * ratio;
        double ver = head[5];
        //printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
               
                        
        Vector x(3);
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
                
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector xeLeft(4),xeRight(4);
        Vector xe = yarp::math::operator *(*invPrj, x);
        xeLeft  = yarp::math::operator *(*invPrjL, x);
        xeRight = yarp::math::operator *(*invPrjR, x);
        xe[3]     = 1.0;  // impose homogeneous coordinates                
        xeLeft[3] = 1.0;
        xeRight[3]= 1.0;
                
        // update position wrt the root frame
        eyeHL = new Matrix(4,4);
        *eyeHL = eyeL->getH(q);        
        eyeHR = new Matrix(4,4);
        *eyeHR = eyeR->getH(q);
        //Matrix* inveyeHL  = new Matrix(pinv(eyeHL->transposed()).transposed());
        
        //printf(" %f %f %f ", eyeH(0,0), eyeH(0,1), eyeH(0,2));
        Vector xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);
        Vector xoRight = yarp::math::operator *(*eyeHR,xeRight);
        
        prec = fp;
        fp.resize(3,0.0);
        fp[0]=xoLeft[0];
        fp[1]=xoLeft[1];
        fp[2]=xoLeft[2];
        printf("object %f,%f,%f \n",fp[0],fp[1],fp[2]);
        
        c2[0].x = 0;    c2[0].y = 0;   
        c2[1].x = 320;  c2[1].y = 0;   
        c2[2].x = 0;    c2[2].y = 240; 
        c2[3].x = 320;  c2[3].y = 240; 

        Vector x_hat(4);

        //___________________________________________________________
        //
        // extracting the warp perspective vertix
        //

        u = 0;
        v = 0;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        
        //------------------------------------------
        //printf("vertix %f,%f,%f \n",xoLeft[0],xoLeft[1],xoLeft[2]); 
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);
        xeLeft = yarp::math::operator *(*inveyeH0,xoLeft);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeLeft);
        c1[0].x = x_hat[0]/z;   c1[0].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]=1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight);
        xeRight = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[0].x = x_hat[0]/z;   cr[0].y = x_hat[1]/z;
        

        //___________________________________________________________
        
        u = 320;
        v = 0;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        //------------------------------------------
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);                
        xeLeft = yarp::math::operator *(*inveyeH0,xoLeft);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeLeft);
        c1[1].x = x_hat[0]/z;   c1[1].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]=1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight); 
        xeRight = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[1].x = x_hat[0]/z;   cr[1].y = x_hat[1]/z;
        

        //__________________________________________________________

        u = 0;
        v = 240;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        //------------------------------------------
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);
        xeLeft = yarp::math::operator *(*inveyeH0,xoLeft);       
        x_hat = yarp::math::operator *(*cyclopicPrj, xeLeft);
        c1[2].x = x_hat[0]/z;   c1[2].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]=1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight);
        xeRight = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[2].x = x_hat[0]/z;   cr[2].y = x_hat[1]/z;
        
        //________________________________________________________

        u = 320;
        v = 240;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;        
        //------------------------------------
        xeLeft = yarp::math::operator *(*invPrjL, x);
        xeLeft[3]=1.0;  // impose homogeneous coordinates
        xoLeft  = yarp::math::operator *(*eyeHL,xeLeft);               
        xeLeft = yarp::math::operator *(*inveyeH0,xoLeft);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeLeft);
        c1[3].x = x_hat[0]/z;   c1[3].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0]/z, x_hat[1]/z, x_hat[2]);
        //------------------------------------
        xeRight = yarp::math::operator *(*invPrjR, x);
        xeRight[3]=1.0;  // impose homogeneous coordinates
        xoRight  = yarp::math::operator *(*eyeHR,xeRight); 
        xeRight = yarp::math::operator *(*inveyeH0,xoRight);
        x_hat = yarp::math::operator *(*cyclopicPrj, xeRight);
        cr[3].x = x_hat[0]/z;   cr[3].y = x_hat[1]/z;

        printf("dimension %f %f",  c1[1].x - c1[0].x , c1[3].x - c1[2].x);
        double dimensionX = c1[1].x - c1[0].x;              
    }

    double distancey = fp[1] - prec[1];
    //Vector angles = eye->getAng();
    Vector x(3), o(4);
    //igaze->getLeftEyePose(x,o);
    igaze->getAngles(x);
    //printf("angles %f, %f, %f, %f, %f, %f, %f, %f \n", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6],  angles[7] );
    //printf("o %f %f %f \n",o[0],o[1],o[2]);
    //printf("distancey %f   ",distancey);
    //calculating the shift in pixels
    double focalLenght = 200;
    double distance = z;
    double baseline = 0.068;
    
    
    // making the mosaic
    int i,j;
    
    unsigned char* outTemp = outputImageMosaic->getRawImage();
    unsigned char* lineOutTemp;
    int iW = inputImageLeft->width();
    int iH = inputImageLeft->height();
    int mPad = outputImageMosaic->getPadding();
    int inputPadding = inputImageLeft->getPadding();
    int rowSize = outputImageMosaic->getRowSize();   
    
    if(rectified) {
        CvMat* mmat = cvCreateMat(3,3,CV_32FC1);
        CvMat* mmatRight = cvCreateMat(3,3,CV_32FC1);
        
        float azimuth = ((x[0] * 3.14) / 180);
        float elevation = ((x[1] * 3.14) / 180);
        printf("angles %f %f",azimuth , elevation);
        
        c1[0].x = c1[0].x * 0.6; 
        c1[0].y = c1[0].y * 0.6;//  - (dimensionX-50)  * 0.1;
        c1[1].x = c1[1].x * 0.6; 
        c1[1].y = c1[1].y * 0.6;//  - (dimensionX-50)  * 0.1;
        c1[2].x = c1[2].x * 0.6; 
        c1[2].y = c1[2].y * 0.6;//  + (dimensionX+50)  * 0.1;
        c1[3].x = c1[3].x * 0.6; 
        c1[3].y = c1[3].y * 0.6;//  + (dimensionX+50)  * 0.1;
    
        cr[0].x = cr[0].x * 0.6; 
        cr[0].y = cr[0].y * 0.6;//  - (dimensionX-50)  * 0.1;
        cr[1].x = cr[1].x * 0.6; 
        cr[1].y = cr[1].y * 0.6;//  - (dimensionX-50)  * 0.1;
        cr[2].x = cr[2].x * 0.6; 
        cr[2].y = cr[2].y * 0.6;//  + (dimensionX+50)  * 0.1;
        cr[3].x = cr[3].x * 0.6; 
        cr[3].y = cr[3].y * 0.6;//  + (dimensionX+50)  * 0.1;
        
        //c1[0].x = 0.5 * c2[0].x - abs(azimuth - 0.1)  * 100; 
        //c1[0].y = 0.5 * c2[0].y - abs(azimuth - 0.1)  * 60;
        //c1[1].x = 0.5 * c2[1].x - abs(azimuth + 0.1)  * 100;
        //c1[1].y = 0.5 * c2[1].y - abs(azimuth + 0.1)  * 60;
        //c1[2].x = 0.5 * c2[2].x - abs(azimuth - 0.1)  * 100;
        //c1[2].y = 0.5 * c2[2].y + abs(azimuth - 0.1)  * 60;
        //c1[3].x = 0.5 * c2[3].x - abs(azimuth + 0.1)  * 100;
        //c1[3].y = 0.5 * c2[3].y + abs(azimuth + 0.1)  * 60;
        
        //cr[0].x = c2[0].x;   cr[0].y = c2[0].y;
        //cr[1].x = c2[1].x;   cr[1].y = c2[1].y;
        //cr[2].x = c2[2].x;   cr[2].y = c2[2].y;
        //cr[3].x = c2[3].x;   cr[3].y = c2[3].y;
        
        printf("getting perspective transform \n");   
        mmat = cvGetPerspectiveTransform(c2, c1, mmat);
        mmatRight = cvGetPerspectiveTransform(c2,cr,mmatRight);
        
        //float* dataLeft = mmat->data.fl;
        //float* dataRight = mmatRight->data.fl;
        //for (int i=0; i<3; i++) {
        //    for (int j=0; j<3; j++){
        //        dataLeft[i * 3 + j] = eyeHL->operator()(i,j);
        //        dataRight[i * 3 + j] = eyeHR->operator()(i,j);
        //    }
        // }
        
        cvWarpPerspective((CvArr*)inputImageLeft->getIplImage(),(CvArr*)warpImLeft->getIplImage(),mmat,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalar(255,3,3));
        cvWarpPerspective((CvArr*)inputImageRight->getIplImage(),(CvArr*)warpImRight->getIplImage(),mmatRight,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS,cvScalar(255,3,3));
    }
    else {
        warpImLeft = inputImageLeft;
        warpImRight = inputImageRight;
    }

    
    
    unsigned char* inpTemp = warpImLeft->getRawImage();
    unsigned char* inpTempRight = warpImRight->getRawImage(); 
    int inpRowsize = warpImLeft->getRowSize();

    int mosaicX, mosaicY;
    int mosaicXRight, mosaicYRight;
    shift_prev = shiftx;
    shiftx =  fxl * 1.0  * ((x[0] * 3.14) / 180);
    shifty =  -fyl * 1.0  * ((x[1] * 3.14) / 180);
    shiftxRight = fxr * 1.0 * ((x[0] * 3.14) / 180);
    shiftyRight = -fyr * 1.0 * ((x[1] * 3.14) / 180);
    printf(" shiftx %f shifty %f  shiftx %f shifty %f  \n",shiftx,shifty, shiftxRight,shiftyRight);        
        
    ycoord = shiftx + floor(width / 2);
    xcoord = shifty + floor(height / 2);
    ycoordRight = shiftxRight + floor(width / 2);
    xcoordRight = shiftyRight + floor(height / 2);

    mosaicX = ycoord ;
    mosaicX -= floor(iH / 2);
    mosaicY = xcoord;
    mosaicY -= floor(iW / 2);
    mosaicXRight = ycoordRight ;
    mosaicXRight -= floor(iH / 2);
    mosaicYRight = xcoordRight ;
    mosaicYRight -= floor(iW / 2);
    float alfa = 0.96;

    //int dimWarpX = max(c1[1].x, c1[3].x) - min(c1[0].x,c1[2].x);
    //int dimWarpY = max(c1[2].y, c1[3].y) - min(c1[0].y,c1[1].y);    
    //printf("%f %f %f %f", c1[0].x,c1[1].x,c1[2].x,c1[3].x);
    //printf("%f %f %f %f", c1[0].y,c1[1].y,c1[2].y,c1[3].y);
    //printf("dimWarp: %d %d ", dimWarpX, dimWarpY);
    
    outTemp = lineOutTemp = outTemp + mosaicY * (rowSize + mPad) + 3 * mosaicX;

    for(i = 0 ; i < iH ; ++i) {
        for(j = 0 ; j < iW ; ++j) {
           *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTemp);
            inpTemp++; outTemp++;
            
            *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTemp); 
            inpTemp++; outTemp++;
            
            *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTemp);
            inpTemp++; outTemp++; 
        }
        inpTemp      += inputPadding;
        outTemp      =  lineOutTemp = lineOutTemp + (rowSize + mPad);
    }
    
    
    if(rectified) {
        outTemp = outputImageMosaic->getRawImage();
        outTemp = lineOutTemp = outTemp + mosaicYRight * (rowSize + mPad) + 3 * mosaicXRight;
        for(i = 0 ; i < iH ; ++i) {
            for(j = 0 ; j < iW ; ++j) {
                *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTempRight);
                inpTempRight++; outTemp++;
                
                *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTempRight); 
                inpTempRight++; outTemp++;
                
                *outTemp = (unsigned char) floor(alfa * *outTemp + (1- alfa) * *inpTempRight);
                inpTempRight++; outTemp++;
            }
            inpTempRight += inputPadding;
            outTemp      =  lineOutTemp = lineOutTemp + (rowSize + mPad);
        }
    }

    //deleting previous locations in the image plane
    //representing new locations on the image plane
    float* pointer = memory;
    //int* pointerimage = memoryimage;
    double distanceEyeObject ;
    Vector xeye(4);
    
    for(int k = 0; k < countMemory; k++) {
        Vector xoi(4);
        Vector xei_or(4);
        xoi[0] = *pointer++;
        xoi[1] = *pointer++;
        xoi[2] = *pointer++;
        xoi[3] = 1.0; // impose homogeneous coordinate 
        igaze->getLeftEyePose(xeye,xei_or);
        printf("eyePose %f %f %f \n",xeye[0],xeye[1],xeye[2]);
        distanceEyeObject = sqrt ( (xoi[0] - xeye[0]) * (xoi[0] - xeye[0]) +
                          (xoi[1] - xeye[1]) * (xoi[1] - xeye[1]) + 
                          (xoi[2] - xeye[2]) * (xoi[2] - xeye[2]) );

        // update position wrt the eye frame
        Matrix* inveyeH=new Matrix(pinv(eyeHL->transposed()).transposed());         
        Vector xei = yarp::math::operator *(*inveyeH, xoi);        
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector x = yarp::math::operator *(*PrjL, xei);
        printf("x: %f, %f, %f distanceEyeObject %f \n", x[0],x[1], x[2], distanceEyeObject);
        int ui = (int) floor(x[0] / distanceEyeObject);
        int vi = (int) floor(x[1] / distanceEyeObject);
        printf ("x %f y %f z %f , mosaicx %d mosaicy %d >>>> u %d v %d \n", xei[0], xei[1], xei[2],mosaicX, mosaicY,ui, vi);
        //pointerimage = pointerimage + counterMemoryImage * 2;
        //*pointerimage = ui; pointerimage++;
        //*pointerimage = vi;
        unsigned char* outTemp = outputImageMosaic->getRawImage();
        outTemp = outTemp + (height / 2) * (rowSize + mPad) + 3 * (width / 2) + vi * (rowSize + mPad) + ui * 3;
        *outTemp++ = 255;
        *outTemp++ = 0;
        *outTemp++ = 0;
    }
}


void mosaicThread::threadRelease() {
    resized = false;     
}

void mosaicThread::onStop() {
    imagePortIn.interrupt();
    imagePortOut.interrupt();
    portionPort.interrupt();
        
    imagePortOut.close();
    imagePortIn.close();
    portionPort.close();
}

