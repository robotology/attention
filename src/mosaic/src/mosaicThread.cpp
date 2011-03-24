// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
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
#include <cstring>

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
                parType.check("fx") && parType.check("fy"))
            {
                // we suppose that the center distorsion is already compensated
                double cx = parType.find("w").asDouble() / 2.0;
                double cy = parType.find("h").asDouble() / 2.0;
                double fx = parType.find("fx").asDouble();
                double fy = parType.find("fy").asDouble();

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
    inputImage = new ImageOf<PixelRgb>;
    outputImageMosaic = new ImageOf<PixelRgb>;
    robot = "icub";
    resized = false;
}

mosaicThread::mosaicThread(string _robot, string _configFile) {
    inputImage = new ImageOf<PixelRgb>;
    outputImageMosaic = new ImageOf<PixelRgb>;
    robot = _robot;
    configFile = _configFile;
    resized = false;
}

mosaicThread::~mosaicThread() {
    // do nothing   
}

bool mosaicThread::threadInit() {
    /* open ports */ 
    if (!imagePortIn.open(getName("/image:i").c_str())) {
        cout <<": unable to open port for camera  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    
    if (!imagePortOut.open(getName("/image:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    //initialising the head polydriver
    printf("starting the polydrive for the head.... of the robot %s \n", robot.c_str());
    Property optHead("(device remote_controlboard)");
    string remoteHeadName="/"+robot+"/head";
    string localHeadName="/"+name+"/head";
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
    optPolyTorso.put("remote",("/"+robot+"/torso").c_str());
    optPolyTorso.put("local",("/"+name+"/torso/position").c_str());

    polyTorso=new PolyDriver;
    if (!polyTorso->open(optPolyTorso))
    {
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
    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        printf("SUCCESS in finding configuaraion of camera param \n");
        Matrix &Prj=*PrjL;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }

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
    //mosaic image default size
    width = w;
    height = h;
    //default position of input image's center
    xcoord = width/2;
    ycoord = height/2;
}

void mosaicThread::setInputDim(int w, int h) {
    //input image default size
    width_orig = w;
    height_orig = h;
}

void mosaicThread::run() {
    
    outputImageMosaic->resize(this->width,this->height);
    outputImageMosaic->zero();
    while (isStopping() != true)  {                
        
        inputImage = imagePortIn.read(false); // do not wait                          
        if (inputImage != NULL ) {            
            if(!resized) {
                resize(inputImage->width(),inputImage->height());
                resized = true;
            }
            makeMosaic(inputImage); 
            imagePortOut.prepare() = *outputImageMosaic;                   
            imagePortOut.write();
        }                      
    }
}

void mosaicThread::resize(int width_orig,int height_orig) {        
    inputImage->resize(width_orig,height_orig);
    printf("resizing using %d %d",width_orig,height_orig);
    this->width_orig = width_orig;
    this->height_orig = height_orig;
    
}

bool mosaicThread::placeInpImage(int X, int Y) {
    if(X > this->width || X < 0 || Y > this->height || Y < 0) return false;
    this->xcoord = X;
    this->ycoord = Y;
    return true;
}

bool mosaicThread::setMosaicSize(int width=DEFAULT_WIDTH, int height=DEFAULT_HEIGHT) {
    if(width > MAX_WIDTH || width < width_orig || width <= 0 
       || height > MAX_HEIGHT || height < height_orig || height <= 0 ) return false;
    this->width = width;
    this->height = height;
    outputImageMosaic->resize(this->width,this->height);
    outputImageMosaic->zero();
    return true;    
}

void mosaicThread::makeMosaic(ImageOf<PixelRgb>* inputImage) {
    int i,j;
    unsigned char* inpTemp = inputImage->getRawImage();
    unsigned char* outTemp = outputImageMosaic->getRawImage();
    int iW = inputImage->width();
    int iH = inputImage->height();
    int mPad = outputImageMosaic->getPadding();
    int inputPadding = inputImage->getPadding();
    
    //recalculing the position in the space
    double u = 160;
    double v = 120;
    double z = 0.5;
    
    bool isLeft = true;
    Vector fp(3);
    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);
    //printf("getting angles \n");
    
    if (invPrj) {
        
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
        q[3]=head[0] * ratio;
        q[4]=head[1] * ratio;
        q[5]=head[2] * ratio;
        q[6]=head[3] * ratio;
        q[7]=head[4] * ratio;
        double ver = head[5];
        printf("0:%f 1:%f 2:%f 3:%f 4:%f 5:%f 6:%f 7:%f \n", q[0],q[1],q[2],q[3],q[4],q[5],q[6],q[7]);
               
                        
        Vector x(3);
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
                
        // find the 3D position from the 2D projection,
        // knowing the distance z from the camera
        Vector xe = yarp::math::operator *(*invPrj, x);
        xe[3]=1.0;  // impose homogeneous coordinates                
                
                        // update position wrt the root frame
        Matrix eyeH = eye->getH(q);
        //printf(" %f %f %f ", eyeH(0,0), eyeH(0,1), eyeH(0,2));
        Vector xo = yarp::math::operator *(eyeH,xe);
        
        fp.resize(3,0.0);
        fp[0]=xo[0];
        fp[1]=xo[1];
        fp[2]=xo[2];
        printf("object %f,%f,%f \n",fp[0],fp[1],fp[2]);
    }

    //calculating the shift in pixels
    double focalLenght = 0.1;
    double distance = z;
    double shift = fp[1] * ( focalLenght / distance );
    printf("shift %f \n", shift);


    /*
    printf("iH %d, iW %d height %d width %d \n", iH, iW, height, width );
    for(i = 0 ; i < iH ; ++i) {
        for(j = 0 ; j < iW ; ++j) {   
            int mosaicX = i; 
            mosaicX -= iH / 2;
            mosaicX += ycoord; 
            int mosaicY = j;
            mosaicY -= iW / 2;
            mosaicY += xcoord;

           
            
            if(mosaicX < height && mosaicY < width) {
                int index = mosaicX;
                index = index *(width * 3 + mPad);
                index += 3 * mosaicY;
                 
                *(outTemp + index) += *inpTemp;
                inpTemp++;
                
                *(outTemp + index + 1) += *inpTemp;
                inpTemp++;
                
                *(outTemp + index + 2) += *inpTemp;
                inpTemp++;
            }
            else {
                inpTemp +=3;
            }
            
        }
        //inpTemp += inputPadding;
    }

    */
    
}


void mosaicThread::threadRelease() {
    resized = false;     
}

void mosaicThread::onStop() {
    imagePortIn.interrupt();
    imagePortOut.interrupt();
        
    imagePortOut.close();
    imagePortIn.close();
}

