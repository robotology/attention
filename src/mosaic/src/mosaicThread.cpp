// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak, Francesco Rea
 * email:   shashank.pathak@iit.it, francesco.rea@iit.it
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
    inputImage = new ImageOf<PixelRgb>;
    outputImageMosaic = new ImageOf<PixelRgb>;
    robot = "icub"; 
    resized = false;
    memory = (float*) malloc(MAXMEMORY);
    memset(memory, 0, MAXMEMORY);
    countMemory = 0;
    azimuth = 0.0;
    elevation = 0.0;
}

mosaicThread::mosaicThread(string _robot, string _configFile) {
    inputImage = new ImageOf<PixelRgb>;
    outputImageMosaic = new ImageOf<PixelRgb>;
    robot = _robot;
    configFile = _configFile;
    resized = false;
    memory = (float*) malloc(MAXMEMORY);
    memset(memory, 0, MAXMEMORY);
    countMemory = 0;
    elevation = 0.0;
    azimuth = 0.0;
}

mosaicThread::~mosaicThread() {
    // do nothing   
    delete inputImage;
    delete outputImageMosaic;
    free(memory);   
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
    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        printf("SUCCESS in finding configuration of camera param \n");
        Matrix &Prj=*PrjL;
        cxl = Prj(0,2);
        cyl = Prj(1,2);
        fxl = Prj(0,0);
        fyl = Prj(1,1);
                  
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
    }
    else { 
        printf("did not find the configuration file for the camera \n");
        return false; //PrjL=invPrjL=NULL;
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
    *eyeH0 = eyeL->getH(q);
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
    printf("resizing using %d %d \n",width_orig,height_orig);
    inputImage->resize(width_orig,height_orig);
    this->width_orig = width_orig;
    this->height_orig = height_orig;
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
    outputImageMosaic->resize(this->width,this->height);
    outputImageMosaic->zero();
    return true;    
}

void mosaicThread::run() {
    printf("Initialization of the run function %d %d .... \n", width, height);
    count++;
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


void mosaicThread::makeMosaic(ImageOf<PixelRgb>* inputImage) {
    printf("making the mosaic \n");
    //recalculing the position in the space
    double u = 160;
    double v = 120;
    double z = 1.0;
    
    bool isLeft = true;
    Vector fp(3);
    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
    iCubEye *eye=(isLeft?eyeL:eyeR);
    Vector prec;
    double shift_prev;

    CvPoint2D32f *c1 = new CvPoint2D32f[4];
    CvPoint2D32f *c2 = new CvPoint2D32f[4];
    
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
        Vector xe = yarp::math::operator *(*invPrj, x);
        xe[3]=1.0;  // impose homogeneous coordinates                
                
        // update position wrt the root frame
        eyeHL = new Matrix(4,4);
        eyeHR = new Matrix(4,4);
        *eyeHL = eyeL->getH(q);
        *eyeHR = eyeR->getH(q);

        //printf(" %f %f %f ", eyeH(0,0), eyeH(0,1), eyeH(0,2));
        Vector xo = yarp::math::operator *(*eyeHL,xe);
        
        prec = fp;
        fp.resize(3,0.0);
        fp[0]=xo[0];
        fp[1]=xo[1];
        fp[2]=xo[2];
        //printf("object %f,%f,%f \n",fp[0],fp[1],fp[2]);
        
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
        xe = yarp::math::operator *(*invPrj, x);
        xe[3]=1.0;  // impose homogeneous coordinates
        xo = yarp::math::operator *(*eyeHL,xe);
        //------------------------------------------
        xe = yarp::math::operator *(*inveyeH0,xo);
        x_hat = yarp::math::operator *(*cyclopicPrj, xe);
        c1[0].x = x_hat[0]/z;   c1[0].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0], x_hat[1], x_hat[2]);

        //___________________________________________________________
        
        u = 320;
        v = 0;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        xe = yarp::math::operator *(*invPrj, x);
        xe[3]=1.0;  // impose homogeneous coordinates
        xo = yarp::math::operator *(*eyeHL,xe);
        //printf("vertix %f,%f,%f \n",xo[0],xo[1],xo[2]);
        //------------------------------------------
        xe = yarp::math::operator *(*inveyeH0,xo);
        x_hat = yarp::math::operator *(*cyclopicPrj, xe);
        c1[1].x = x_hat[0]/z;   c1[1].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0], x_hat[1], x_hat[2]);
        //__________________________________________________________

        u = 0;
        v = 240;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        xe = yarp::math::operator *(*invPrj, x);
        xe[3]=1.0;  // impose homogeneous coordinates
        xo = yarp::math::operator *(*eyeHL,xe);
        //printf("vertix %f,%f,%f \n",xo[0],xo[1],xo[2]);
        //------------------------------------------
        xe = yarp::math::operator *(*inveyeH0,xo);
        x_hat = yarp::math::operator *(*cyclopicPrj, xe);
        c1[2].x = x_hat[0]/z;   c1[2].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0], x_hat[1], x_hat[2]);
        //________________________________________________________

        u = 320;
        v = 240;
        x[0]=z * u;   //epipolar correction excluded the focal lenght
        x[1]=z * v;
        x[2]=z;
        xe = yarp::math::operator *(*invPrj, x);
        xe[3]=1.0;  // impose homogeneous coordinates
        xo = yarp::math::operator *(*eyeHL,xe);
        //printf("vertix %f,%f,%f \n",xo[0],xo[1],xo[2]);
        //------------------------------------
        xe = yarp::math::operator *(*inveyeH0,xo);
        x_hat = yarp::math::operator *(*cyclopicPrj, xe);
        c1[3].x = x_hat[0]/z;   c1[3].y = x_hat[1]/z;
        printf("onPlane %f %f %f \n \n",x_hat[0], x_hat[1], x_hat[2]);
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
    
    shift_prev = shiftx;
    shiftx = fxl  * ((x[0]*3.14)/180);
    shifty = -fyl  * ((x[1]*3.14)/180);
    
    ycoord = shiftx + floor(width / 2);
    xcoord = shifty + floor(height / 2);
    printf("ycoord %d shiftx %f shifty %f shift   \n",ycoord,shiftx,shifty);
    
    // making the mosaic
    int i,j;
    
    unsigned char* outTemp = outputImageMosaic->getRawImage();
    unsigned char* lineOutTemp;
    int iW = inputImage->width();
    int iH = inputImage->height();
    int mPad = outputImageMosaic->getPadding();
    int inputPadding = inputImage->getPadding();
    int rowSize = outputImageMosaic->getRowSize();   
    

    //warping the image
    CvPoint2D32f srcTri[3], dstTri[3];
    CvMat* warp_mat = cvCreateMat(2,3,CV_32F);
    CvSize dsize = {320,240};
    // ---------- 

    CvMat* mmat = cvCreateMat(3,3,CV_32FC1);
    
    /*
    CvPoint2D32f* c1 = (&cvPoint2D32f(0,0), &cvPoint2D32f(320,0), &cvPoint2D32f(0,240), &cvPoint2D32f(320,240));
    CvPoint2D32f* c2 = (&cvPoint2D32f(0,0), &cvPoint2D32f(320,0), &cvPoint2D32f(0,240), &cvPoint2D32f(320,240)); 
    */    

    CvMat* src = cvCreateMat(4,4,CV_32FC1); //vector of 4 points
    CvMat* dest = cvCreateMat(3,3,CV_32FC3); //vector of 4 points
    CvMat* mlx = cvCreateMat(4,4,CV_32FC1);
    
    //int step = src->step/sizeof(float);
    //float* value = src->data.fl;
    //value[0] = 0.0;  value[1] = 0.0 ;  value[2] = 0.0; value[3] = 1.0; 
    //value[4] = 320.0; value[5] = 0.0;  value[6] = 0.0; value[7] = 1.0; 
    //value[8] = 0.0;  value[9] = 240.0;  value[10] = 0.0; value[11] = 1.0; 
    //value[12] = 320.0;  value[13] = 240.0; value[14] = 0.0; value[15] = 1.0;
 
    //float* pointerMlx = mlx->data.fl;
    //double* pointerEyeH = eyeH->data();  
    //for (int i = 0; i < 4 ; i++) {
    //    for (int j = 0;j < 4; j++) {
    //        pointerMlx[i*4+j] = (float) *pointerEyeH;
    //        printf("%f,  ", pointerMlx[i*4+j]);
    //        //pointerMlx++;
    //        pointerEyeH++;            
    //    }
    //    printf("\n");
    // }
 
    //cvPerspectiveTransform(c3, c4, dest);
    //c4[0].x = 0.0;    c4[0].y = 50.0;   c4[0].z = 0.0;
    //c4[1].x = 320.0;  c4[1].y = 0.0;   c4[1].z = 0.0;
    //c4[2].x = 0.0;    c4[2].y = 240.0-50; c4[2].z = 0.0;
    //c4[3].x = 320.0;  c4[3].y = 240.0; c4[3].z = 0.0;
    
    //printf("success in getting the perspective \n");
    //float* pointerDest = dest->data.fl;
    //for (int i = 0; i < 4 ; i++) {
    //    c1[i].x = c4[i].x;
    //    c1[i].y = c4[i].y;
    //    printf("position %f %f \n",c4[i].x,c4[i].y);
    //}    
    
    printf("getting perspective transform \n");   
    mmat = cvGetPerspectiveTransform(c1, c2, mmat);

    CvSize size={320,240};
    ImageOf<PixelRgb>* destIm = new ImageOf<PixelRgb>;
    destIm->resize(320,240);
	
    cvWarpPerspective((CvArr*)inputImage->getIplImage(),(CvArr*)destIm->getIplImage(),mmat,CV_INTER_LINEAR+CV_WARP_FILL_OUTLIERS); 
    //destIm = inputImage;
    unsigned char* inpTemp = destIm->getRawImage();

    int mosaicX, mosaicY;
    mosaicX = ycoord;
    mosaicX -= floor(iH / 2);
    mosaicY = xcoord;
    mosaicY -= floor(iW / 2);
    float alfa = 0.9;
    printf("rowSize %d mosaicX %d mosaicY %d ycoord %d xcoord  %d \n", rowSize, mosaicX, mosaicY,ycoord, xcoord);
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
        inpTemp += inputPadding;
        outTemp = lineOutTemp = lineOutTemp + (rowSize + mPad);
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

