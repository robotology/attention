// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
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
 * @file blobFinderThread.cpp
 * @brief module class implementation of the blob finder thread (this is the module actual processing engine).
 */

#include <cstring>
#include <iostream>

#include <iCub/blobFinderThread.h>
#include <yarp/math/SVD.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;
using namespace yarp::sig;
using namespace yarp::math;
using namespace iCub::iKin;

const int DEFAULT_THREAD_RATE = 100;
#define thresholdDB 250
#define MAXMEMORY 100

#define OPPONENCYBYCONVOLUTION

// defining seperate vectors (horizontal and vertical) for gaussians of size 5x5 (sigma 1) and 7x7 (sigma 3)
// method to do so could be: 1. [u s v] = svd(G) 2. Normalize columns of u and v, by sqrt of largest singular
// value ie s(1,1)
static float G7[7] = {     -0.1063f,
                           -0.1403f,
                           -0.1658f,
                           -0.1752f,
                           -0.1658f,
                           -0.1403f,
                           -0.1063f
                    };

static float G5[5] = { -0.0545f,
                       -0.2442f,
                       -0.4026f,
                       -0.2442f,
                       -0.0545f
                     };


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

/**********************************************************************************/

blobFinderThread::blobFinderThread(int rateThread = DEFAULT_THREAD_RATE, string _configFile = "") : RateThread(rateThread){
    saddleThreshold = 10;
    reinit_flag = false;
    resized_flag = false;
    configFile = _configFile;

    targetRed   = 255;
    targetGreen = 0;
    targetBlue  = 0;

    weightBU = 1.0;
    weightTD = 0.0;

    outContrastLP   = new ImageOf<PixelMono>;
    outMeanColourLP = new ImageOf<PixelBgr>;

    _procImage    = new ImageOf<PixelRgb>;
    _outputImage3 = new ImageOf<PixelRgb>;

    ptr_inputImg       = new ImageOf<yarp::sig::PixelRgb>;
    ptr_foveaRgbImg    = new ImageOf<yarp::sig::PixelRgb>;
    ptr_foveaImg       = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgRed    = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgGreen  = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgBlue   = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgYellow = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgRG     = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgGR     = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgBY     = new ImageOf<yarp::sig::PixelMono>;

    ptr_tmpRplus   = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpRminus  = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpGplus   = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpGminus  = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpBplus   = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpYminus  = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpRpluss  = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpRminuss = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpGpluss  = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpGminuss = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpBpluss  = new ImageOf<yarp::sig::PixelMono>; 
    ptr_tmpYminuss = new ImageOf<yarp::sig::PixelMono>; 
 
    edges     = new ImageOf<PixelMono>; 
    img       = new ImageOf<PixelRgb>;
    tmpImage  = new ImageOf<PixelMono>;
    image_out = new ImageOf<PixelMono>;
    
    
    _inputImgRGS = new ImageOf<PixelMono>;
    _inputImgGRS = new ImageOf<PixelMono>;
    _inputImgBYS = new ImageOf<PixelMono>;

    memory = new char[3 * MAXMEMORY ];
    memset(memory,0, 3 * MAXMEMORY);

    // some standard parameters on the blob search.
    maxBLOB = 4096;
    minBLOB = 300;
    minBoundingArea = 225;
}

blobFinderThread::~blobFinderThread() {
    delete outContrastLP;
    delete outMeanColourLP;
    delete _procImage;
    delete _outputImage3;
    delete ptr_inputImg;        // pointer to the input image
    delete edges;               // pointer to the edges image

    delete ptr_foveaImg;        
    delete ptr_foveaRgbImg;
    delete ptr_inputImgRed;     // pointer to the input image of the red plane
    delete ptr_inputImgGreen;   // pointer to the input image of the green plane
    delete ptr_inputImgBlue;    // pointer to the input image of the blue plane
    delete ptr_inputImgYellow;  // pointer to the input image of the blue plane
    delete ptr_inputImgRG;      // pointer to the input image of the R+G- colour opponency
    delete ptr_inputImgGR;      // pointer to the input image of the G+R- colour opponency
    delete ptr_inputImgBY;      // pointer to the input image of the B+Y- colour opponency
 
    delete ptr_tmpRplus;
    delete ptr_tmpRminus;
    delete ptr_tmpGplus;
    delete ptr_tmpGminus;
    delete ptr_tmpBplus;
    delete ptr_tmpYminus;
    delete ptr_tmpRpluss;
    delete ptr_tmpRminuss;
    delete ptr_tmpGpluss;
    delete ptr_tmpGminuss;
    delete ptr_tmpBpluss;
    delete ptr_tmpYminuss;
    
    delete _inputImgRGS;
    delete _inputImgGRS;
    delete _inputImgBYS;

    delete img;
    delete tmpImage;
    delete image_out;
    delete wOperator;
    delete salience;
    delete ptr_tagged;

    delete[] memory;
}

void blobFinderThread::setName(std::string str) {
    this->name=str; 
}

void blobFinderThread::setRobotName(std::string str) {
    this->robot=str; 
}

std::string blobFinderThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void blobFinderThread::reinitialise(int width, int height) {
    this->width = width;
    this->height = height;

    resizeImages(width, height);

    reinit_flag = true;
}

void blobFinderThread::resizeImages(int width, int height) {
    //srcsize.height = height;
    //srcsize.width = width;

    const int widthstep = int(ceil(width/32.0)*32);

    ptr_tagged = new yarp::sig::ImageOf<yarp::sig::PixelInt>;
    ptr_tagged->resize(widthstep, height);
    
    this->wOperator=new WatershedOperator(true, width, height, widthstep, saddleThreshold);
    this->salience=new SalienceOperator(width, height);

    outMeanColourLP->resize(width, height);
    outContrastLP->resize(width, height);

    _procImage->resize(width, height);
    _outputImage3->resize(width, height);
    _inputImgRGS->resize(width, height);
    _inputImgGRS->resize(width, height);
    _inputImgBYS->resize(width, height);
    
    img->resize(width, height);
    edges->resize(width, height);
    tmpImage->resize(width, height);
    image_out->resize(width, height);

    blobList = new char [width*height+1];
    
    ptr_inputImg      ->resize(width, height);
    ptr_foveaRgbImg   ->resize(width, height);
    ptr_foveaImg      ->resize(width, height);
    ptr_inputImgRed   ->resize(width, height);
    ptr_inputImgGreen ->resize(width, height);
    ptr_inputImgBlue  ->resize(width, height);
    ptr_inputImgYellow->resize(width, height);
    ptr_inputImgRG    ->resize(width, height);
    ptr_inputImgGR    ->resize(width, height);
    ptr_inputImgBY    ->resize(width, height);

    ptr_tmpRplus->resize(width,height);
    ptr_tmpRminus->resize(width,height);
    ptr_tmpGplus->resize(width,height);
    ptr_tmpGminus->resize(width,height);
    ptr_tmpBplus->resize(width,height);
    ptr_tmpYminus->resize(width,height);
    ptr_tmpRpluss->resize(width,height);
    ptr_tmpRminuss->resize(width,height);
    ptr_tmpGpluss->resize(width,height);
    ptr_tmpGminuss->resize(width,height);
    ptr_tmpBpluss->resize(width,height);
    ptr_tmpYminuss->resize(width,height);
    
    resized_flag = true;
}

/**
 * initialization of the thread 
 */
bool blobFinderThread::threadInit() {
    if (!inputPort.open(getName("/image:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!edgesPort.open(getName("/edges:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!blobDatabasePort.open(getName("/database").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!saliencePort.open(getName("/salienceMap:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputPort3.open(getName("/imageC3:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!rgPort.open(getName("/rg:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!grPort.open(getName("/gr:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!byPort.open(getName("/by:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!rgOut.open(getName("/rg:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!grOut.open(getName("/gr:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!byOut.open(getName("/by:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!foveaPort.open(getName("/foveaBlob:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!foveaRgbPort.open(getName("/foveaRgbBlob:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    //initializing gazecontrollerclient
    Property option;
    option.put("device","gazecontrollerclient");
    option.put("remote","/iKinGazeCtrl");
    string localCon("/client/gaze");
    localCon.append(getName(""));
    option.put("local",localCon.c_str());

    printf("starting the interface with the iKinGazeCtrl \n");
    clientGazeCtrl=new PolyDriver();
    clientGazeCtrl->open(option);
    igaze=NULL;

    if (clientGazeCtrl->isValid()) {
       clientGazeCtrl->view(igaze);
    }
    else
        return false;


    string robot("icub"); //TODO: hard coded here remove asap

    //initialising the head polydriver
    printf("starting the polydrive for the head.... \n");
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


    //extracting the kinematic chain of the two eyes
    eyeL=new iCubEye("left");
    eyeR=new iCubEye("right");

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

    printf("trying to CAMERA projection from %s.......... ", configFile.c_str());
    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
        printf("SUCCESS in finding configuaraion of camera param \n");
        Matrix &Prj=*PrjL;
        //cxl=Prj(0,2);
        //cyl=Prj(1,2);
        invPrjL=new Matrix(pinv(Prj.transposed()).transposed());
        printf("found the matrix of projection of left %f %f %f", Prj(0,0),Prj(1,1),Prj(2,2));
    }

    return true;
}

/**
* function called when the module is poked with an interrupt command
*/
void blobFinderThread::interrupt() {
    inputPort.interrupt();          // getName("image:i");
    edgesPort.interrupt();          // getName(edges:i);
    blobDatabasePort.interrupt();
    saliencePort.interrupt();
    outputPort3.interrupt();
    
    rgPort      .interrupt();
    grPort      .interrupt();
    byPort      .interrupt();
    rgOut       .interrupt();
    grOut       .interrupt();
    byOut       .interrupt();
    foveaPort   .interrupt();
    foveaRgbPort.interrupt();
}

/**
 *	releases the thread
 */
void blobFinderThread::threadRelease() {
    blobDatabasePort.close();
    outputPort3.close();
    saliencePort.close();
    inputPort.close();
    
    rgPort       .close();
    grPort       .close();
    byPort       .close();
    rgOut        .close();
    grOut        .close();
    byOut        .close();
    foveaPort    .close();
    foveaRgbPort .close();
}


void blobFinderThread::getFoveaRgb(int& redValue, int& greenValue, int& blueValue) {
    salience->getFoveaBlobColor(redValue, greenValue, blueValue);
}

/**
 * active loop of the thread
 */
void blobFinderThread::run() {
    img = inputPort.read(false);
    if (0 != img) {
        if (!reinit_flag) {
            //srcsize.height = img->height();
            //srcsize.width  = img->width();
            height = img->height();
            width  = img->width();
            reinitialise(img->width(), img->height());
        }

        //printf("original size %d %d \n", height, width);
        memcpy(ptr_inputImg->getRawImage(),img->getRawImage(),img->getRowSize()* img->height());
        
        bool ret1=true, ret2=true;
        ret2 = getPlanes(img);
        ret1 = getOpponencies();
        if (!ret1 || !ret2)
            return;

        tmpImage = edgesPort.read(false);
        if (tmpImage != 0) {
            memcpy(edges->getRawImage(),tmpImage->getRawImage(),tmpImage->getRowSize()* tmpImage->height());
        }

        rain(edges);
        drawAllBlobs(false);

        //salience->DrawMaxSaliencyBlob(*salience->maxSalienceBlob_img, max_tag, *tagged);
        //copy_8u_C1R(salience->maxSalienceBlob_img->getRawImage(), salience->maxSalienceBlob_img->getRowSize(), _outputImage->getRawImage(), _outputImage->getRowSize(), srcsize);

        salience->ComputeMeanColors(max_tag);
        salience->DrawMeanColorsLP(*outMeanColourLP, *ptr_tagged);

        //ippiCopy_8u_C3R(outMeanColourLP->getRawImage(), outMeanColourLP->getRowSize(), _outputImage3->getRawImage(), _outputImage3->getRowSize(), srcsize);	
        memcpy(_outputImage3->getRawImage(),outMeanColourLP->getRawImage(), outMeanColourLP->getRowSize() *  outMeanColourLP->height());
        
        /*   
        if((0 != _outputImage3) && (outputPort3.getOutputCount())) { 
            outputPort3.prepare() = *(_outputImage3);
            outputPort3.write();
        }
        */

        if((0 != ptr_foveaImg) ||  (foveaPort.getOutputCount())) { 
            foveaPort.prepare() = *((ImageOf<PixelMono>*)ptr_foveaImg);
            foveaPort.write();
        }
        
        if((0 != ptr_foveaRgbImg) ||  (foveaRgbPort.getOutputCount())) { 
            foveaRgbPort.prepare() = *((ImageOf<PixelRgb>*)ptr_foveaRgbImg);
            foveaRgbPort.write();
        }

        if((0 != _outputImage3) ||  (outputPort3.getOutputCount())) { 
            outputPort3.prepare() = *((ImageOf<PixelRgb>*)_outputImage3);
            outputPort3.write();
        }

        if((0 != outContrastLP) && (saliencePort.getOutputCount())) { 
            saliencePort.prepare() = *((ImageOf<PixelMono>*)outContrastLP);
            saliencePort.write();
        }

        if(blobDatabasePort.getOutputCount()) {
            
            
            Vector fp;
            YARPBox* pBlob = salience->getBlobList();
            bool isLeft = true;            
            int u, v; //coordinate in the image plane
            double z; //distance from the eye


            u = 160; //pBlob[i].centroid_x;
            v = 120; //pBlob[i].centroid_y;
            z = 0.5;
            
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
            
            
            for (int i = 1; i < nBlobs; i++) {
                if ((pBlob[i].valid)&&(pBlob[i].areaLP > thresholdDB)) {
                    printf("areaLP: %d \n", pBlob[i].areaLP);
                    
                    /*
                        char* pointer = memory;
                        if ((memoryPos != 0)&&(memoryPos < MAXMEMORY)) {
                            //checking the distance with the previously memorised 3D locations
                            int j;
                            for (j = 0; j < memoryPos; j++) {
                                int x = *pointer++; 
                                int y = *pointer++;
                                int z = *pointer++;
                                
                                double distance = sqrt((fp[0] - x) * (fp[0] - x) + (fp[1] - y) * (fp[1] - y) + (fp[2] - z) * (fp[2] - z));
                                printf("distance %f \n", distance);
                                if( distance < 0.6) {
                                    printf("already saved position \n");
                                    break;
                                }
                            }
                            if( j >= memoryPos) {
                                //need to add a new position
                                pointer++;
                                *pointer = fp[0];
                                pointer++;
                                *pointer = fp[1];
                                pointer++;
                                *pointer = fp[2];
                                memoryPos++;
                                
                                //adding novel position to the GUI
                                Bottle request, reply;
                                request.clear(); reply.clear();
                                request.addVocab(VOCAB3('a','d','d'));
                                Bottle& listAttr=request.addList();
                                
                                Bottle& sublistX = listAttr.addList();
                                
                                sublistX.addString("x");
                                sublistX.addDouble(fp[0] * 1000);    
                                listAttr.append(sublistX);
                                
                                Bottle& sublistY = listAttr.addList();
                                sublistY.addString("y");
                                sublistY.addDouble(fp[1] * 1000);      
                                listAttr.append(sublistY);
                                
                                Bottle& sublistZ = listAttr.addList();            
                                sublistZ.addString("z");
                                sublistZ.addDouble(fp[2] * 1000);   
                                listAttr.append(sublistZ);
                                
                                Bottle& sublistR = listAttr.addList();
                                sublistR.addString("r");
                                sublistR.addDouble(255.0);
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
                                sublistLife.addDouble(60.0);
                                listAttr.append(sublistLife);          
                   
                                
                                blobDatabasePort.write(request, reply);
                            }
                        }
                        else {  //case where the element is either the first of the list or the last of the list
                            //memoryPos = 1; 
                            pointer = memory;
                            *pointer = fp[0]; pointer++;
                            *pointer = fp[1]; pointer++;
                            *pointer = fp[2];
                            
                            Bottle request, reply;
                            request.clear(); reply.clear();
                            request.addVocab(VOCAB3('a','d','d'));
                            Bottle& listAttr=request.addList();
                            
                            Bottle& sublistX = listAttr.addList();
                            
                            sublistX.addString("x");
                            sublistX.addDouble(fp[0] * 1000);      
                            listAttr.append(sublistX);
                            
                            Bottle& sublistY = listAttr.addList();
                            sublistY.addString("y");
                            sublistY.addDouble(fp[1] * 1000);      
                            listAttr.append(sublistY);
                            
                            Bottle& sublistZ = listAttr.addList();            
                            sublistZ.addString("z");
                            sublistZ.addDouble(fp[2] * 1000);   
                            listAttr.append(sublistZ);
                            
                            Bottle& sublistR = listAttr.addList();
                            sublistR.addString("r");
                            sublistR.addDouble(255.0);
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
                            sublistLife.addDouble(60.0);
                            listAttr.append(sublistLife);          
                            
                            
                            blobDatabasePort.write(request, reply);
                            memoryPos++;
                        }
                    }
                    */
                } //if ((pBlob[i].valid)&&(pBlob[i].areaLP > thresholdDB))
            } //for (int i = 1; i < nBlobs; i++)
        } //if(blobDatabasePort.getOutputCount())
    } // if (0 != img)
}



/**
 * function that reads the ports for colour RGB opponency maps
 */
bool blobFinderThread::getOpponencies() {

    unsigned char* pRG = ptr_inputImgRG->getRawImage();
    unsigned char* pGR = ptr_inputImgGR->getRawImage();
    unsigned char* pBY = ptr_inputImgBY->getRawImage();

#ifdef OPPONENCYBYCONVOLUTION
    // We have got planes by now, so we need to convolve them and then get R+G- and others
    //printf("Convolving for opponency maps \n");
    //Positive
    convolve1D(5,G5,ptr_inputImgRed,ptr_tmpRpluss,.5,0);
    convolve1D(5,G5,ptr_tmpRpluss,ptr_tmpRplus,.5,1);

    convolve1D(5,G5,ptr_inputImgGreen,ptr_tmpGpluss,.5,0);
    convolve1D(5,G5,ptr_tmpGpluss,ptr_tmpGplus,.5,1);

    convolve1D(5,G5,ptr_inputImgBlue,ptr_tmpBpluss,.5,0);
    convolve1D(5,G5,ptr_tmpBpluss,ptr_tmpBplus,.5,1);

    //Negative
    convolve1D(7,G7,ptr_inputImgRed,ptr_tmpRminuss,.5,0);
    convolve1D(7,G7,ptr_tmpRminuss,ptr_tmpRminus,.5,1);
    
    convolve1D(7,G7,ptr_inputImgGreen,ptr_tmpGminuss,.5,0);
    convolve1D(7,G7,ptr_tmpGminuss,ptr_tmpGminus,.5,1);
    
    convolve1D(7,G7,ptr_inputImgYellow,ptr_tmpYminuss,.5,0);
    convolve1D(7,G7,ptr_tmpYminuss,ptr_tmpYminus,.5,1);

    // Finding color opponency now

    unsigned char* rPlus  = ptr_tmpRplus->getRawImage();
    unsigned char* rMinus = ptr_tmpRminus->getRawImage();
    unsigned char* gPlus  = ptr_tmpGplus->getRawImage();
    unsigned char* gMinus = ptr_tmpGminus->getRawImage();
    unsigned char* bPlus  = ptr_tmpBplus->getRawImage();
    unsigned char* yMinus = ptr_tmpYminus->getRawImage();

    int h   = ptr_tmpRplus->height();
    int w   = ptr_tmpRplus->width(); //ASSUME: all dimensions are similar
    int pad = ptr_tmpRplus->getPadding();

    for(int r = 0; r < h; r++) {
        for(int c = 0; c < w; c++) {
            
            *pRG++ = ((*rPlus >> 1) + 128 - (*gMinus >> 1) );
            *pGR++ = ((*gPlus >> 1) + 128 - (*rMinus >> 1) );
            *pBY++ = ((*bPlus >> 1) + 128 - (*yMinus >> 1) );

            rMinus++;
            rPlus++;
            gMinus++;
            gPlus++;
            yMinus++;
            bPlus++;
        }

        rMinus += pad;
        rPlus  += pad;
        gMinus += pad;
        gPlus  += pad;
        yMinus += pad;
        bPlus  += pad;
        pRG += pad;
        pGR += pad;
        pBY += pad;

    }

    if((0 != ptr_inputImgRG ) &&  (rgOut.getOutputCount())) { 
        rgOut.prepare() = *((ImageOf<PixelMono>*)ptr_inputImgRG);
        rgOut.write();
    }
    if((0 != ptr_inputImgGR ) &&  (grOut.getOutputCount())) { 
        grOut.prepare() = *((ImageOf<PixelMono>*)ptr_inputImgGR);
        grOut.write();
    }
    if((0 != ptr_inputImgBY ) &&  (rgOut.getOutputCount())) { 
        byOut.prepare() = *((ImageOf<PixelMono>*)ptr_inputImgBY);
        byOut.write();
    }
    
#else
    tmpImage = rgPort.read(false);
    if (tmpImage != 0) {        
        memcpy(pRG ,tmpImage->getRawImage(),tmpImage->getRowSize()* tmpImage->height());
        //printf("copying the image %d %d \n", tmpImage->getRowSize(),tmpImage->height() );
        if((0 != ptr_inputImgRG ) &&  (rgOut.getOutputCount())) { 
            rgOut.prepare() = *((ImageOf<PixelMono>*)ptr_inputImgRG);
            rgOut.write();
        }
        //printf("Success in preparing \n");
    }
    tmpImage = grPort.read(false);
    if (tmpImage != 0) {
        memcpy(pGR ,tmpImage->getRawImage(),tmpImage->getRowSize()* tmpImage->height());
    }
    tmpImage = byPort.read(false);
    if (tmpImage != 0) {
        memcpy(pBY ,tmpImage->getRawImage(),tmpImage->getRowSize()* tmpImage->height());
    }
    
#endif
    /*

    
    if((0 != ptr_inputImgGR) &&  (grOut.getOutputCount())) { 
        grOut.prepare() = *((ImageOf<PixelMono>*)ptr_inputImgGR);
        grOut.write();
    }
    if((0 != ptr_inputImgBY) &&  (byOut.getOutputCount())) { 
        byOut.prepare() = *((ImageOf<PixelMono>*)ptr_inputImgBY);
        byOut.write();
    }
    */
    

    return true;

}

/**
 * function that reads the ports for the RGB planes
 */
bool blobFinderThread::getPlanes(ImageOf<PixelRgb>* inputImage) {
    //Ipp8u* shift[3];
    unsigned char* shift[4];

    // assuming image planes are sized appropriately
    shift[0] = ptr_inputImgRed->getRawImage(); 
    shift[1] = ptr_inputImgGreen->getRawImage();
    shift[2] = ptr_inputImgBlue->getRawImage();
    shift[3] = ptr_inputImgYellow->getRawImage();

    int pad_inptImg = inputImage->getPadding();
    int pad_redPlane = ptr_inputImgRed->getPadding();
    int pad_greenPlane = ptr_inputImgGreen->getPadding();
    int pad_bluePlane = ptr_inputImgBlue->getPadding();
    int pad_yellowPlane = ptr_inputImgYellow->getPadding();
    unsigned char* ptr_inputImage = inputImage->getRawImage();

    for(int height=0; height<inputImage->height(); ++height){
        for(int width=0; width<inputImage->width(); ++width){
            *shift[0] = *ptr_inputImage;ptr_inputImage++;
            *shift[1] = *ptr_inputImage;ptr_inputImage++;
            *shift[2] = *ptr_inputImage;ptr_inputImage++;
            *shift[3] = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));
            shift[0]++;
            shift[1]++;
            shift[2]++;
            shift[3]++;
            
        }
        shift[0] += pad_redPlane;
        shift[1] += pad_greenPlane;
        shift[2] += pad_bluePlane;
        shift[3] += pad_yellowPlane;
        ptr_inputImage += pad_inptImg;

    }
            
    //ippiCopy_8u_C3P3R(inputImage->getRawImage(), inputImage->getRowSize(), shift, ptr_inputImgRed->getRowSize(), srcsize);
    //memcpy(inputImage->getRawImage(),ptr_inputImgRed->getRawImage(),inputImage->getRowSize()* inputImage->height());
    return true;
}

/**
 * applies the watershed (rain falling) algorithm
 */
void blobFinderThread::rain(ImageOf<PixelMono>* edgesImage) {
    max_tag = wOperator->apply(*edgesImage, *ptr_tagged);
}

void blobFinderThread::drawAllBlobs(bool stable)
{
    // initialization of all blobs (bounding boxes of the blobs). Store blob list internally (to SalienceOperator class).
    // starts from a tagged image and builds a list of boxes.
    salience->blobCatalog(*ptr_tagged, *ptr_inputImgRG, *ptr_inputImgGR, *ptr_inputImgBY,
                          *ptr_inputImgBlue, *ptr_inputImgGreen, *ptr_inputImgRed, max_tag);

    // computes the salience of each blob (fills the rg, gr, by average values into the boxes). 
    salience->ComputeSalienceAll(max_tag, max_tag);

    // extracts the PixelBgr color as color opponency map of tag=1. Assuming this is the fovea!
    //PixelBgr varFoveaBlob = salience->varBlob(*ptr_tagged, *ptr_inputImgRG, *ptr_inputImgGR, *ptr_inputImgBY, 1 /* (*ptr_tagged)(0,0) */);

    // draw the fovea blob into the blobFov image? Also assuming the tag=1.
    if(foveaPort.getOutputCount()) {
        salience->drawFoveaBlob(*ptr_foveaImg, *ptr_tagged);
    }
    if(foveaRgbPort.getOutputCount()) {
        salience->drawColorFoveaBlob(*ptr_foveaRgbImg, *ptr_tagged);
    }
    
    //memset(blobList, 0, sizeof(char)*(max_tag+1));

    // finds all pixels that have != tags from the (0,0) which is the fovea but are connected according
    // to a certain geometric neighborhood relation. Returns a list of blobtags of the connected pixels.
    //wOperator->findNeighborhood(*ptr_tagged, 0, 0, blobList);

    // it uses the varFoveaBlob (color) to compare similar pixels and the list of blobs searched earlier.
    // sets all pixels that are fovea-like to tag=1.
    //salience->fuseFoveaBlob3(*ptr_tagged, blobList, varFoveaBlob, max_tag);

    // Comment the following line to disable the elimination of invalid blob
    // salience->RemoveNonValidNoRange(max_tag, BLOB_MAXSIZE, BLOB_MINSIZE);
    salience->RemoveNonValidNoRange(max_tag, maxBLOB, minBLOB);

    //targetRG = 0;
    //targetGR = 0;
    //targetBY = 0;
    targetRG = (targetRed/2 - targetGreen/2) + 127;
    targetGR = (targetGreen/2 - targetRed/2) + 127;
    PixelMono targetYellow = (targetGreen + targetRed)>>1;
    //printf("targetRed %d targetGreen %d targetBlue %d \n", targetRed, targetGreen, targetBlue);
    targetBY = (targetBlue/2 - targetYellow/2) + 127;
    //printf("targetRG %d targetGR %d targetBY %d \n",targetRG, targetGR, targetBY );
    
    // draws the saliency of all blobs into the tagged image and returns a mono image.
    nBlobs=salience->DrawContrastLP2(*ptr_inputImgRG, *ptr_inputImgGR, *ptr_inputImgBY,
        *outContrastLP, *ptr_tagged, max_tag,
        weightBU, weightTD,
        targetRG, targetGR, targetBY, 255); 
}

void blobFinderThread::convolve1D(int vecSize, float* vec, ImageOf<PixelMono>* img, ImageOf<PixelMono>* resImg, float factor,int shift,int direction, int maxVal){
    
    // ASSUME: No overflow, same size image
    float maxPixelVal = 0;
    float minPixelVal = -256;
    int maxRange = 255;
    int ROIRowStart = vecSize/2;
    int ROIRowEnd = img->height()-vecSize/2;
    int ROIColEnd = img->width() - vecSize/2;
    int ROIColStart = vecSize/2;    
    int vecStart = -vecSize/2;
    int vecEnd = vecSize/2;
    int rowSize = img->getRowSize();
    int resRowSize = resImg->getRowSize();
    unsigned char* mat = img->getRawImage();
    unsigned char* res = resImg->getRawImage();
    float* midVec = vec + vecSize/2; // middle of linear kernel
    int pixelPos = 0; int pixPos =0;    
    
    if(direction == 0){ //horizontal convolution
        for(int i=0;i<resImg->height();++i){
            for(int j=0;j<resImg->width();++j){
                pixelPos = i*resRowSize;
                //float* tmpVec = midVec;
                pixPos = j;
                float sum = *midVec * *(mat+pixelPos+pixPos);
                pixPos--;
                for(int k=0; k<vecSize/2 && pixPos>0; k++, pixPos--){
                    sum += (*(mat+pixelPos+pixPos))* (*(midVec-k));
                    //tmpVec++;
                }
                pixPos = j+1;
                for(int k=0; k<vecSize/2 && pixPos<img->width(); k++, pixPos++){
                    sum += (*(mat+pixelPos+pixPos))* (*(midVec+k));
                    //tmpVec++;
                }
                sum *= factor; 
                //if(sum>maxPixelVal) maxPixelVal=sum;
                //else if(sum<minPixelVal) minPixelVal=sum;               
                *(res+i*resRowSize+j)=sum;//+ shift;//<0?0: sum>maxVal? maxVal:sum;
            }
        } 
    } 
    else {
        for(int i=0;i<resImg->height();++i){
            for(int j=0;j<resImg->width();++j){
                pixelPos = j;
                //float* tmpVec = midVec;
                pixPos = i;
                float sum = *midVec * *(mat+pixPos*rowSize+pixelPos);
                pixPos--;
                for(int k=0; k<vecSize/2 && pixPos>0; k++, pixPos--){
                    sum += (*(mat+pixelPos+pixPos*rowSize))* (*(midVec-k));
                    //tmpVec++;
                }
                pixPos = i+1;
                for(int k=0; k<vecSize/2 && pixPos<img->height(); k++, pixPos++){
                    sum += (*(mat+pixelPos+pixPos*rowSize))* (*(midVec+k));
                    //tmpVec++;
                }
                sum *= factor;
                if(sum>maxPixelVal) maxPixelVal=sum;
                else if(sum<minPixelVal) minPixelVal=sum;  
                *(res+i*resRowSize+j)=sum;//<0?0: sum>maxVal? maxVal:sum;
            }
        } 
    }
    
}


