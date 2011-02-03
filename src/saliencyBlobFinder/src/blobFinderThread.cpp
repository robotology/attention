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
#define thresholdDB 0
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

    outContrastLP=new ImageOf<PixelMono>;
    outMeanColourLP=new ImageOf<PixelBgr>;

    _procImage = new ImageOf<PixelRgb>;
    _outputImage3 = new ImageOf<PixelRgb>;

    ptr_inputImg = new ImageOf<yarp::sig::PixelRgb>; 
    ptr_inputImgRed = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgGreen = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgBlue = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgRG = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgGR = new ImageOf<yarp::sig::PixelMono>; 
    ptr_inputImgBY = new ImageOf<yarp::sig::PixelMono>; 
    edges = new ImageOf<yarp::sig::PixelMono>; 
    img = new ImageOf<PixelRgb>;
    tmpImage = new ImageOf<PixelMono>;
    image_out = new ImageOf<PixelMono>;
    
    _inputImgRGS = new ImageOf<PixelMono>;
    _inputImgGRS = new ImageOf<PixelMono>;
    _inputImgBYS = new ImageOf<PixelMono>;

    // some standard parameters on the blob search.
    maxBLOB = 4096;
    minBLOB = 100;
    minBoundingArea = 225;
}

blobFinderThread::~blobFinderThread() {
    delete outContrastLP;
    delete outMeanColourLP;
    delete _procImage;
    delete _outputImage3;
    delete ptr_inputImg;        // pointer to the input image
    delete edges;               // pointer to the edges image

    delete ptr_inputImgRed;     // pointer to the input image of the red plane
    delete ptr_inputImgGreen;   // pointer to the input image of the green plane
    delete ptr_inputImgBlue;    // pointer to the input image of the blue plane
    delete ptr_inputImgRG;      // pointer to the input image of the R+G- colour opponency
    delete ptr_inputImgGR;      // pointer to the input image of the G+R- colour opponency
    delete ptr_inputImgBY;      // pointer to the input image of the B+Y- colour opponency

    delete _inputImgRGS;
    delete _inputImgGRS;
    delete _inputImgBYS;

    delete img;
    delete tmpImage;
    delete image_out;
    delete wOperator;
    delete salience;
    delete ptr_tagged;
}

void blobFinderThread::setName(std::string str) {
    this->name=str; 
}

std::string blobFinderThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void blobFinderThread::reinitialise(int width, int height) {
    this->width = width;
    this->height = height;

    img->resize(width, height);
    edges->resize(width, height);
    tmpImage->resize(width, height);
    image_out->resize(width, height);
    
    resizeImages(width, height);

    reinit_flag = true;
}

void blobFinderThread::resizeImages(int width, int height) {
    srcsize.height = height;
    srcsize.width = width;

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

    blobList = new char [width*height+1];

    ptr_inputImg->resize(width, height);
    ptr_inputImgRed->resize(width, height);
    ptr_inputImgGreen->resize(width, height);
    ptr_inputImgBlue->resize(width, height);
    ptr_inputImgRG->resize(width, height);
    ptr_inputImgGR->resize(width, height);
    ptr_inputImgBY->resize(width, height);

    resized_flag = true;
}

/**
 * initialization of the thread 
 */
bool blobFinderThread::threadInit() {
    
    inputPort.open(getName("/image:i").c_str());
    edgesPort.open(getName("/edges:i").c_str());
    rgPort.open(getName("/rg:i").c_str());
    grPort.open(getName("/gr:i").c_str());
    byPort.open(getName("/by:i").c_str());
    blobDatabasePort.open(getName("/database").c_str());
    saliencePort.open(getName("/salienceMap:o").c_str());
    outputPort3.open(getName("/imageC3:o").c_str());

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

    string robot("icub");
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

    // remove constraints on the links
    // we use the chains for logging purpose
    eyeL->setAllConstraints(false);
    eyeR->setAllConstraints(false);

    // release links
    eyeL->releaseLink(0);
    //eyeC.releaseLink(0);
    eyeR->releaseLink(0);
    eyeL->releaseLink(1);
    //eyeC.releaseLink(1);
    eyeR->releaseLink(1);
    eyeL->releaseLink(2);
    //eyeC.releaseLink(2);
    eyeR->releaseLink(2);

    // get camera projection matrix from the configFile
    if (getCamPrj(configFile,"CAMERA_CALIBRATION_LEFT",&PrjL)) {
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
    rgPort.interrupt();             // open(getName("rg:i"));
    grPort.interrupt();             // open(getName("gr:i"));
    byPort.interrupt();             // open(getName("by:i"));
    blobDatabasePort.interrupt();
    saliencePort.interrupt();
    outputPort3.interrupt();
}

/**
 * active loop of the thread
 */
void blobFinderThread::run() {
    img = inputPort.read(false);
    if (0 != img) {
        if (!reinit_flag) {
            srcsize.height=img->height();
            srcsize.width=img->width();
            height=img->height();
            width=img->width();
            reinitialise(img->width(), img->height());
        }

        ippiCopy_8u_C3R(img->getRawImage(), img->getRowSize(), ptr_inputImg->getRawImage(), ptr_inputImg->getRowSize(), srcsize);
        bool ret1=true, ret2=true;
        ret1 = getOpponencies();
        ret2 = getPlanes(img);
        if (!ret1 || !ret2)
            return;

        tmpImage=edgesPort.read(false);
        if (tmpImage != 0)
            ippiCopy_8u_C1R(tmpImage->getRawImage(), tmpImage->getRowSize(), edges->getRawImage(), edges->getRowSize(), srcsize);

        rain(edges);
        drawAllBlobs(false);

        //salience->DrawMaxSaliencyBlob(*salience->maxSalienceBlob_img, max_tag, *tagged);
        //ippiCopy_8u_C1R(salience->maxSalienceBlob_img->getRawImage(), salience->maxSalienceBlob_img->getRowSize(), _outputImage->getRawImage(), _outputImage->getRowSize(), srcsize);

        salience->ComputeMeanColors(max_tag);
        salience->DrawMeanColorsLP(*outMeanColourLP, *ptr_tagged);

        /*ippiCopy_8u_C3R(outMeanColourLP->getRawImage(), outMeanColourLP->getRowSize(), _outputImage3->getRawImage(), _outputImage3->getRowSize(), srcsize);	
        if((0 != _outputImage3) && (outputPort3.getOutputCount())) { 
            outputPort3.prepare() = *(_outputImage3);
            outputPort3.write();
        }
        */

        if((0 != _outputImage3) && (outputPort3.getOutputCount())) { 
            outputPort3.prepare() = *((ImageOf<PixelRgb>*)outMeanColourLP);
            outputPort3.write();
        }

        if((0 != outContrastLP) && (saliencePort.getOutputCount())) { 
            saliencePort.prepare() = *(outContrastLP);
            saliencePort.write();
        }

        if(blobDatabasePort.getOutputCount()) {
            
            
            Vector fp;
            YARPBox* pBlob = salience->getBlobList();
            int u, v, z;
            bool isLeft = true;             //TODO :  remove hardcoded here!
            for (int i = 1; i < nBlobs; i++) {
                if ((pBlob[i].valid)&&(pBlob[i].areaLP > thresholdDB)) {
                    printf("areaLP:", pBlob[i].areaLP);
                    
                    u = 160; //pBlob[i].centroid_x;
                    v = 120; //pBlob[i].centroid_y;
                    z = 0.5;

                    Matrix  *invPrj=(isLeft?invPrjL:invPrjR);
                    iCubEye *eye=(isLeft?eyeL:eyeR);

                    if (invPrj) {
                        
                        Vector torso(3);
                        igaze->getAngles(torso);
                        Vector head(4);
                        igaze->getAngles(head);

                        Vector q(8);
                        q[0]=torso[0];
                        q[1]=torso[1];
                        q[2]=torso[2];
                        q[3]=head[0];
                        q[4]=head[1];
                        q[5]=head[2];
                        q[6]=head[3];

                        if (isLeft)
                            q[7]=head[4]+head[5] / 2.0;
                        else
                            q[7]=head[4]-head[5] / 2.0;
                            

                        Vector x(3);
                        x[0]=z * u;
                        x[1]=z * v;
                        x[2]=z;

                        // find the 3D position from the 2D projection,
                        // knowing the distance z from the camera
                        Vector xe = yarp::math::operator *(*invPrj, x);
                        xe[3]=1.0;  // impose homogeneous coordinates                

                        // update position wrt the root frame
                        Vector xo = yarp::math::operator *(eye->getH(q),xe);

                        fp.resize(3,0.0);
                        fp[0]=xo[0];
                        fp[1]=xo[1];
                        fp[2]=xo[2];
                    }
                    printf("object %f,%f,%f \n",fp[0],fp[1],fp[2]);

                    
                    Bottle request, reply;
                    request.clear(); reply.clear();
                    request.addVocab(VOCAB3('a','d','d'));
                    Bottle& listAttr=request.addList();
                    Bottle& sublist=listAttr.addList();
                    sublist.addString("x");
                    sublist.addDouble(1.0 * 1000);
                    sublist=listAttr.addList();
                    sublist.clear();

                    sublist.addString("y");
                    sublist.addDouble(1.0 * 1000);
                    sublist=listAttr.addList();
                    sublist.clear();

                    sublist.addString("z");
                    sublist.addDouble(1.0 * 1000);
                    sublist=listAttr.addList();
                    sublist.clear();

                    sublist.addString("r");
                    sublist.addDouble(pBlob[i].meanColors.r);
                    sublist=listAttr.addList();
                    sublist.clear();

                    sublist.addString("g");
                    sublist.addDouble(pBlob[i].meanColors.g);
                    sublist=listAttr.addList();
                    sublist.clear();

                    sublist.addString("b");
                    sublist.addDouble(pBlob[i].meanColors.b);
                    sublist=listAttr.addList();
                    sublist.clear();

                    sublist.addString("lifeTimer");
                    sublist.addInt(3);
                    sublist=listAttr.addList();
                    sublist.clear();
                    
                    
                    //listAttr.add(sublist);
                    //blobDatabasePort.write(request, reply);
                    //extract ID of the object added
                    //int id = reply.get(1).asList()->get(1).asInt();
                    //printf("id of the object properties %d", id);
                    
                    
                    // set the lifetime
                    /*
                    printf("cleared");
                    request.addVocab(VOCAB3('s','e','t'));
                    listAttr=request.addList();
                    sublist=listAttr.addList();
                    sublist.addString("id");
                    sublist.addInt(id);
                    sublist=listAttr.addList();
                    sublist.clear();

                    sublist.addString("lifeTimer");
                    sublist.addInt(100);
                    sublist=listAttr.addList();
                    sublist.clear();
                    */

                    //blobDatabasePort.write(request, reply);
                    
                   
                }
            }
        }
    }
}

/**
 *	releases the thread
 */
void blobFinderThread::threadRelease() {
    rgPort.close();
    grPort.close();
    byPort.close();
    blobDatabasePort.close();
    outputPort3.close();
    saliencePort.close();
    inputPort.close();
}

/**
 * function that reads the ports for colour RGB opponency maps
 */
bool blobFinderThread::getOpponencies() {

    tmpImage=rgPort.read(false);
    if (tmpImage != NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(), tmpImage->getRowSize(), ptr_inputImgRG->getRawImage(), ptr_inputImgRG->getRowSize(), srcsize);
    
    tmpImage=grPort.read(false);
    if (tmpImage != NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(), tmpImage->getRowSize(), ptr_inputImgGR->getRawImage(), ptr_inputImgGR->getRowSize(), srcsize);
    
    tmpImage=byPort.read(false);
    if (tmpImage != NULL)
        ippiCopy_8u_C1R(tmpImage->getRawImage(), tmpImage->getRowSize(), ptr_inputImgBY->getRawImage(), ptr_inputImgBY->getRowSize(), srcsize);

    return true;
}

/**
 * LATER: proper Doxygen documentation here!
 * function that reads the ports for the RGB planes
 */
bool blobFinderThread::getPlanes(ImageOf<PixelRgb>* inputImage) {
    Ipp8u* shift[3];
    shift[0] = ptr_inputImgRed->getRawImage(); 
    shift[1] = ptr_inputImgGreen->getRawImage();
    shift[2] = ptr_inputImgBlue->getRawImage();
    ippiCopy_8u_C3P3R(inputImage->getRawImage(), inputImage->getRowSize(), shift, ptr_inputImgRed->getRowSize(), srcsize);
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

    // extracts the PixelBgr color of tag=1. Assuming this is the fovea?
    //PixelBgr varFoveaBlob = salience->varBlob(*ptr_tagged, *ptr_inputImgRG, *ptr_inputImgGR, *ptr_inputImgBY, 1 /* (*ptr_tagged)(0,0) */);

    // draw the fovea blob into the blobFov image? Also assuming the tag=1.
    //salience->drawFoveaBlob(*blobFov, *tagged);
    
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

    PixelMono pixelRG = 0;
    PixelMono pixelGR = 0;
    PixelMono pixelBY = 0;

    // draws the saliency of all blobs into the tagged image and returns a mono image.
    nBlobs=salience->DrawContrastLP2(*ptr_inputImgRG, *ptr_inputImgGR, *ptr_inputImgBY,
        *outContrastLP, *ptr_tagged, max_tag,
        1.0, 0.0,
        pixelRG, pixelGR, pixelBY, 255); 
}

