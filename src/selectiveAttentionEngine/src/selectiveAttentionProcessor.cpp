// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file selectiveAttentionProcessor.cpp
 * @brief Implementation of the thread of selective attention module (see header file).
 */

#include <iCub/selectiveAttentionProcessor.h>


#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <cstdio>


using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::sig::draw;
using namespace yarp::dev;
using namespace std;
using namespace iCub::logpolar;

#define XSIZE_DIM        320   // original mapping 
#define YSIZE_DIM        240   // original mapping
#define TIME_CONST       50    // number of times period rateThread to send motion command
#define BASELINE         0.068 // distance in millimeters between eyes
#define MAXCOUNTERMOTION 40   

template<class T>

inline int round(T a) {
    int ceilValue=(int)ceil(a);
    int floorValue=(int)floor(a);
    if(a-floorValue<=0.5)
        return floorValue;
    else
        return ceilValue;
}

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

inline void crossAssign(unsigned char* p,int value, int rowsize) {
    
    // in the conversion between cartesian and logpolar the single maxresponse pixel can go lost
    // enhancing the response of the neightbourhood 
    *p = value;
    p++;              *p = value;
    p -= 2;           *p = value;
    p += 1 + rowsize; *p = value;
    p -= 2 * rowsize; *p = value;
    p += rowsize;
 
}                    

void selectiveAttentionProcessor::copy_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    bool hue = false;
    bool sat = false;
    if(dest == hueMap) {
        hue = true;
    }
    else if(dest == satMap) {
        sat = true;
    }

    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    unsigned char* pface = faceMask->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            /*
            if(sat) {
                if((*psrc>200)&&(*psrc<240)) {
                    *pface = (unsigned char) 127;
                }
                else {
                    *pface = (unsigned char) 0;
                }
            }
            if(hue) {
                if((*psrc>200)&&(*psrc<240)) {
                    *pface = (unsigned char) 127; 
                }
                else {
                    *pface = (unsigned char) 0;
                }
            }
            pface++;
            */
            
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
        pface += padding;
    }
}

selectiveAttentionProcessor::selectiveAttentionProcessor(int rateThread):RateThread(rateThread) {
    this->inImage = new ImageOf<PixelRgb>;
    earlystage = false;
    secondstage = false;
    reinit_flag = false;
    inputImage_flag = 0;
    idle = true;
    interrupted = false;
    gazePerform = true;
    handFixation = false;
    directSaccade = false;

    cLoop=0;
    endInt=0;
    startInt=Time::now();
    saccadeInterv = 3000; //milliseconds    
    counterMotion = 0;

    // images initialisation
    edges_yarp       = new ImageOf <PixelMono>;
    tmp              = new ImageOf <PixelMono>;
    
    
    map1_yarp        = new ImageOf <PixelMono>; // intensity
    map2_yarp        = new ImageOf <PixelMono>; // motion
    map3_yarp        = new ImageOf <PixelMono>; // chrominance
    map4_yarp        = new ImageOf <PixelMono>; // orientation
    map5_yarp        = new ImageOf <PixelMono>; // edges 
    map6_yarp        = new ImageOf <PixelMono>; // blob
    motion_yarp      = new ImageOf <PixelMono>;
    cart1_yarp       = new ImageOf <PixelMono>;
    faceMask         = new ImageOf <PixelMono>;
    habituationImage = new ImageOf <PixelMono>;
    linearCombinationPrev = new ImageOf <PixelMono>;
    
    tmp=new ImageOf<PixelMono>;
    hueMap = 0;
    satMap = 0;
    image_out=new ImageOf<PixelRgb>;
    image_tmp=new ImageOf<PixelMono>;
}

selectiveAttentionProcessor::~selectiveAttentionProcessor(){
    delete inImage;
    delete habituation;
    delete edges_yarp;

    delete map1_yarp;
    delete map2_yarp;
    delete map3_yarp;
    delete map4_yarp;
    delete map5_yarp;
    delete map6_yarp;
    delete inputLogImage;
    delete motion_yarp;
    delete cart1_yarp;
    delete faceMask;

    delete tmp;
    delete image_out;
    delete image_tmp;
    delete intermCartOut;
    delete habituationImage;

    delete habituation;
    delete linearCombinationPrev;
}

selectiveAttentionProcessor::selectiveAttentionProcessor(ImageOf<PixelRgb>* inputImage):RateThread(THREAD_RATE) {
    this->inImage=inputImage;
    tmp=new ImageOf<PixelMono>;
}

void selectiveAttentionProcessor::reinitialise(int width, int height){
    this->width  = width;
    this->height = height;

    inImage = new ImageOf<PixelRgb>;
    inImage->resize(width,height);
   
    map1_yarp->resize(width,height);
    map1_yarp->zero();
    
    map2_yarp->resize(width,height);
    map2_yarp->zero();
    
    map3_yarp->resize(width,height);
    map3_yarp->zero();
    
    map4_yarp->resize(width,height);
    map4_yarp->zero();
    
    map5_yarp->resize(width,height);
    map5_yarp->zero();
    
    map6_yarp->resize(width,height);
    map6_yarp->zero();
    
    faceMask->resize(width,height);
    faceMask->zero();

    inputLogImage         = new ImageOf<PixelRgb>;   
    intermCartOut         = new ImageOf<PixelRgb>;
    motion_yarp           = new ImageOf<PixelMono>;
    cart1_yarp            = new ImageOf<PixelMono>;
    inhicart_yarp         = new ImageOf<PixelMono>;
    habituationImage      = new ImageOf<PixelMono>;
    linearCombinationPrev = new ImageOf<PixelMono>;
        
    inputLogImage->resize(width,height);
    intermCartOut->resize(xSizeValue,ySizeValue);
    motion_yarp->resize(xSizeValue,ySizeValue);
    cart1_yarp->resize(xSizeValue,ySizeValue);
    inhicart_yarp->resize(xSizeValue,ySizeValue);
    habituationImage->resize(xSizeValue,ySizeValue);
    linearCombinationPrev->resize(xSizeValue,ySizeValue);

    motion_yarp->zero();     
    cart1_yarp->zero();      
    inhicart_yarp->zero();
    habituationImage->zero();     
    linearCombinationPrev->zero();

    habituation      = new float[width * height];
}

void selectiveAttentionProcessor::resizeImages(int width,int height) {

    tmp->resize(width,height);
    inImage->resize(width,height);
    
    tmp->resize(width,height);
    image_out->resize(width,height);
    image_tmp->resize(width,height);

    cvImage16 = cvCreateImage(cvSize(width,height),IPL_DEPTH_16S,1);
    cvImage8  = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U,1);
}

/**
*	initialization of the thread 
*/
bool selectiveAttentionProcessor::threadInit(){
    printf("Thread initialization .... \n");
    //opening ports"
    vergencePort.open(getName("/vergence:o").c_str());

    map1Port.open(getName("/intensity:i").c_str());
    map2Port.open(getName("/motion:i").c_str());
    map3Port.open(getName("/chrominance:i").c_str());
    map4Port.open(getName("/orientation:i").c_str());
    map5Port.open(getName("/edges:i").c_str());
    map6Port.open(getName("/blobs:i").c_str());

    cart1Port.open(getName("/cart1:i").c_str());
    motionPort.open(getName("/motionCart:i").c_str());

    inhiCartPort.open(getName("/inhiCart:i").c_str());
    inhiPort.open(getName("/inhi:i").c_str());

    linearCombinationPort.open(getName("/combination:o").c_str());
    centroidPort.open(getName("/centroid:o").c_str());
    vergenceCmdPort.open(getName("/vergence:i").c_str());
    outputCmdPort.open(getName("/cmd:o").c_str());
    feedbackPort.open(getName("/feedback:o").c_str());
    imageCartOut.open(getName("/cartesian:o").c_str());
    thImagePort.open(getName("/wta:o").c_str());
    portionRequestPort.open(getName("/portionRequest:o").c_str());
    testPort.open(getName("/test:o").c_str());

    //initializing logpolar mapping
    cout << "||| initializing the logpolar mapping" << endl;

    if (!trsf.allocLookupTables(L2C, numberOfRings, numberOfAngles, xSizeValue, ySizeValue, overlap)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "||| lookup table allocation done" << endl;

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
    else {
        return false;
    }

    /*
    // set up the ARM MOTOR INTERFACE	
    name = getName("");
    string localName = "/" + name + "/armCtrl";
    string remoteName;

    remoteName = "/" + robotName + "/left_arm";
    //remoteName = "/" + robotName + "/right_arm";
    Property options;
    options.put("device", "remote_controlboard");	
    options.put("local", localName.c_str());                 //local port names
    options.put("remote", remoteName.c_str());				//where we connect to
    armRobotDevice = new PolyDriver(options);
    if (!armRobotDevice->isValid()) {
        printf("initialization failed: arm device not available.\n");
        return false;
    }
    armRobotDevice->view(armPos);
    armRobotDevice->view(armEnc);
    armEnc->getAxes(&jointNum);

    // SET UP the CARTESIAN INTERFACE
    localName = name + "/cartArm";
    remoteName = "/" + robotName + "/cartesianController/left_arm";
    //remoteName = "/cartesianSolver/left_arm";
    //remoteName = "/" + robotName + "/cartesianController/right_arm";

    options.clear();
    options.put("device","cartesiancontrollerclient");
    options.put("local", localName.c_str());                //local port names
    options.put("remote", remoteName.c_str());              //where we connect to
    cartCtrlDevice = new PolyDriver(options);
    if (!cartCtrlDevice->isValid()) {
       printf("initialization failed: cartesian controller not available.\n");
       return false;
    }
    cartCtrlDevice->view(armCart);
    */

    habituationStart = Time::now();

    return true;
}

void selectiveAttentionProcessor::setName(std::string str){
    this->name=str; 
}


std::string selectiveAttentionProcessor::getName(const char* p){
    string str(name);
    str.append(p);
    //printf("name: %s", name.c_str());
    return str;
}

void selectiveAttentionProcessor::setRobotName(std::string str) {
    this->robotName = str;
    printf("name: %s", name.c_str());
}

/**
* active loop of the thread
*/
void selectiveAttentionProcessor::run(){
    cLoop++;
    //synchronisation with the input image occuring
    if(!interrupted){

        // code for fixating its head
        if((outputCmdPort.getOutputCount())&&(handFixation)) {
            Vector x(3); Vector o(4);
            armCart->getPose(x,o);
            printf("position of the choosen link in meters %f,%f,%f \n", x(0), x(1), x(2));
            Bottle& commandBottle=outputCmdPort.prepare();
            commandBottle.clear();
            commandBottle.addString("SAC_ABS");
            commandBottle.addDouble(x(0));
            commandBottle.addDouble(x(1));
            commandBottle.addDouble(x(2));
            outputCmdPort.write();
            return; 
        }                        
        
        //--read value from the preattentive level
        if(feedbackPort.getOutputCount()){

            float xm=0,ym=0;
            
            /*
            Bottle in,out;
            out.clear();
            out.addString("get");
            out.addString("ktd");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            salienceTD=in.pop().asDouble();
            out.clear();
            in.clear();
            
            out.addString("get");
            out.addString("kbu");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            salienceBU=in.pop().asDouble();
            out.clear();
            in.clear();
            
            out.addString("get");
            out.addString("rin");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            targetRED=in.pop().asInt();
            out.clear();
            in.clear();
            
            out.addString("get");
            out.addString("gin");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            targetGREEN=in.pop().asInt();
            out.clear();
            in.clear();
            
            out.addString("get");
            out.addString("bin");
            feedbackPort.write(out,in);
            name=in.pop().asString();
            targetBLUE=in.pop().asDouble();
            out.clear();
            in.clear();
            */
        }
    
        //printf("reading input signals \n");
        // reading intensity map
        tmp=map1Port.read(false);
        if((tmp == 0)&&(!reinit_flag)){
            return;
        }
        if(!reinit_flag){
            reinitialise(tmp->width(), tmp->height());
            reinit_flag = true;
        }
        
        idle = false;
        if((map1Port.getInputCount())&&(k1!=0)) {
            if(tmp != 0) {
                copy_8u_C1R(tmp,map1_yarp);
                //idle=false;
            }
        }

        //reading motion map       
        if((map2Port.getInputCount())&&(k2!=0)) { 
            tmp = map2Port.read(false);
            if(tmp != 0) {
                copy_8u_C1R(tmp,map2_yarp);
                //idle=false;
            }
        }
        
        ImageOf<PixelMono>& linearCombinationImage = linearCombinationPort.prepare();
        linearCombinationImage.resize(width,height);

        double sumK = k1 + k2 + k3 + k4 + k5 + k6 + kmotion + kc1;  //added kmotion and any coeff.for cartesian map to produce a perfect balance within clues 
        unsigned char  maxValue    = 0;
        unsigned char* pmap1Left   = map1_yarp->getRawImage();
        unsigned char* pmap1Right  = map1_yarp->getRawImage(); 
        unsigned char* pmap2Left   = map2_yarp->getRawImage();  
        unsigned char* pmap2Right  = map2_yarp->getRawImage();
        unsigned char* pmap3Left   = map3_yarp->getRawImage();
        unsigned char* pmap3Right  = map3_yarp->getRawImage();
        unsigned char* pmap4Left   = map4_yarp->getRawImage();
        unsigned char* pmap4Right  = map4_yarp->getRawImage();
        unsigned char* plinearLeft = linearCombinationImage.getRawImage();
        unsigned char* plinearRight= linearCombinationImage.getRawImage();    
        unsigned char* pmap1       = map1_yarp->getRawImage();
        unsigned char* pmap2       = map2_yarp->getRawImage();  
        unsigned char* pmap3       = map3_yarp->getRawImage();
        unsigned char* pmap4       = map4_yarp->getRawImage();
        unsigned char* pmap5       = map5_yarp->getRawImage();
        unsigned char* pmap6       = map6_yarp->getRawImage();
        unsigned char* pface       = faceMask ->getRawImage();
        unsigned char* pHabituationImage  = habituationImage->getRawImage();
        float * pHabituation              = habituation;
        
        unsigned char* plinear     = linearCombinationImage.getRawImage();
        unsigned char* plinearprev = linearCombinationPrev->getRawImage();

        int ratioX     = xSizeValue / XSIZE_DIM;    //introduced the ratio between the dimension of the remapping and 320
        int ratioY     = ySizeValue / YSIZE_DIM;    //introduced the ration between the dimension of the remapping and 240
        int padding    = map1_yarp->getPadding();
        int rowSize    = map1_yarp->getRowSize();
        int halfwidth  = width  >> 1;
        int halfheight = height >> 1;

        double timeVariable;

        // ------------ early stage of response ---------------
        //printf("entering the first stage of vision....\n");
        if(earlystage) {

            ImageOf<PixelMono>& tmpImage = testPort.prepare();
            tmpImage.resize(height,width);
            tmpImage.zero();

            unsigned char* ptmp        = tmpImage.getRawImage();
            ptmp         += halfwidth - 1;            
            pmap1Left    += halfwidth - 1;                   
            pmap1Right   += halfwidth;                  
            pmap2Left    += halfwidth - 1;                  
            pmap2Right   += halfwidth;
            plinearLeft  += halfwidth - 1;
            plinearRight += halfwidth;
            
            // exploring the image from rho=0 and from theta = 0
            double value;
            double threshold = 255;
            double thresholdInt = 225;
            
            for(int y = 0 ; y < height ; y++){
                for(int x = 0 ; x < halfwidth ; x++){
                   
                    //------------- motion ----------------------
                    
                    if(counterMotion >= MAXCOUNTERMOTION) {

                        //value = (k2/sumK) *  (double) *pmap2Right ;                   
                        value = *pmap2Right;
                        if(*pmap2Right >= threshold){
                            printf("max in motion Right %d %f \n",*pmap2Right, value); 
                        }
                        *plinearRight = value;
                        if (value >= threshold) {
                            printf("max in motion Right \n");                    
                            printf("max in motion Right \n");
                            printf("max in motion Right \n");
                            printf("max in motion Right \n");
                            *plinearRight = 255;
                            crossAssign(plinearRight, 255, rowSize);
                            xm = halfwidth + x;
                            ym = y;
                            timing = 0.1;
                            //printf("Jumping to cartSpace \n");
                            goto cartSpace;
                            //y = height;// for jumping out of the outer loop
                            //idle = true;                        
                            //break;
                        }                
                        pmap2Right++;
                        
                        
                        //value = (k2/sumK) * (double) *pmap2Left;
                        value = *pmap2Left;
                        if(*pmap2Left >= threshold){
                            printf("max in motion Left %d %f \n",*pmap2Left, value); 
                        }
                        *ptmp = *pmap2Left;
                        *plinearLeft = value;
                        if (value >= threshold) {
                            printf("max in motion Left    \n", (unsigned char) *pmap2Left); 
                            printf("max in motion Left    \n", (unsigned char) *pmap2Left); 
                            printf("max in motion Left    \n", (unsigned char) *pmap2Left); 
                            printf("max in motion Left    \n", (unsigned char) *pmap2Left); 
                            *plinearLeft = 255;
                            crossAssign(plinearLeft, 255, rowSize);
                            xm = halfwidth - x;
                            ym = y;
                            timing = 0.1;
                            //printf("Jumping to cartSpace \n");
                            goto cartSpace;
                            //y = height;// for jumping out of the outer loop
                            //idle = true;                        
                            //break;
                        }
                        
                        pmap2Left--;
                        ptmp--;
                        
                    }
                    
                    
                    // ----------- intensity ---------------------
                    //value = (k1/sumK) * (double) *pmap1Right;
                    value = 0.80 * (double) *pmap1Right;
                    //*plinearRight = value;
                    if (value >= thresholdInt){
                        printf("max in intesity Right %f \n",value);                    
                        //*plinearRight = 255;
                        //crossAssign(plinearRight, 255, rowSize);
                        xm = halfwidth + x;
                        ym = y;
                        timing = 0.1;
                        //printf("Jumping to cartSpace \n");
                        goto cartSpace;
                        //y = height;// for jumping out of the outer loop
                        //idle =  true;                        
                        //break;
                    }
                    pmap1Right++;
                    
                    
                    //value = (k1/sumK) * (double) *pmap1Left;
                    value = 0.85 * (double)*pmap1Left;
                    //*plinearLeft = value;
                    if (value >= thresholdInt){
                        printf("max in intensity Left %f \n",value);
                        //*plinearRight = 255;
                        //crossAssign(plinearLeft, 255, rowSize);                   
                        xm = halfwidth - x;
                        ym = y;
                        timing = 0.1;
                        //printf("Jumping to cartSpace \n");
                        goto cartSpace;
                        //y = height;// for jumping out of the outer loop
                        //idle =  true;                        
                        //break;
                    }
                    pmap1Left--;
                    
                    
                    
                    
                    // moving pointer of the plinear
                    // in this version only the motion map is saved in the plinear
                    plinearRight++;
                    plinearLeft--;
                    
                }
                pmap1Right   += rowSize - halfwidth;
                pmap1Left    += rowSize + halfwidth;
                pmap2Right   += rowSize - halfwidth;
                pmap2Left    += rowSize + halfwidth;
                
                ptmp         += rowSize + halfwidth;
                plinearRight += rowSize - halfwidth;
                plinearLeft  += rowSize + halfwidth;
            }
            //handling the counterMotion
            counterMotion++;
            if(counterMotion >= MAXCOUNTERMOTION){ 
                counterMotion = MAXCOUNTERMOTION;
            }
            if(counterMotion == MAXCOUNTERMOTION - 1 ){
                printf("counterMotion %d going to remove magnocellular suppression\n", counterMotion);
            }
            //tmpImage = *(map2_yarp);
            testPort.write();

        } //end of the early stage


        //reading the second set of maps
        pmap1Left  = map1_yarp->getRawImage();
        pmap1Right = map1_yarp->getRawImage();
        pmap1Left  += halfwidth - 1;        
        pmap1Right += halfwidth;
    
        pmap2Left  = map2_yarp->getRawImage();
        pmap2Right = map2_yarp->getRawImage();
        pmap2Left  += halfwidth - 1;
        pmap2Right += halfwidth;

        if((map3Port.getInputCount())&&(k3!=0)) {
            tmp = map3Port.read(false);
            if(tmp!= 0) {
                copy_C1R(tmp,map3_yarp);
                //idle=false;
            }
        }
        if((map4Port.getInputCount())&&(k4!=0)) {
            tmp = map4Port.read(false);
            if(tmp!= 0) {
                copy_C1R(tmp,map4_yarp);
                //idle=false;
            }
        }
        
        pmap3Left   = map3_yarp->getRawImage();
        pmap3Right  = map3_yarp->getRawImage();
        pmap4Left   = map4_yarp->getRawImage();
        pmap4Right  = map4_yarp->getRawImage();


        pmap3Right += halfwidth;
        pmap4Right += halfwidth;
        pmap3Left  += halfwidth - 1;
        pmap4Left  += halfwidth - 1;

        plinearLeft  = linearCombinationImage.getRawImage();
        plinearRight = linearCombinationImage.getRawImage();
        plinearLeft += halfwidth - 1;
        plinearRight+= halfwidth;
       
        if(secondstage) {
            //printf("activating the second stage of early vision... \n");             
            //------ second stage of response  ----------------            
            for(int y = 0 ; y < height; y++){
                for(int x = 0 ; x < halfwidth; x++){
                    //unsigned char value = *pmap1++ + *pmap2++ + *pmap3++ + *pmap4++;
                    double value = (double) (*pmap1Right++ * (k1/sumK) + *pmap2Right++ * (k2/sumK) + *pmap3Right++ * (k3/sumK) + *pmap4Right++ * (k4/sumK));
                    *plinearRight++ = value;
                    if (value >= 255) {
                        printf("max in the second stage right \n");
                        xm = x + width;
                        ym = y;
                        timing = 0.5;
                        goto cartSpace;
                        //y = height;    // for jumping out of outer loop
                        //idle =  true;
                        //break;
                    } 

                    value = (double) (*pmap1Left-- * (k1/sumK) + *pmap2Left-- * (k2/sumK) + *pmap3Left-- * (k3/sumK) + *pmap4Left-- * (k4/sumK));
                    *plinearLeft-- = value;
                    if (value >= 255) {
                        printf("max in the second stage left \n");
                        xm = x - width;
                        ym = y;
                        timing = 0.5;
                        goto cartSpace;
                        //y = height;   // for jumping out of the outer loop
                        //idle =  true;
                        //break;
                    }                     
                }

                pmap1Right   += rowSize - halfwidth;
                pmap1Left    += rowSize + halfwidth;
                pmap2Right   += rowSize - halfwidth;
                pmap2Left    += rowSize + halfwidth;
                pmap3Right   += rowSize - halfwidth;
                pmap3Left    += rowSize + halfwidth;
                pmap4Right   += rowSize - halfwidth;
                pmap4Left    += rowSize + halfwidth;
                plinearRight += rowSize - halfwidth;
                plinearLeft  += rowSize + halfwidth;
            }            
        }//end of the idle after first two stages of response
        

        //2. processing of the input images
        //if(!idle) {
        //printf("processing the whole compilation of feature maps \n ");
            timing = 1.0;
            if((map5Port.getInputCount())&&(k5!=0)) {
                tmp = map5Port.read(false);
                if(tmp!= 0) {
                    copy_C1R(tmp,map5_yarp);
                    //idle=false;
                }
            }
            if((map6Port.getInputCount())&&(k6!=0)) {
                tmp = map6Port.read(false);
                if(tmp!= 0) {
                    copy_C1R(tmp,map6_yarp);
                    //idle=false;
                }
            }
            
            if((motionPort.getInputCount())&&(kmotion!=0)) {
                tmp = motionPort.read(false);
                if(tmp!= 0) {
                    copy_8u_C1R(tmp,motion_yarp);
                    //idle=false;
                }
            }
            if((cart1Port.getInputCount())&&(kc1!=0)) {
                tmp = cart1Port.read(false);
                if(tmp!= 0) {             
                    copy_8u_C1R(tmp,cart1_yarp);
                    //idle=false;
                }
            }
            if(inhiPort.getInputCount()) {
                tmp = inhiPort.read(false);
                if(tmp!= 0) {
                    copy_8u_C1R(tmp,inhi_yarp);
                    //idle=false;
                }
            }            
            
            // combination of all the feature maps
            // thresholding for habituation comes at this stage because express response resets habituation
            //printf("combining the feature maps \n");
            habituationStop = Time::now();
            timeVariable    = habituationStop - habituationStart;
            for(int y = 0 ; y < height ; y++){
                for(int x = 0 ; x < width ; x++){
                    unsigned char value;
                    if(sumK==0) {
                        value=0;
                    }
                    else {
                        double combinValue = (double) ((*pmap1 * (k1/sumK) + *pmap2 * (k2/sumK) + *pmap3 * (k3/sumK) + *pmap4 * (k4/sumK) + *pface * (k5/sumK) + *pmap6 * (k6/sumK)));
                        value=(unsigned char)ceil(combinValue);
                    }
                    
                    if((value < thresholdHabituation)&&(value > 0)&&(abs((double)*plinearprev - value) <= 20.0)) {                          //&&(abs((double)*plinearprev - value) <= 10.0)
                    //*pHabituationImage = round((double)*pHabituationImage * 2 + 1);                        
                        if(*pHabituation <= 254.0) {
                            *pHabituation += exp(timeVariable / 50);
                        }
                        *pHabituationImage = round(*pHabituation);
                    }
                    else {
                        //*pHabituationImage = round((double)*pHabituationImage / 2 - 1);  
                        if(*pHabituation >= 1.0) {
                            *pHabituation -= exp(timeVariable / 50);
                        }
                        *pHabituationImage = round(*pHabituation);
                        
                    }                    
                                        
                    if(*pHabituation >= 255) {
                        *pHabituation = 255;
                    }
                    else if(*pHabituation <= 0) {
                        *pHabituation = 0;
                    }
                    
                    // printf("pHabituation %f \n", *pHabituation);
                    //*pHabituation = 0;                    

                    pmap1++;
                    pmap2++;
                    pmap3++;
                    pmap4++;
                    pmap5++;
                    pmap6++;
                    pface++;
                    
                    if(value >= *pHabituationImage) {
                        *plinear = value - *pHabituationImage;
                    }
                    else {
                        *plinear = 0;
                    }
                    double alfa  = 0.6;
                    *plinearprev =round( 0.6 * (double)*plinearprev + (1 - 0.6) * (double)*plinear);  //saving the actual value in the prevstep image
                    
                    //*plinear = max (0, min (255, (int) *plinear));
                    pHabituationImage++;
                    pHabituation++;
                    plinear++;
                    plinearprev++;
                }
                pmap1       += padding;
                pmap2       += padding;
                pmap3       += padding;
                pmap4       += padding;
                pmap5       += padding;
                pmap6       += padding;
                plinear     += padding;
                plinearprev += padding;
                pface       += padding;

                pHabituationImage += padding;
            }
            
            
            
            
//********************************************************************************************
cartSpace:
            //trasform the logpolar to cartesian (the logpolar image has to be 3channel image)
            //printf("trasforming the logpolar image into cartesian \n");
            plinear = linearCombinationImage.getRawImage();
            int rowsize = linearCombinationImage.getRowSize();
            unsigned char* pImage = inputLogImage->getRawImage();
            int padding3C = inputLogImage->getPadding();
            maxValue = 0;
            
            //TODO: reduce the cycle steps if the maximum has been already found            
            for(int y = 0 ; y < height ; y++) {
                for(int x = 0 ; x < width ; x++) {                    
                    *pImage++ = (unsigned char) *plinear;
                    *pImage++ = (unsigned char) *plinear;
                    *pImage++ = (unsigned char) *plinear;
                    plinear++;
                }
                pImage  += padding3C;
                plinear += padding;
            }
            ImageOf<PixelRgb>  &outputCartImage = imageCartOut.prepare();  // preparing the cartesian output for combination
            ImageOf<PixelMono> &threshCartImage = thImagePort.prepare();   // preparing the cartesian output for WTA
            ImageOf<PixelMono> &inhiCartImage   = inhiCartPort.prepare();  // preparing the cartesian image for inhibith a portion of the saliency map            
            
            // the ratio can be used to assure that the saccade command is located in the plane image (320,240)
            int outputXSize = xSizeValue;
            int outputYSize = ySizeValue;
            outputCartImage.resize(outputXSize,outputYSize);
            threshCartImage.resize(outputXSize,outputYSize);
            threshCartImage.zero();
            //printf("outputing cartesian image dimension %d,%d-> %d,%d \n", width,height , intermCartOut->width() , intermCartOut->height());
            trsf.logpolarToCart(*intermCartOut,*inputLogImage);

            //code for preparing the inhibition of return 
            if((inhiCartPort.getInputCount())&&(portionRequestPort.getOutputCount())) {
                //send information about the portion
                //double azimuth   =  10.0;
                //double elevation = -10.0;
                Vector angles(3);
                bool b = igaze->getAngles(angles);
                //printf(" azim %f, elevation %f, vergence %f \n",angles[0],angles[1],angles[2]);
                Bottle* sent     = new Bottle();
                Bottle* received = new Bottle();    
                sent->clear();
                sent->addString("fetch");
                sent->addDouble(angles[0]);
                sent->addDouble(angles[1]);
                portionRequestPort.write(*sent, *received);
                delete sent;
                delete received;
            }
            Time::delay(0.05);
            if(inhiCartPort.getInputCount()) {            
                tmp = inhiCartPort.read(false);
                if(tmp!= 0) {
                    copy_8u_C1R(tmp,inhicart_yarp);
                    //idle=false;
                }
            }

            //find the max in the cartesian image and downsample
            maxValue=0;            
            int countMaxes=0;
            pImage = outputCartImage.getRawImage();
            unsigned char* pInter    = intermCartOut->getRawImage();
            unsigned char* pcart1    = cart1_yarp->getRawImage();
            unsigned char* pinhicart = inhicart_yarp->getRawImage();
            unsigned char* pmotion   = motion_yarp->getRawImage();
            int paddingInterm        = intermCartOut->getPadding(); //padding of the colour image
            int rowSizeInterm        = intermCartOut->getRowSize();
            //double sumCart=2 - kc1 - kmotion + kmotion + kc1;
            int paddingCartesian = cart1_yarp->getPadding();
            int paddingOutput    = outputCartImage.getPadding();
            //adding cartesian and finding the max value
            //removed downsampling of the image.
            
            
            maxResponse = false;
            for(int y=0; (y < ySizeValue) && (!maxResponse); y++) {
                for(int x=0; (x < xSizeValue) && (!maxResponse); x++) {
                    //double combinValue = (double) (*pcart1 * (kc1/sumK) + *pInter * ((k1 + k2 + k3 + k4 + k5 + k6)/sumK) + *pmotion * (kmotion/sumK));
                    double combinValue   = (double)  *pInter;
                    //if(combinValue >= 255.0) {
                    //    printf("maxResponse for combinValue \n");
                    //}
                    if(*pinhicart > 10) {
                        combinValue = 0;
                    }
                    
                    if(combinValue < 0) {
                        combinValue = 0;
                    }
                    unsigned char value = (unsigned char) ceil(combinValue);
                    //unsigned char value=*pInter;
                    if((y==ySizeValue>>1)&&(x==xSizeValue>>1)){
                        *pImage = 0;
                    }
                    else {
                        *pImage = value;
                    }
                    if(value == 255) {
                        maxResponse = true;
                        printf("maxResponse!!!!!! ");
                        xm = (float) x;
                        ym = (float) y;
                        startInt = 0;      // forces the immediate saccade to the very salient object
                        break;
                    }
                    if(maxValue < value) {
                        maxValue = value;                 
                    }
                    if((y==ySizeValue>>1)&&(x==xSizeValue>>1)){
                       pImage++; pInter++;
                        *pImage = 255;
                        pImage++; pInter++;
                        *pImage = 0;
                        pImage++; pInter++; 
                    }
                    else {
                        pImage++; pInter++;
                        *pImage = value;
                        pImage++; pInter++;
                        *pImage = value;
                        pImage++; pInter++;
                    }
                    pcart1++;
                    pmotion++;
                    pinhicart++;
                }
                pImage    += paddingOutput;
                pInter    += paddingInterm;
                pcart1    += paddingCartesian;
                pmotion   += paddingCartesian;
                pinhicart += paddingCartesian;
            }
           
            if(!maxResponse) {                
                pImage = outputCartImage.getRawImage();                
                float distance = 0;
                bool foundmax=false;
                //looking for the max value 
                for(int y = 0 ; y < ySizeValue ; y++) {
                    for(int x = 0 ; x < xSizeValue ; x++) {
                        //*pImage=value;
                        if(*pImage==maxValue) {
                            if(!foundmax) {
                                *pImage=255;pImage++;
                                *pImage=0;pImage++;
                                *pImage=0;pImage-=2;
                                countMaxes++;
                                xm = (float)x;
                                ym = (float)y;
                                foundmax = true;
                            }
                            else {
                                distance = sqrt((x-xm)*(x-xm)+(y-ym)*(y-ym));
                                // beware:the distance is useful to decrease computation demand but the WTA is selected in the top left hand corner!
                                if(distance < 10) {
                                    *pImage = 255;
                                    pImage++;
                                    *pImage=0;
                                    pImage++;
                                    *pImage=0;
                                    pImage-=2;
                                    //*pThres = 255; pThres++;
                                    countMaxes++;
                                    xm += x;
                                    ym += y;
                                }
                                else {
                                    break;
                                }
                            }
                        }
                        pImage+=3;
                    }
                    pImage += paddingOutput;
                }
                xm = xm / countMaxes;
                ym = ym / countMaxes;
            }

            //representation of red lines where the WTA point is
            //representation of the vertical line
            pImage = outputCartImage.getRawImage();
            pImage += (int)round(xm) * 3;
            for(int i = 0 ; i < ySizeValue ; i++) {
                *pImage = 255; pImage++;
                *pImage = 0;   pImage++;
                *pImage = 0;   pImage++;
                pImage += (xSizeValue - 1) * 3 + paddingOutput;
            }
            //representation of the horizontal line
            pImage = outputCartImage.getRawImage();
            pImage += (int) round(ym) * (3 * (xSizeValue) + paddingOutput);
            for(int i = 0 ; i < xSizeValue; i++) {
                *pImage++ = 255;
                *pImage++ = 0;
                *pImage++ = 0;
            }
            
            
            
            //representing the depressing gaussian
            //unsigned char* pThres = threshCartImage.getRawImage();
            //int paddingThresh = threshCartImage.getPadding();
            //int rowsizeThresh = threshCartImage.getRowSize();
            //pThres +=   ((int)ym - 5) * rowsizeThresh + ((int)xm - 5);
            //calculating the peek value
            //int dx = 30.0;
            //int dy = 30.0;
            //double sx = (dx / 2) / 3 ; //0.99 percentile
            //double sy = (dy / 2) / 3 ;
            //double vx = 8; //sx * sx; // variance          
            //double vy = 8; //sy * sy;
           
            //double rho = 0;
            
            //double a = 0.5 / (3.14159 * vx * vy * sqrt(1-rho * rho));
            //double b = -0.5 /(1 - rho * rho);
            //double k = 1 / (a * exp (b));
                                     
 
            //double f, e, d;            

            //double zmax = 0;
            //for the whole blob in this loop
            //for (int r = ym - (dy>>1); r <= ym + (dy>>1); r++) {
            //    for (int c = xm - (dx>>1); c <= xm + (dx>>1); c++){
            //        
            //        if((c == xm)&&(r == ym)) { 
            //            //z = a * exp (b);
            //            //z = z * k;
            //            z = 1;
            //        }
            //        else {    
            //            f = ((c - xm) * (c - xm)) /(vx * vx);
            //            d = ((r - ym)  * (r - ym)) /(vy * vy);
            //            //e = (2 * rho* (c - ux) * (r - uy)) / (vx * vy);
            //            e = 0;
            //            z = a * exp ( b * (f + d - e) );
            //            z = z * k;
            //            z = (1 / 1.638575) * z;
            //            //z = 0.5;
            //        }
            //        
            //        // restrincting the z gain between two thresholds
            //        if (z > 1) {
            //            z = 1;
            //        }
            //        //if (z < 0.3) {
            //        //    z = 0.3;
            //        //}
            //        
            //        //set the image 
            //        *pThres++ = 255 * z;                    
            //    }
            //    pThres += rowsizeThresh - (dx + 1) ;
            //}
            
            //} //end of the last idle.

        //---------- SECTION 2 ----------
        // requiring the saccade starting from xm,ym coordinates
        // controlling the heading of the robot
        endInt=Time::now();
        double diff = endInt - startInt;
        // idle: when any of the first two stages of response fires
        // maxresponse: when within the linear combination one region fires
        // the rest is a constant rate firing
        if((diff * 1000 > saccadeInterv)||(idle)||(maxResponse)) {
            habituationStart = Time::now(); //resetting exponential of habituation
            memset(habituation, 0, height * width * sizeof(float));
            printf("gazePerforming after %d idle %d maxResponse %d \n", saccadeInterv, idle, maxResponse);
            if(gazePerform) {
                Vector px(2);
                // ratio maps the WTA to an image 320,240 (see definition) because it is what iKinGazeCtrl asks
                px[0] = round(xm / ratioX - 80);   // divided by ratioX because the iKinGazeCtrl receives coordinates in image plane of 320,240
                px[1] = round(ym / ratioY);        // divided by ratioY because the iKinGazeCtrl receives coordinates in image plane of 320,240
                centroid_x = round(xm / ratioX);   // centroid_x is the value of gazeCoordinate streamed out
                centroid_y = round(ym / ratioY);   // centroid_y is the value of gazeCoordinate streamed out
                                
                //if(vergencePort.getOutputCount()) {
                //    //suspending any vergence control;
                //    Bottle& command=vergencePort.prepare();
                //    command.clear();
                //    command.addString("sus");
                //    printf("suspending vergerce \n");
                //    vergencePort.write();
                //    //waiting for the ack from vergence
                //    bool flag=false;
                //}
                
                
                //for(int k=0;k<10;k++){ 
                //    Time::delay(0.050);
                //    printf("*");
                //}
                
                
                if(vergenceCmdPort.getInputCount()) {
                    Vector angles(3);
                    bool b = igaze->getAngles(angles);
                    printf(" azim %f, elevation %f, vergence %f \n",angles[0],angles[1],angles[2]);
                    double vergence   = (angles[2] * 3.14) / 180;
                    double version    = (angles[0] * 3.14) / 180;
                    double leftAngle  = version + vergence / 2.0;
                    double rightAngle = version - vergence / 2.0;
                    z = BASELINE / (2 * sin ( vergence / 2 )); //in m
                    
                    //if(leftAngle >= 0) {
                    //    if(rightAngle >= 0) {
                    //        rightHat = 90 - rightAngle;
                    //        leftHat = 90 - leftAngle;
                    //        alfa = 90 - rightHat;
                    //        h = BASELINE * (
                    //            (sin(leftHat) * sin(rightHat)) / 
                    //            (sin(rightHat) * sin(vergence + alfa) - sin(alfa) * sin(leftHat))
                    //            );
                    //    }
                    //    else {
                    //        if(rightAngle >= leftAngle) {
                    //            rightHat = 90 - rightAngle;
                    //            leftHat = 90 - leftAngle;
                    //            alfa = 90 - rightHat;
                    //            h = BASELINE * (
                    //            (sin(leftHat) * sin(rightHat)) / 
                    //            (sin(leftHat) * sin(vergence + alfa) + sin(alfa) * sin(rightHat))
                    //            );
                    //        }
                    //        else {
                    //            rightHat = 90 - rightAngle;
                    //            leftHat = 90 - leftAngle;
                    //            alfa = 90 - rightHat;
                    //            h = BASELINE * (
                    //           (sin(leftHat) * sin(rightHat)) / 
                    //            (sin(rightHat) * sin(vergence + alfa) + sin(alfa) * sin(leftHat))
                    //            );
                    //        }
                    //    }
                    //}
                    //else {
                    //   rightHat = 90 - rightAngle;
                    //    leftHat = 90 - leftAngle;
                    //    alfa = 90 - rightHat;
                    //    h = BASELINE * (
                    //            (sin(leftHat) * sin(rightHat)) / 
                    //            (sin(leftHat) * sin(vergence + alfa) - sin(alfa) * sin(rightHat))
                    //            );
                    //}
                    
                }
                
                if(directSaccade) {
                    igaze->lookAtMonoPixel(camSel,px,z);
                }
                
                
                if(outputCmdPort.getOutputCount()){
                    if(!handFixation) {            
                        printf("sending saccade mono \n");
                        Bottle& commandBottle=outputCmdPort.prepare();
                        commandBottle.clear();
                        commandBottle.addString("SAC_MONO");
                        commandBottle.addInt(centroid_x);
                        commandBottle.addInt(centroid_y);
                        commandBottle.addDouble(z);
                        commandBottle.addDouble(timing);
                        outputCmdPort.write();
                    } 
                }

                //if(vergencePort.getOutputCount()) { 
                //    //waiting for the end of the saccadic event
                //    bool flag=false;
                //    bool res=false;
                //    while(!flag) {
                //        igaze->checkMotionDone(&flag);
                //        Time::delay(0.010);
                //    }
                //    for(int k=0;k<10;k++){ 
                //        Time::delay(0.050);
                //    } 
                //    //suspending any vergence control
                //    Bottle& command=vergencePort.prepare();
                //    //resuming vergence
                //    command.clear();
                //    command.addString("res");
                //    printf("resuming vergence \n");
                //    vergencePort.write();
                //}
                
                //adding the element to the DB
                //if(databasePort.getOutputCount()) {
                //    //suspending any vergence control
                //    Bottle& command=databasePort.prepare();
                //    command.clear();
                //    command.addInt(xm);
                //    command.addInt(ym);
                //    databasePort.write();
                //}
                
            } //ifgaze Arbiter
            startInt=Time::now();
        } //if diff
        outPorts();
        //}// end idle
    }
}

void selectiveAttentionProcessor::setGazePerform(bool value) {
    gazePerform=value;
}

void selectiveAttentionProcessor::setCamSelection(int value) {
    camSel=value;
}

void selectiveAttentionProcessor::setXSize(int xSize) {
    xSizeValue=xSize;
}

void selectiveAttentionProcessor::setYSize(int ySize) {
    ySizeValue=ySize;
}

void selectiveAttentionProcessor::setSaccadicInterval(int interval) {
    this->saccadeInterv=interval;
}

void selectiveAttentionProcessor::setOverlap(double _overlap) {
    overlap=_overlap;
}

void selectiveAttentionProcessor::setNumberOfRings(int _numberOfRings) {
    numberOfRings=_numberOfRings;
}

void selectiveAttentionProcessor::setNumberOfAngles(int _numberOfAngles) {
    numberOfAngles=_numberOfAngles;
}

bool selectiveAttentionProcessor::outPorts(){
    bool ret = false;
    if(linearCombinationPort.getOutputCount()){
        linearCombinationPort.write();
    }
    if(imageCartOut.getOutputCount()){
        imageCartOut.write();
    }
    if(thImagePort.getOutputCount()) {
        thImagePort.write();
    }
    if(centroidPort.getOutputCount()){  
        Bottle& commandBottle=centroidPort.prepare();
        commandBottle.clear();
        commandBottle.addString("sac");
        commandBottle.addString("img");
        commandBottle.addInt(centroid_x);
        commandBottle.addInt(centroid_y);
        centroidPort.write();
    }
    if(feedbackPort.getOutputCount()){
        //Bottle& commandBottle=feedbackPort.prepare();
        Bottle in,commandBottle;
        commandBottle.clear();
        
        
        time (&end2);
        double dif = difftime (end2,start2);
        if(dif > 30 + 2){
                //restart the time interval
                 time(&start2);
        }
        else if((dif>2)&&(dif<30+2)){
            //setting coefficients
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','t','d'));
            salienceTD=salienceTD+0.1;
        
            //if(salienceTD>0.99)
                salienceTD=1.0;
            printf("salienceTD \n");
            commandBottle.addDouble((double)salienceTD);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','b','u'));
            salienceBU=salienceBU-0.1;
            
            //if(salienceBU<=0)
                salienceBU=0;
            commandBottle.addDouble((double)salienceBU);
            feedbackPort.write(commandBottle,in);    
            printf("read: %f,%f,%f \n",(double)targetRED,(double)targetGREEN,(double)targetBLUE);
            
        }
        else{
            printf("salienceBU \n");
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','t','d'));
            salienceTD=0.0;
            commandBottle.addDouble((double)salienceTD);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('k','b','u'));
            salienceBU=1.0;
            commandBottle.addDouble((double)salienceBU);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('r','i','n'));
            commandBottle.addDouble((double)targetRed);
            //commandBottle.addDouble(255.0);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('g','i','n'));
            commandBottle.addDouble((double)targetGreen);
            //commandBottle.addDouble(0.0);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            commandBottle.addVocab(VOCAB3('s','e','t'));
            commandBottle.addVocab(VOCAB3('b','i','n'));
            commandBottle.addDouble((double)targetBlue);
            feedbackPort.write(commandBottle,in);
            commandBottle.clear();
            //commandBottle.addDouble(0.0);
            printf("%f,%f,%f \n",(double)targetRed,(double)targetGreen,(double)targetBlue);
        }
    }
    return true;
}


void selectiveAttentionProcessor::setHueMap(int p) {
    switch (p) {
        case 1: {
            hueMap = map1_yarp;
            }
            break;
        case 2: { 
            hueMap = map2_yarp;
            }
        break;
        case 3: { 
            hueMap = map3_yarp;
            }
            break;
        case 4: { 
            hueMap = map4_yarp;
            }
            break;
        case 5: { 
            hueMap = map5_yarp;
            }
            break;
        case 6: { 
            hueMap = map5_yarp;
            }
            break;
        default:{
                 hueMap = 0;
                }
            break;
    }
}


void selectiveAttentionProcessor::setSatMap(int p) {
    switch (p) {
        case 1: {
            satMap = map1_yarp;
            }
            break;
        case 2: { 
            satMap = map2_yarp;
            }
        break;
        case 3: { 
            satMap = map3_yarp;
            }
            break;
        case 4: { 
            satMap = map4_yarp;
            }
            break;
        case 5: { 
            satMap = map5_yarp;
            }
            break;
        case 6: { 
            satMap = map5_yarp;
            }
            break;
        default:{
                 satMap = 0;
                }
            break;
    }
}


void selectiveAttentionProcessor::extractContour(ImageOf<PixelMono>* inputImage,ImageOf<PixelRgb>* inputColourImage,int& x,int& y) { 

}


void selectiveAttentionProcessor::getPixelColour(ImageOf<PixelRgb>* inputColourImage,int x ,int y, unsigned char &targetRed, unsigned char &targetGreen, unsigned char &targetBlue){
    //printf("max image dim:%d with rowsize %d \n",inImage->getRawImageSize(),inImage->getRowSize());
    unsigned char pColour=inImage->getRawImage()[(x*3)+y*inImage->getRowSize()];
    targetRed=pColour;
    //pColour++;
    pColour=inImage->getRawImage()[(x*3)+1+y*inImage->getRowSize()];
    targetGreen=pColour;
    //pColour++;
    pColour=inImage->getRawImage()[(x*3)+2+y*inImage->getRowSize()];
    targetBlue=pColour;
    //printf("colour found: %d %d %d \n",(int) targetBlue,(int)targetGreen,(int)targetRed);
}

/**
* function called when the module is poked with an interrupt command
*/
void selectiveAttentionProcessor::interrupt(){
    interrupted=true;
    printf("interrupting the module.. \n");
    vergencePort.interrupt();
    map1Port.interrupt();
    map2Port.interrupt();
    map3Port.interrupt();
    map4Port.interrupt();
    map5Port.interrupt();
    map6Port.interrupt();

    cart1Port.interrupt();
    inhiCartPort.interrupt();
    inhiPort.interrupt();
    motionPort.interrupt();
    
    portionRequestPort.interrupt();
    linearCombinationPort.interrupt();
    centroidPort.interrupt();
    outputCmdPort.interrupt();
    thImagePort.interrupt();
    vergenceCmdPort.interrupt();
    feedbackPort.interrupt();
}

/**
*	releases the thread
*/
void selectiveAttentionProcessor::threadRelease(){
    trsf.freeLookupTables();

    printf("Thread realeasing .... \n");
    printf("Closing all the ports.. \n");
    //closing input ports
    vergencePort.close();
    map1Port.close();
    map2Port.close();
    map3Port.close();
    map4Port.close();
    map5Port.close();
    map6Port.close();

    cart1Port.close();
    inhiCartPort.close();
    inhiPort.close();
    motionPort.close();

    linearCombinationPort.close();
    centroidPort.close();
    feedbackPort.close();
    outputCmdPort.close();
    thImagePort.close();
    imageCartOut.close();
    vergenceCmdPort.close();
    portionRequestPort.close();

    delete clientGazeCtrl;
    //delete cartCtrlDevice;
}

void selectiveAttentionProcessor::suspend() {
    printf("suspending processor....after stopping control \n");
    if(igaze!=0) {
        igaze->stopControl();
    }
    RateThread::suspend();
}

void selectiveAttentionProcessor::resume() {
    RateThread::resume();
}

void selectiveAttentionProcessor::setIdle(bool value){
    mutex.wait();
    idle=value;
    mutex.post();
}



//----- end-of-file --- ( next line intentionally left blank ) ------------------

