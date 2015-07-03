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
 * @file featExtractorThread.cpp
 * @brief Implementation of the thread that represent image (see header featExtractorThread.h)
 */

#include <iCub/featExtractorThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 66   //15 fps

featExtractorThread::featExtractorThread() : RateThread(THRATE) {
    synchronised = false;
    count  = 0;
    width  = 320;   //default dimension width 
    height = 240;   //default dimension height
}

featExtractorThread::~featExtractorThread() {

}

bool featExtractorThread::threadInit() {
    printf("\n starting the thread.... \n");

    //opening port
    outputPortDescr.open      (getName("/descrResult:o").c_str());      //ddd
    //initialisig flag
    featDataready      = false;

    // initialising images

    //uImage      = new ImageOf<PixelMono>;
    //uImage->resize(width,height);

    //vImage      = new ImageOf<PixelMono>;
    //vImage->resize(width,height);

    //mImage      = new ImageOf<PixelMono>;
    //mImage->resize(width,height);

    Ut_1 = cv::Mat::zeros(height, width, CV_32FC1);  //I have already use U and V to put them in U_1 and V_1  //?or is it better to initialize U, V, U_1, V_1 in the threadInit?
    Vt_1 = cv::Mat::zeros(height, width, CV_32FC1);

    printf("initialization in feature  extractor thread correctly ended \n");
    timeCounter=0;
    return true;
}

void featExtractorThread::interrupt() {
    outputPortDescr.interrupt();      //ddd
}



void featExtractorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string featExtractorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void featExtractorThread::resize(int widthp, int heightp) {
}

void featExtractorThread::setFlag() {
    sem.wait();
    featDataready=true;
    sem.post();
}

/*
void featExtractorThread::setFlagPointer(bool* flagPointer) {           //p1
	sem.wait();
    featDataready=flagPointer;           //p1=p2
	sem.post();
}
*/


void featExtractorThread::copyAll(cv::Mat U, cv::Mat V, cv::Mat M) {
    sem.wait();
    Ut=U.clone(); 
    Vt=V.clone();
    Maskt=M;
    sem.post();

}

void featExtractorThread::convertMat2ImageOf(cv::Mat a, ImageOf<PixelMono>* image) {
    IplImage tempIpla = (IplImage) a;
    //printf("%08x \n", image);
    image-> wrapIplImage(&tempIpla);
}


bool featExtractorThread::test(){
    bool ret = true;
    ret &= Network::exists(getName("/image:i").c_str());
    return ret;
}

void featExtractorThread::run() {    //uImage,vImage,mImage
//ddd

    if(outputPortDescr.getOutputCount()) {
        //yDebug("plotting the image");
        Bottle& outbot = outputPortDescr.prepare();
        outbot.clear();
        outbot = contentBottle;//descrBottle
        outputPortDescr.write();
    }

    int DELTA = 10;
    int LATO = DELTA*2+1;

    bool tempReady;
    sem.wait();
    tempReady = featDataready;          //this is to protect featDataready
    sem.post();


    if(tempReady){

        //yInfo("ciaoooooooo");
        //cv::Mat Ut((IplImage*) uImage->getIplImage(), false);
        //cv::Mat Vt((IplImage*) vImage->getIplImage(), false);
        //cv::Mat Maskt((IplImage*) mImage->getIplImage(), false);

        descr.clear();

        /*if(!COND) {
    
            if(DEBUG_) {
                cv::Mat Im = cv::Mat::zeros(Ut.rows, Ut.cols, CV_8UC1);
                cv::imshow("DEBUG", Im);
                cv::waitKey(10);
                Im.release();
      
            }
    
            MAGt.release();
            THETAt.release();
            MAGt_1.release();
            THETAt_1.release();
            Maskt.release();
            computed = 0;
            return;
        }*/


        /*for(int x = 0; x < NewMask.cols; ++x) {
            for(int y = 0; y < NewMask.rows; ++y) {
                if(NewMask.at<uchar>(y,x) > 0) {
	                CUM.at<float>(y,x) += MAGt.at<float>(y,x);   //CUM is just to  visualize the evolution of the region (it is something like to put together the binary matrices)->it can be commented
                }
            }
        }*/


        /*if(DEBUG_) {
    

        double minVal, maxVal;
        cv::Point maxPos;
        cv::minMaxLoc(CUM, &minVal, &maxVal, NULL, &maxPos, NewMask);
    
        cv::Mat Ig = cv::Mat::zeros(CUM.rows, CUM.cols, CV_8UC3);
        cv::Mat C = 255*(CUM-((float)(minVal)))/(((float)(maxVal))-((float)(minVal)));     //Normalization of CUM
        C.convertTo(Ig, CV_8UC3);
    
        cv::circle(Ig, maxPos, 3, CV_RGB(255,0,0));
    
        cv::imshow("DEBUG", NewMask);
        //cv::imshow("DEBUG", NewMask3);
        cv::waitKey(10);
        //If.release();
        //Im.release();
        //Imrgb.release();
    
        }*/
 
        cv::Mat VEL = cv::Mat::zeros(1,3,CV_32FC1); 
        cv::Mat ACC = cv::Mat::zeros(1,3,CV_32FC1);
        cv::Moments MOMt;
        double minValt_1, maxValt_1;
        cv::Point maxPost_1;
        cv::Mat Probt_1;
        cv::Mat Maskt_1;
        cv::Mat mean, std;


        cv::Mat MatV = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
        cv::Mat MatC = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
        cv::Mat MatR = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
        cv::Mat MatA = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
        int delta = 3;

        //AAA togliere commenti e decidere dove settare il case, per ora c'e' solo quella del centroide
        //switch(flag_) {     //dove lo setto
        //case 0: // analisi del centroide  

        // devo trovare il centroide della roi         //roi=region of interest
        //aaa da qui decommenta
        MOMt = cv::moments(Maskt, true);    //moments(Calculates all of the moments up to the third order of a polygon or rasterized shape.)
        cv::Point maxPost;
        if (MOMt.m00 == 0)
            MOMt.m00 = 0.000001;
        maxPost.x = MOMt.m10/MOMt.m00;
        maxPost.y = MOMt.m01/MOMt.m00;
        // spatial moments
        //double  m00, m10, m01, m20, m11, m02, m30, m21, m12, m03;
        // central moments
        //double  mu20, mu11, mu02, mu30, mu21, mu12, mu03;
        // central normalized moments
        //double  nu20, nu11, nu02, nu30, nu21, nu12, nu03;

        MAGt_1 = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
        THETAt_1 = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);  
        float a = Ut_1.depth();
        float b = Ut.depth();
        float c = Maskt.depth();


        double minvala, maxvala;	
		cv::Point  minLoca, maxLoca;
		cv::minMaxLoc(Ut,&minvala, &maxvala, &minLoca, &maxLoca);

        double minvalz, maxvalz;	
		cv::Point  minLocz, maxLocz;
		cv::minMaxLoc(Ut_1,&minvalz, &maxvalz, &minLocz, &maxLocz);

        cv::cartToPolar(Ut_1, Vt_1, MAGt_1, THETAt_1, false);
        double minValM, maxValM;
        cv::minMaxLoc(MAGt_1, &minValM, &maxValM);

        Probt_1 = cv::Mat::zeros(MAGt_1.rows, MAGt_1.cols, CV_32FC1);

        for(int i = DELTA; i < Probt_1.rows-DELTA; ++i) {        //as  before  during segmentation -> Probt_1 will be a matrix of values of probabilities between 0 and 1 
            for(int j = DELTA; j < Probt_1.cols-DELTA; ++j) {
                if(MAGt_1.at<float>(i,j)> TH1__)
                    Probt_1.at<float>(i,j) = ((float)(cv::sum(MAGt_1(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1)) >= TH2__).val[0]))/((float)(LATO*LATO));
            }
        }
      
            Maskt_1 = Probt_1 >= PTH__;

        //COND = cv::sum(Maskt_1).val[0] >= 0.05*(255*Ut.rows*Ut.cols);   //?rimane sempre a 0, quindi  ho  tolto l'if successivo  
        //if(COND) {
        //  yInfo("interno del  cond");
        //}

        //    devo trovare il centroide della roi
        cv::Moments MOMt_1 = cv::moments(Maskt_1, true);
        if (MOMt_1.m00 == 0) {
            MOMt_1.m00 = 0.000001;
        }
        maxPost_1.x = MOMt_1.m10/MOMt_1.m00;      //??
        maxPost_1.y = MOMt_1.m01/MOMt_1.m00;

        // DESCRITTORE: Vt Ct Rt At
        cv::Mat Maskt8U;
        Maskt.convertTo(Maskt8U, CV_8UC1);
        VEL.at<float>(0,0) = cv::mean(Ut, Maskt8U).val[0];   //crasha
        VEL.at<float>(0,1) = cv::mean(Vt, Maskt8U).val[0]; 
        VEL.at<float>(0,2) = 0.1;  //10 fps  (1/fps=1/10=0.1;  1/30=0.0333333)
	
        descr.push_back(13);
        timeCounter++;
        descr.push_back(timeCounter);

        float roba =  sqrt(VEL.at<float>(0,0) * VEL.at<float>(0,0) + VEL.at<float>(0,1) * VEL.at<float>(0,1) + VEL.at<float>(0,2)*VEL.at<float>(0,2))  ;
        descr.push_back(roba);                                              //V=is the norm of the velocity (mean of optical flow)

        ACC.at<float>(0,0) =  cv::mean(Ut, Maskt8U).val[0] - cv::mean(Ut_1, Maskt_1).val[0]; 
        ACC.at<float>(0,1) = cv::mean(Vt, Maskt8U).val[0] - cv::mean(Vt_1, Maskt_1).val[0]; 
        ACC.at<float>(0,2) = 0.;
	
        descr.push_back(cv::norm(VEL.cross(ACC))/pow(cv::norm(VEL),3));   //C=Curvature
        descr.push_back(1/descr[3]);                                      //R=Radius of curvature
        descr.push_back(descr[2]/descr[4]);                               //A=V/R
	
        FILE *fid = fopen("provaalvolo.txt","a" );
        //fprintf(fid, "%lf\n", maxValM);
        fprintf(fid, "%lf\n", cv::norm(VEL));
        fclose(fid);


        descr.push_back(maxPost.x);
        descr.push_back(maxPost.y);


        //std::cout  << "  Ut Vt  " <<  cv::mean(Ut, Maskt8U).val[0] << " " <<  cv::mean(Vt, Maskt8U).val[0] << " "  << std::endl;  

        //std::cout  << "Descriptor    " <<  descr[2] << " " << descr[3] << " " << descr[4] << " " << descr[5]  << std::endl;  

        computed = 1;   

        contentBottle = descrBottle.addList();

        contentBottle.addDouble(descr[0]);
        contentBottle.addDouble(descr[1]);           
        contentBottle.addDouble(descr[2]);           
        contentBottle.addDouble(descr[3]);           
        contentBottle.addDouble(descr[4]);           
        contentBottle.addDouble(descr[5]);           

        //printf("bottle is %s\n", contentBottle.toString().c_str());

        //  } else {
        //computed = 0;
        //  }

        Ut_1=Ut.clone();
        Vt_1=Vt.clone();
        int e=Ut_1.depth();
        int f=Ut.depth();


        sem.wait();
        featDataready=false;
        sem.post();

        } 
}


void featExtractorThread::threadRelease() {
    printf("featExtractorThread: portClosing \n");  
    outputPortDescr.close();   //ddd
    printf("freeing memory \n");

    // free allocated memory here please
    //delete uImage;
    //delete vImage;
    //delete mImage;

    printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ----------------