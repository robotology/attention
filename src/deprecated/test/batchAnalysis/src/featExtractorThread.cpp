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
#include <opencv2/imgproc/imgproc.hpp> 




using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10 //THRATE 66   //15 fps

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
    Maskt_1 = cv::Mat::zeros(height, width, CV_32FC1);

    currentSmoothedV = 0.0;
    currentSmoothedC = 0.0;
    currentSmoothedR = 0.0;
    currentSmoothedA = 0.0;

    Vmin=100.0;
    Vmax=0.000001;
    Cmin=100.0;
    Cmax=0.000001;
    Rmin=100.0;
    Rmax=0.000001;
    Amin=100.0;
    Amax=0.000001;


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


void featExtractorThread::copyAll(cv::Mat U, cv::Mat V, cv::Mat M, int seqID,  int nFrame) {
    sem.wait();
    Ut=U.clone(); 
    Vt=V.clone();
    Maskt=M;
    sequenceID=seqID;
    nFr=nFrame;
    sem.post();

}

void featExtractorThread::convertMat2ImageOf(cv::Mat a, ImageOf<PixelMono>* image) {
    IplImage tempIpla = (IplImage) a;
    //printf("%08x \n", image);
    image-> wrapIplImage(&tempIpla);
}

void featExtractorThread::buffering(int bufferSize, std::vector<float>& buffer, float   data){
        if (buffer.size()<bufferSize)
            buffer.push_back(data);
        if (buffer.size()>=bufferSize){
            for (int i=0;  i<bufferSize-1; i++){
                buffer[i]=buffer[i+1];
            }
        buffer.pop_back(); 
        buffer.push_back(data);
        }
}

void featExtractorThread::convolution(std::vector<float>& buffer, std::vector<float>& kernel, std::vector<float>& smoothedBuffer, float& currentSmoothed){
    for(int i = kernel.size() -1; i < buffer.size(); ++i) 
    {
        smoothedElem[i] = 0;                             // init to 0 before accumulate
        int j;
        int k;
        for( j = i, k = 0; k < kernel.size(); --j, ++k)
            smoothedElem[i] += buffer[j] * kernel[k];
        currentSmoothed= smoothedElem[i];
        smoothedBuffer.push_back(currentSmoothed);
        

    }
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
        cv::Mat SmoothedVEL = cv::Mat::zeros(1,3,CV_32FC1); 
        cv::Moments MOMt;
        double minValt_1, maxValt_1;
        cv::Point maxPost_1;
        cv::Mat Probt_1;
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

        for(int i = DELTA; i < Probt_1.rows-DELTA; ++i) {        //as  before  during segmentation -> Probt_1 will be a matrix of values of pVbilities between 0 and 1 
            for(int j = DELTA; j < Probt_1.cols-DELTA; ++j) {
                if(MAGt_1.at<float>(i,j)> TH1__)
                    Probt_1.at<float>(i,j) = ((float)(cv::sum(MAGt_1(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1)) >= TH2__).val[0]))/((float)(LATO*LATO));
            }
        }
      
            //Maskt_1 = Probt_1 >= PTH__;               

        //COND = cv::sum(Maskt_1).val[0] >= 0.05*(255*Ut.rows*Ut.cols);   //?rimane sempre a 0, quindi  ho  tolto l'if successivo  
        //if(COND) {
        //  yInfo("interno del  cond");
        //}

        //    devo trovare il centroide della roi
        //cv::Moments MOMt_1 = cv::moments(Maskt_1, true);
        //if (MOMt_1.m00 == 0) {
        //    MOMt_1.m00 = 0.000001;
        //}
        //maxPost_1.x = MOMt_1.m10/MOMt_1.m00;      //??
        //maxPost_1.y = MOMt_1.m01/MOMt_1.m00;

        // DESCRITTORE: Vt Ct Rt At
        cv::Mat Maskt8U;
        Maskt.convertTo(Maskt8U, CV_8UC1);
        VEL.at<float>(0,0) = cv::mean(Ut, Maskt8U).val[0];   //crasha
        VEL.at<float>(0,1) = cv::mean(Vt, Maskt8U).val[0]; 
        VEL.at<float>(0,2) = 0.033;  //10 fps  (1/fps=1/10=0.1;  1/30=0.0333333)

        float V =  sqrt(VEL.at<float>(0,0) * VEL.at<float>(0,0) + VEL.at<float>(0,1) * VEL.at<float>(0,1) + VEL.at<float>(0,2)*VEL.at<float>(0,2))  ;

        //if (V!=float(0.033))  {
        descr.push_back(sequenceID);
        timeCounter++;
        descr.push_back(timeCounter);

        //float gaussian [21]= {0.00441960718074799,	0.00800287473499368,	0.0136133485037391, 	0.0217540689675673, 	0.0326567253937836, 	0.0460533643657333, 	0.0610107951999105, 	0.0759291614170878, 	0.0887701832148991, 	0.0974949731273269, 	0.100589795788422,  	0.0974949731273269, 	0.0887701832148991, 	0.0759291614170878, 	0.0610107951999105, 	0.0460533643657333, 	0.0326567253937836, 	0.0217540689675673, 	0.0136133485037391,   	0.00800287473499368, 	0.00441960718074799};
        std::vector<float> gaussian ;
        gaussian.push_back( 0.0110020044943488);
        gaussian.push_back( 0.0431751450217583);
        gaussian.push_back( 0.114643519437383);
        gaussian.push_back( 0.205977096991566);
        gaussian.push_back( 0.350404468109888);
        gaussian.push_back( 0.205977096991566);
        gaussian.push_back( 0.114643519437383);
        gaussian.push_back( 0.0431751450217583);
        gaussian.push_back( 0.0110020044943488);

        /*float gaussian [9] ={ 0.0110020044943488,
                            0.0431751450217583,
                            0.114643519437383,
                            0.205977096991566,
                            0.350404468109888,
                            0.205977096991566,
                            0.114643519437383,
                            0.0431751450217583,
                            0.0110020044943488  };
        */
        int kernelSize=9;

     //  //normalizing the filtered signal
     //if(sequenceID==1 || sequenceID==3){
     //                Vmin=0.0395140000000000;
     //                Vmax=2.97941200000000;
     //                Cmin=0.00109100000000000;
     //                Cmax=466.055145000000;
     //                Rmin=0.00214600000000000;
     //                Rmax=916.988098000000;
     //                Amin=0.00184800000000000;
     //                Amax=18.4157330000000;   }

     //if (sequenceID==2 || sequenceID==4)   {
     //                Vmin=0.701857000000000;
     //                Vmax=2.81591400000000;
     //                Cmin=0.00494500000000000;
     //                Cmax=0.682409000000000;
     //                Rmin=1.46539700000000;
     //                Rmax=202.235123000000;
     //                Amin=0.00358000000000000;
     //                Amax=0.774497000000000;
     //}

        //cv::imshow("Ut",Ut)  ;
        //cv::imshow("Ut_1",Ut_1)  ;
        //cv::imshow("Maskt",Maskt)  ;
        //cv::imshow("Maskt_1",Maskt_1)  ;
        //cv::waitKey(10);

        cv::Mat Maskt_18U;
        Maskt_1.convertTo(Maskt_18U, CV_8UC1);

        ////in case you would like to find the other 3 features from the not smoothed velocity and THEN smooth them
        //Compute other features from not smoothed Vx and Vy

        ACC.at<float>(0,0) =  cv::mean(Ut, Maskt8U).val[0] - cv::mean(Ut_1, Maskt_18U).val[0]; 
        ACC.at<float>(0,1) = cv::mean(Vt, Maskt8U).val[0] - cv::mean(Vt_1, Maskt_18U).val[0]; 
        ACC.at<float>(0,2) = 0.;
	
        float C = cv::norm(VEL.cross(ACC))/pow(cv::norm(VEL),3);       //C=Curvature
        float R=1/C;                                                   //R=Radius of curvature
        float A=V/R;                                                   //A=V/R

        //Smoothing the features
       int bufferSize=9;

       buffering(bufferSize, bufferV, V)   ;                          //create the buffer of data for convolution  with kernel
       if (bufferV.size()==bufferSize) 
            convolution(bufferV, gaussian, smoothedBufferV, currentSmoothedV);

        buffering(bufferSize, bufferC, C)   ;
        if (bufferC.size()==bufferSize) 
            convolution(bufferC, gaussian, smoothedBufferC, currentSmoothedC);

        buffering(bufferSize, bufferR, R)   ;
        if (bufferR.size()==bufferSize) 
            convolution(bufferR, gaussian, smoothedBufferR, currentSmoothedR);

        buffering(bufferSize, bufferA, A)   ;
        if (bufferA.size()==bufferSize) 
            convolution(bufferA, gaussian, smoothedBufferA, currentSmoothedA);


         if (currentSmoothedV < Vmin)
            Vmin=currentSmoothedV;
        if (currentSmoothedV > Vmax)
            Vmax=currentSmoothedV;

        if (currentSmoothedC < Cmin)
            Cmin=currentSmoothedC;
        if (currentSmoothedC > Cmax)
            Cmax=currentSmoothedC;

        if (currentSmoothedR < Rmin)
            Rmin=currentSmoothedR;
        if (currentSmoothedR > Rmax)
            Rmax=currentSmoothedR;

        if (currentSmoothedA < Amin)
            Amin=currentSmoothedA;
        if (currentSmoothedA > Amax)
            Amax=currentSmoothedA;


        //if(nFr<10){
        //    Vmin=100.0;
        //    Vmax=0.000001;
        //    Cmin=100.0;
        //    Cmax=0.000001;
        //    Rmin=100.0;
        //    Rmax=0.000001;
        //    Amin=100.0;
        //    Amax=0.000001;
        //}


       float currentNormalizedSmoothedV=(currentSmoothedV-Vmin)/(Vmax-Vmin+0.000001);      //+0.000001 because at the beginning Vmax=Vmin=currentSmoothedV
       float currentNormalizedSmoothedC=(currentSmoothedC-Cmin)/(Cmax-Cmin+0.000001);
       float currentNormalizedSmoothedR=(currentSmoothedR-Rmin)/(Rmax-Rmin+0.000001);
       float currentNormalizedSmoothedA=(currentSmoothedA-Amin)/(Amax-Amin+0.000001);



       // ////in case you would like to smooth Vx and Vy and from them compute the other features
       // //Smooth Vx and Vy
       //int bufferSize=9;

       // float Vx = cv::mean(Ut, Maskt8U).val[0];   //crasha
       // float Vy = cv::mean(Vt, Maskt8U).val[0]; 
       // float Vz = 0.033;  //10 fps  (1/fps=1/10=0.1;  1/30=0.0333333)

       // float Vx_1 = cv::mean(Ut_1, Maskt8U).val[0];   //crasha
       // float Vy_1 = cv::mean(Vt_1, Maskt8U).val[0]; 
       // float Vz_1 = 0.033;  //10 fps  (1/fps=1/10=0.1;  1/30=0.0333333)

       //buffering(bufferSize, bufferVx, Vx)   ;                          //create the buffer of data for convolution  with kernel
       //if (bufferVx.size()==bufferSize) 
       //     convolution(bufferVx, gaussian, smoothedBufferVx, currentSmoothedVx);

       // buffering(bufferSize, bufferVy, Vy)   ;                          //create the buffer of data for convolution  with kernel
       //if (bufferVy.size()==bufferSize) 
       //     convolution(bufferVy, gaussian, smoothedBufferVy, currentSmoothedVy);

       // buffering(bufferSize, bufferVx_1, Vx_1)   ;                          //create the buffer of data for convolution  with kernel
       //if (bufferVx_1.size()==bufferSize) 
       //     convolution(bufferVx_1, gaussian, smoothedBufferVx_1, currentSmoothedVx_1);

       // buffering(bufferSize, bufferVy_1, Vy_1)   ;                          //create the buffer of data for convolution  with kernel
       //if (bufferVy_1.size()==bufferSize) 
       //     convolution(bufferVy_1, gaussian, smoothedBufferVy_1, currentSmoothedVy_1);

       // SmoothedVEL.at<float>(0,0) = currentSmoothedVx;
       // SmoothedVEL.at<float>(0,1) = currentSmoothedVy;
       // SmoothedVEL.at<float>(0,2) = Vz;

       // ACC.at<float>(0,0) =  currentSmoothedVx - currentSmoothedVx_1; 
       // ACC.at<float>(0,1) =  currentSmoothedVx - currentSmoothedVx_1;
       // ACC.at<float>(0,2) = 0.;
       // 
       // float V =  sqrt(SmoothedVEL.at<float>(0,0) * SmoothedVEL.at<float>(0,0) + SmoothedVEL.at<float>(0,1) * SmoothedVEL.at<float>(0,1) + SmoothedVEL.at<float>(0,2)*SmoothedVEL.at<float>(0,2))  ;
       // float C = cv::norm(SmoothedVEL.cross(ACC))/pow(cv::norm(SmoothedVEL),3);       //C=Curvature
       // float R=1/C;                                                   //R=Radius of curvature
       // float A=V/R;                                                   //A=V/R


       // float currentNormalizedSmoothedV=(currentSmoothedV-Vmin)/(Vmax-Vmin+0.000001);      //+0.000001 because at the beginning Vmax=Vmin=currentSmoothedV
       // float currentNormalizedSmoothedC=(currentSmoothedC-Cmin)/(Cmax-Cmin+0.000001);
       // float currentNormalizedSmoothedR=(currentSmoothedR-Rmin)/(Rmax-Rmin+0.000001);
       // float currentNormalizedSmoothedA=(currentSmoothedA-Amin)/(Amax-Amin+0.000001);


        descr.push_back(V);                                              //V=is the norm of the velocity (mean of optical flow)
        descr.push_back(C);   //C=Curvature
        descr.push_back(R);                                      //R=Radius of curvature
        descr.push_back(A);                               //A=V/R

	    descr.push_back(currentSmoothedV);
        descr.push_back(currentSmoothedC);
        descr.push_back(currentSmoothedR);
        descr.push_back(currentSmoothedA);
        descr.push_back(currentNormalizedSmoothedV);
        descr.push_back(currentNormalizedSmoothedC);
        descr.push_back(currentNormalizedSmoothedR);
        descr.push_back(currentNormalizedSmoothedA);

        descr.push_back(VEL.at<float>(0,0));
        descr.push_back(VEL.at<float>(0,1));

        FILE *fid = fopen("provaalvolo.txt","a" );
        //fprintf(fid, "%lf\n", maxValM);
        fprintf(fid, "%lf\n", cv::norm(VEL));
        fclose(fid);


        descr.push_back(maxPost.x);
        descr.push_back(maxPost.y);


        //std::cout  << "  Ut Vt  " <<  cv::mean(Ut, Maskt8U).val[0] << " " <<  cv::mean(Vt, Maskt8U).val[0] << " "  << std::endl;  

        //std::cout  << "Descriptor    " <<  descr[0]  << " " << descr[1]  << " " << descr[2] << " " << descr[3] << " " << descr[4] << " " << descr[5]  << std::endl;  
        //std::cout  << "bufferV    " <<  bufferV   << std::endl;  

        computed = 1;   

        contentBottle = descrBottle.addList();
        //if(nFr>30){
            contentBottle.addFloat32(descr[0]);
            contentBottle.addFloat32(descr[1]);
            //contentBottle.addFloat32(descr[2]);
            //contentBottle.addFloat32(descr[3]);
            //contentBottle.addFloat32(descr[4]);
            //contentBottle.addFloat32(descr[5]);
            //contentBottle.addFloat32(descr[6]);               //filtered V:  to be sent to the Carlo's module
            //contentBottle.addFloat32(descr[7]);               //filtered C:  to be sent to the Carlo's module
            //contentBottle.addFloat32(descr[8]);               //filtered R:  to be sent to the Carlo's module
            ///contentBottle.addFloat32(descr[9]);               //filtered A:  to be sent to the Carlo's module
            //contentBottle.addFloat32(descr[10]);
            //contentBottle.addFloat32(descr[11]);
            //contentBottle.addFloat32(descr[12]);
            //contentBottle.addFloat32(descr[13]);

            contentBottle.addFloat32(descr[14]);   //Vx:  to be sent to a dumper for analysis in batch
            contentBottle.addFloat32(descr[15]);   //Vy:  to be sent to a dumper for analysis in batch
        //}
        //printf("bottle is %s\n", contentBottle.toString().c_str());

        //  } else {
        //computed = 0;
        //  }

        Ut_1=Ut.clone();
        Vt_1=Vt.clone();
        Maskt_1=Maskt.clone();
        int e=Ut_1.depth();
        int f=Ut.depth();


        sem.wait();
        featDataready=false;
        sem.post();

        //}//if V!=0.033 
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
