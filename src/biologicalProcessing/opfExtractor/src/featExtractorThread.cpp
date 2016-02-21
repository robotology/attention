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
#include <limits>

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
    outputPortPlot.open      (getName("/plotResult:o").c_str());      //ddd

    //initialisig flag
    featDataready      = false;

    // initialising images

    //plotImage      = new ImageOf<PixelRgb>;
    //plotImage->resize(160,180);

    memoryPlot      = new ImageOf<PixelRgb>;
    memoryPlot->resize(1600,1800);

    //uImage      = new ImageOf<PixelMono>;
    //uImage->resize(width,height);

    //vImage      = new ImageOf<PixelMono>;
    //vImage->resize(width,height);

    //mImage      = new ImageOf<PixelMono>;
    //mImage->resize(width,height);

    Ut_1 = cv::Mat::zeros(height, width, CV_32FC1);  //I have already use U and V to put them in U_1 and V_1  //?or is it better to initialize U, V, U_1, V_1 in the threadInit?
    Vt_1 = cv::Mat::zeros(height, width, CV_32FC1);
    Maskt_1 = cv::Mat::zeros(height, width, CV_32FC1);

    Plot = cv::Mat::zeros(1800, 1600, CV_32FC3);

    currentSmoothedV = 0.0;
    currentSmoothedC = 0.0;
    currentSmoothedR = 0.0;
    currentSmoothedA = 0.0;

    Vmin=100.0;
    Vmax=0.0;
    Cmin=100.0;
    Cmax=0.0;
    Rmin=100.0;
    Rmax=0.0;
    Amin=100.0;
    Amax=0.0;

    counter=0;


    printf("initialization in feature  extractor thread correctly ended \n");
    return true;
}

void featExtractorThread::interrupt() {
    outputPortDescr.interrupt();      //ddd
    outputPortPlot.interrupt();
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
    Ut=U; 
    Vt=V;
    Maskt=M;
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

/*featExtractorThread takes U thresholded,V thresholded and the mask from opfExtractorThread, and from them it computes the features*/
void featExtractorThread::run() {

    if(outputPortDescr.getOutputCount()) {
        if(contentBottle.size()!=0.0 && contentBottle.get(0).asDouble() !=0.0 ){
            //yDebug("plotting the image");
            Bottle& outbot = outputPortDescr.prepare();
            outbot.clear();
            outbot = contentBottle;//descrBottle
            outputPortDescr.write();
        }
    }


    if(outputPortDescr.getOutputCount() || outputPortPlot.getOutputCount()) {
        //yDebug("plotting the image");
        ImageOf<PixelRgb>& imagePlot = outputPortPlot.prepare();
        imagePlot.resize(1600, 1800);
        imagePlot.zero();

        bool tempReady;
        sem.wait();
        tempReady = featDataready;          //this is to protect featDataready
        sem.post();

        if(tempReady){
            descr.clear();
            cv::Mat VEL = cv::Mat::zeros(1,3,CV_32FC1); 
            cv::Mat ACC = cv::Mat::zeros(1,3,CV_32FC1);
            cv::Mat SmoothedVEL = cv::Mat::zeros(1,3,CV_32FC1); 
            cv::Moments MOMt;
            cv::Mat mean, std;

            cv::Mat MatV = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
            cv::Mat MatC = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
            cv::Mat MatR = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
            cv::Mat MatA = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
            int delta = 3;

            //togliere commenti e decidere dove settare il case, per ora c'e' solo quella del centroide
            //switch(flag_) {
            //case 0: // analisi del centroide  

            //to find the centroid of the roi=region of interest
            MOMt = cv::moments(Maskt, true);    //moments(Calculates all of the moments up to the third order of a polygon or rasterized shape.)
            cv::Point maxPost;
            if (MOMt.m00 == 0)
                MOMt.m00 = 0.000001;
            maxPost.x = MOMt.m10/MOMt.m00;
            maxPost.y = MOMt.m01/MOMt.m00;

            // DESCRITTORE: Vt Ct Rt At
            cv::Mat Maskt8U;
            Maskt.convertTo(Maskt8U, CV_8UC1);
            Maskt_1.convertTo(Maskt_18U, CV_8UC1);
            //cv::imshow("Maskt",Maskt)  ;
            //cv::imshow("Maskt_1",Maskt_1)  ;
            //cv::waitKey(10);

            VEL.at<float>(0,0) = cv::mean(Ut, Maskt8U).val[0];
            VEL.at<float>(0,1) = cv::mean(Vt, Maskt8U).val[0]; 
            VEL.at<float>(0,2) = 0.0666;  //15 fps

            float V =  sqrt(VEL.at<float>(0,0) * VEL.at<float>(0,0) + VEL.at<float>(0,1) * VEL.at<float>(0,1) + VEL.at<float>(0,2)*VEL.at<float>(0,2))  ;

            ////in case you would like to find the other 3 features from the not smoothed velocity and THEN smooth them
            //Compute other features from not smoothed Vx and Vy
            ACC.at<float>(0,0) =  cv::mean(Ut, Maskt8U).val[0] - cv::mean(Ut_1, Maskt_18U).val[0]; 
            ACC.at<float>(0,1) = cv::mean(Vt, Maskt8U).val[0] - cv::mean(Vt_1, Maskt_18U).val[0]; 
            ACC.at<float>(0,2) = 0.;

            float C = cv::norm(VEL.cross(ACC))/pow(cv::norm(VEL),3);       //C=Curvature
            float R=1/C;                                                   //R=Radius of curvature
            float A=V/R;                                                   //A=V/R

            if (    (V!=VEL.at<float>(0,2))  && C!=0.0 ){ 
                //Smoothing the features
                //gaussian filter to be convoluted with the signal to filter it
                std::vector<float> gaussian ;
                gaussian.push_back( 0.0110020044943488);
                gaussian.push_back( 0.0431751450217583);
                gaussian.push_back( 0.114643519437383);
                gaussian.push_back( 0.205977096991566);
                gaussian.push_back( 0.250404468109888);
                gaussian.push_back( 0.205977096991566);
                gaussian.push_back( 0.114643519437383);
                gaussian.push_back( 0.0431751450217583);
                gaussian.push_back( 0.0110020044943488);
                int kernelSize=9;
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



                float currentNormalizedSmoothedV=(currentSmoothedV-Vmin)/(Vmax-Vmin+0.000001);      //+0.000001 because at the beginning Vmax=Vmin=currentSmoothedV
                float currentNormalizedSmoothedC=(currentSmoothedC-Cmin)/(Cmax-Cmin+0.000001);
                float currentNormalizedSmoothedR=(currentSmoothedR-Rmin)/(Rmax-Rmin+0.000001);
                float currentNormalizedSmoothedA=(currentSmoothedA-Amin)/(Amax-Amin+0.000001);

                //put the information in the bottle
                descr.push_back(sequenceID);
                counter++;
                descr.push_back(counter);

                descr.push_back(V);     //V=is the norm of the velocity (mean of optical flow)
                descr.push_back(C);     //C=Curvature
                descr.push_back(R);     //R=Radius of curvature
                descr.push_back(A);     //A=V/R

                descr.push_back(currentSmoothedV);
                descr.push_back(currentSmoothedC);
                descr.push_back(currentSmoothedR);
                descr.push_back(currentSmoothedA);
                descr.push_back(currentNormalizedSmoothedV);
                descr.push_back(currentNormalizedSmoothedC);
                descr.push_back(currentNormalizedSmoothedR);
                descr.push_back(currentNormalizedSmoothedA);

                descr.push_back(maxPost.x);
                descr.push_back(maxPost.y);


                int rf = log (A)*100 +1200 ;
                int cf = log (C)*100 + 1000;

                ////for a  black  and white graph
                //if (cf%3==0)
                //    cf=cf;
                //else if ((cf+1)%3==0)
                //    cf=cf+1;
                //else if ((cf+2)%3==0)
                //    cf=cf+2;
                //Plot.at<float>(rf,cf)=255;
                //Plot.at<float>(rf,cf+1)=255;
                //Plot.at<float>(rf,cf+2)=255;
                //uchar *MatrixPointer = Plot.data;
                //for(int r = 0; r < 180; ++r) {
                //    for(int c = 0; c < 160; ++c) {
                //        if (Plot.at<float>(r,c) > 0)    //if it is 0,  it remains 0
                //           Plot.at<float>(r,c) --;
                //       if (r==int(0.667*(c-100)+120))
                //          Plot.at<float>(r,c)=255;
                //    }
                //}

                pPlot = imagePlot.getRawImage();
                pMem =  memoryPlot->getRawImage();

                int totRow =  imagePlot.height();
                int totCol =  imagePlot.width();
                int nChannel = imagePlot.getPixelSize();
                int padding  = imagePlot.getPadding();

                for(int r = 0; r < totRow; ++r) {
                    for(int c = 0; c < totCol; ++c) {

                        // 1.  writing information about new feature
                        if((r==totRow -rf) && (c == cf)) {
                            *pMem = 255; 
                            bigPixel(pMem,8,255);

                            pMem++;
                            *pMem = 255;
                            bigPixel(pMem,8,255);
                            pMem++;
                            *pMem = 255;
                            bigPixel(pMem,8,255);
                            pMem++;
                        }
                        else if((*pMem)>0)  {
                            *pMem=*pMem-3;
                            pMem++;
                            *pMem=*pMem-3;
                            pMem++;
                            *pMem=*pMem-3;
                            pMem++;
                        }
                        else {
                            pMem  += 3;
                        }
                        pMem -=3;

                        //2. copying
                        if (r==totRow-int(0.667*(c-1000)+1200)) {          //two-thirds line
                            *pPlot = 255;
                            bigPixel(pPlot, 8, 255);
                            pPlot++; pMem++;
                            *pPlot = 0;
                            bigPixel(pPlot,8,0);
                            pPlot++; pMem++;
                            *pPlot = 0;
                            bigPixel(pPlot,8,0);
                            pPlot++; pMem++;

                        }
                        else {                              //copying
                            *pPlot = *pMem;
                            pPlot++; pMem++;
                            *pPlot = *pMem;

                            pPlot++; pMem++;
                            *pPlot = *pMem;

                            pPlot++; pMem++;
                        }
                    }
                    pPlot += padding;
                    pMem  += padding;
                }

                outputPortPlot.write();
        

                //for(int r = 0; r < 180; ++r) {
                //    for(int c = 0; c < 160; ++c) {
                //        //if (Plot.at<float>(r,c) > 0)    //if it is 0,  it remains 0
                //           //Plot.at<float>(r,c) --;
                //       //if (r==int(0.667*(c-100)+120)){
                //          MatrixPointer[0]=255;
                //          MatrixPointer[1]=0;
                //          MatrixPointer[2]=0;
                //          MatrixPointer += 3;
                //       //}
                //    }
                //} //*/

                //std::cout  << "  Ut Vt  " <<  cv::mean(Ut, Maskt8U).val[0] << " " <<  cv::mean(Vt, Maskt8U).val[0] << " "  << std::endl;  
                //std::cout  << "Descriptor    " <<  descr[0] << " " << descr[1] << " " << descr[2] << " " << descr[3]  << std::endl;  

                computed = 1;   

                contentBottle = descrBottle.addList();


                //contentBottle.addDouble(descr[0]);
                //contentBottle.addDouble(descr[1]);           
                //contentBottle.addDouble(descr[2]);           
                //contentBottle.addDouble(descr[3]);           
                //contentBottle.addDouble(descr[4]);           
                //contentBottle.addDouble(descr[5]);          
                contentBottle.addDouble(descr[6]);     //mando i smoothed a modulo class
                contentBottle.addDouble(descr[7]);
                contentBottle.addDouble(descr[8]);
                contentBottle.addDouble(descr[9]);
                //contentBottle.addDouble(descr[10]);
                //contentBottle.addDouble(descr[11]);
                //contentBottle.addDouble(descr[12]);
                //contentBottle.addDouble(descr[13]);

                //printf("bottle is %s\n", contentBottle.toString().c_str());

                //  } else {
                //computed = 0;
                //  }

                Ut_1=Ut.clone();
                Vt_1=Vt.clone();
                Maskt_1=Maskt.clone();

                sem.wait();
                featDataready=false;
                sem.post();
            }//end if (V!=float(0.033))
        }
    }
}
 ///
void featExtractorThread::bigPixel(unsigned char* p, int mult,  int color) {    //for drowing a "*"
    for (int i=1; i<=mult; i++)
    {
    *(p-3*i)=color;
    *(p+3*i)=color;
    *(p-1600*3*i)=color;
    *(p+1600*3*i)=color;

    *(p-3*i-1600*3*i)=color;
    *(p-3*i+1600*3*i)=color;
    *(p+3*i-1600*3*i)=color;
    *(p+3*i+1600*3*i)=color;



    }
}


void featExtractorThread::threadRelease() {
    printf("featExtractorThread: portClosing \n");  
    outputPortDescr.close();   //ddd
    outputPortPlot.close();
    printf("freeing memory \n");

    // free allocated memory here please
    delete plotImage;
    //delete vImage;
    //delete mImage;

    printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ----------------
