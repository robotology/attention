// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*                                                                                                                                  
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file opfExtractorThread.cpp
 * @brief Implementation of the eventDriven thread (see opfExtractorThread.h).
 */

#include <iCub/opfExtractorThread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;

opfExtractorThread::opfExtractorThread() {
    robot = "icub";        
}

opfExtractorThread::opfExtractorThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

opfExtractorThread::~opfExtractorThread() {
    // do nothing
}

bool opfExtractorThread::threadInit() {
    // initialization of the attributes
    width  = WIDTH;
    height = HEIGHT;
    idle   = false;
    throwAway = false;
    firstProcessing=true;
    numberProcessing=0;
    setAlgorithm(ALGO_FB);

    countDescr=0;

    inputImage = new ImageOf<PixelRgb>();
    inputImage->resize(width, height);

    processingImage = new ImageOf<PixelRgb>();
    processingImage->resize(width, height);

    // opening the port for direct input
    if (!inputPort.open(getName("/image:i").c_str())) {
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }  

    // opening the port for direct output
    if (!outputPort.open(getName("/result:o").c_str())) {
        yError("unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }   

    //starting the plotterThread      bbb
    pt = new plotterThread();
    pt->setName(getName("").c_str());
    pt->start();

    //starting the featExtractorThread        aaa
    fet = new featExtractorThread();
    fet->setName(getName("").c_str());
    fet->start();

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void opfExtractorThread::setName(string str) {
    this->name=str;
}


std::string opfExtractorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void opfExtractorThread::setInputPortName(string InpPort) {
    
}

void opfExtractorThread::visualizationResume(){
    pt->resume();
}

void opfExtractorThread::visualizationSuspend(){
    pt->suspend();
}

bool opfExtractorThread::test(){
    bool reply = true;

    if(pt==NULL){
        reply = false;
    }
    else {
        reply &= pt->test();
    }

    reply  &= Network::exists(getName("/image:i").c_str());

    return reply;
}

void opfExtractorThread::run() {    
    while (isStopping() != true) {
        bool result;

        if(!idle){
            if(inputPort.getInputCount()) {
                inputImage = inputPort.read(true);   //blocking reading for synchr with the input

                if (throwAway){
                    throwAway = false;
                }
                else {
                    double timeStart = Time::now();
                    //processing
                    result = processing();               //generates the outputImage which is what we want to plot
                    double timeStopProcessing = Time::now();
                    double timeDiffProcessing =timeStopProcessing-timeStart;
                    yDebug("timeDiff %f", timeDiffProcessing);


                    //bbb
                    pt->copyImage(processingImage);
                    pt->copyU(U);                       //I have instantiated an  object p of type plotterThread, and now I can call the function of this class (copyU)
                    pt->copyV(V);
                    pt->copyM(Maskt);

                    fet->copyAll(U,V,Maskt);    //aaa
                    fet->setFlag();             //aaa

                    /*
                    if (outputPort.getOutputCount()) {
                        //yDebug("debug");
                        ImageOf<PixelMono> &c = outputPort.prepare();
                        c.resize(320,240);
                        c.copy(*outputImage);
                        outputPort.write();
                        //yDebug("debug2");
                    }
                    */
                    double timeStop = Time::now();
                    double timeDiff =timeStop-timeStart;
                    //yDebug("timeDiff %f", timeDiff);

                    throwAway = true;
                }
            }
            else {
                result = 0;
            }
        }
    }
}


void opfExtractorThread::motionToColor(cv::Mat U, cv::Mat V, cv::Mat& colorcodeMatrix){
    double minval_x, maxval_x, minval_y, maxval_y;
    cv::Point  minLoc_x, maxLoc_x, minLoc_y, maxLoc_y;
    cv::minMaxLoc(U,&minval_x, &maxval_x, &minLoc_x, &maxLoc_x);
    std::cout  << "maxval U     "<< maxval_x << std::endl;
    cv::minMaxLoc(V,&minval_y, &maxval_y, &minLoc_y, &maxLoc_y);
    std::cout  << "maxval V     "<< maxval_y << std::endl;

    uchar *ccMatrixPointer = colorcodeMatrix.data;                  //data is pointing to the Red of the 1st pixel

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float fx = U.at<float>(y, x);//motim.Pixel(x, y, 0);
            float fy = V.at<float>(y, x);//motim.Pixel(x, y, 1);
            //.Pixel(x, y, 0);
            float fxnorm, fynorm;

            if (maxval_x == minval_x)
                fxnorm=0.00000001;
            else
                fxnorm = (fx-minval_x)/(maxval_x-minval_x);
            if (maxval_y == minval_y)
                fynorm=0.00000001;
            else
                fynorm = (fy-minval_y)/(maxval_y-minval_y); 

            assert(fxnorm >=0); assert(fxnorm <= 1.00);
            assert(fynorm >= 0); assert(fynorm <= 1.00);
            
            //computeColor(fxnorm, fynorm, ccMatrixPointer);

            computeColor(fx, fy, ccMatrixPointer);   //fx and fy are not normalized, therefore they are not between 0 and 1
            ccMatrixPointer += 3;
        }
    }
}


void opfExtractorThread::fakethresholding(cv::Mat& U, cv::Mat& V){
    printf("faketh start \n");
    U = cv::Mat::zeros(U.rows, U.cols, CV_32FC1);
    V = cv::Mat::zeros(V.rows, V.cols, CV_32FC1);
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {         
                U.at<float>(y, x)=0.00010;
                V.at<float>(y, x)=-1.0;
        }
    }
    printf("faketh end \n");
}


void opfExtractorThread::thresholding(cv::Mat& Ut, cv::Mat& Vt, cv::Mat& maskThresholding){
    cv::Mat MAGt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat THETAt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);

    // from u-v to Magnitude-Theta
    double minval, maxval;
    cv::Point  minLoc, maxLoc;
    cv::minMaxLoc(Ut,&minval, &maxval, &minLoc, &maxLoc);
    //std::cout  << minval << "Ut " << maxval << std::endl;
    cv::cartToPolar(Ut, Vt, MAGt, THETAt, false);
  
    cv::Mat Probt = cv::Mat::zeros(MAGt.rows, MAGt.cols, CV_32FC1);
    int DELTA = 10;
    int LATO = DELTA*2+1;
    for(int i = DELTA; i < Probt.rows-DELTA; ++i) {
        for(int j = DELTA; j < Probt.cols-DELTA; ++j) {
            //std::cout <<  MAGt.at<float>(i,j) << " " << TH1_ << " " << std::endl;  
            if(MAGt.at<float>(i,j)> TH1_ ) {
                cv::Mat Q = MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1));
                cv::Mat MQ = Q >= TH2_;	   //>= returns a map of 0 and 255 instead of 1
                MQ = MQ/255.;
                Probt.at<float>(i,j) = cv::sum(MQ).val[0]/((float)(LATO*LATO));		  // divide by lato*lato in such a way to have 1 as maximum
                if(Probt.at<float>(i,j) >= PTH_) {
                    Maskt.at<float>(i,j) = 1.0;
                }
                else {
                    Maskt.at<float>(i,j) =  0.0;
                } 
                //Maskt.at<float>(i,j) = (Probt.at<float>(i,j) >= PTH_)/255.;
                //Probt.at<float>(i,j) = ((float)(cv::sum(MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1)) >= TH2_).val[0]))/((float)(LATO*LATO));
        
                Q.release();
                MQ.release();
            }
            else {
                if(Probt.at<float>(i,j) >= PTH_) {
                    Maskt.at<float>(i,j) = 1;  //ccc     1.0;
                }
                else {
                    Maskt.at<float>(i,j) =  0;  ///ccc 0.0
                }
            }
        }
    }


    //double minValt, maxValt;
    //cv::Point maxPost;
    //cv::minMaxLoc(MAGt, &minValt, &maxValt, NULL, &maxPost);

    //cv::Mat Maskt = MAGt >= TH_; // identificazione della roi
    //Maskt = (Probt >= PTH_)/255.;

    bool COND = cv::sum(Maskt).val[0] >= 0.01*(Ut.rows*Ut.cols);	//global statistics: we consider just what has a movement greater than the 1%  of  the total of the image
    //bool COND = cv::sum(Maskt).val[0] >= 0.01*(255); 
    //   std::cout << cv::sum(Maskt).val[0] << " " << 0.1*(Ut.rows*Ut.cols) << " " << COND << std::endl;


    /*
    if(!COND) {  
        //if(DEBUG_) {
        //  cv::Mat Im = cv::Mat::zeros(Ut.rows, Ut.cols, CV_8UC1);
        //  cv::imshow("DEBUG", Im);
        //  cv::waitKey(10);
        //  Im.release();
        //  
        //}
        MAGt.release();
        THETAt.release();
        MAGt_1.release();
        THETAt_1.release();
        Maskt.release();
        //computed = 0;                                   
        //return;               // REMEMBER: this makes it exit from the function
    }
    */
    
    
    //identify the connected components and visualize them
    //yInfo("identifying the connect components");
    std::vector<std::vector<cv::Point> > contours;

    //cv::Mat MaskClone = Maskt.clone();
    /*
            for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                cout <<  MaskClone.at<float>(y, x) <<  " " ;
        }
     }
     */
    cv::Mat helpframe;
    Maskt.convertTo(helpframe, CV_8U);
    
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(helpframe, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);	//MaskClone because findContours admit to write on the input
    //MaskClone.release();
  
    
    //yInfo("iterating the pixels");
    for(std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ) {						 //contours.begin() is the pointer to the initial element
        if(cv::contourArea(*it)/(Maskt.rows*Maskt.cols)>0.005)  //??? why 0.005
            ++it; //we are at the level of each bounding box,then the threshold is less
        else
            it = contours.erase(it);
    }

    
    //if(DEBUG_) {
    //  cv::Mat Im = cv::Mat::zeros(Probt.rows, Probt.cols, CV_8UC1);
    //  Probt_norm.convertTo(Im, CV_8UC1);
    //  cv::Mat Im3 = cv::Mat::zeros(Probt.rows, Probt.cols, CV_8UC3);
    //  cv::cvtColor(Im, Im3, CV_GRAY2RGB);
    //  cv::drawContours(Im3, contours, -1, CV_RGB(255,255,0), -1);
    //  cv::imshow("MAP", Im3);
    //  cv::waitKey(10);
    //  Im.release();
    //  
    //}

        /*for (int y = DELTA; y < height; y++) {
            for (int x = DELTA; x < width; x++) {
                cout <<  Maskt.at<float>(y, x) <<  " " ;
        }
     }*/
    //yInfo("drawing contours");
    cv::drawContours(maskThresholding, contours, -1, CV_RGB(255,255,255), -1);			 //it puts zeros where the optical flow is low and 255 where the optical   flow is high             -1 vuol   dire che  devo  colarle tutti  i bounding box. l'altro -1 che devo colarli  pieno (controlla)    

    //yInfo("removing the UV in the mask");



    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (maskThresholding.at<float>(y, x)==0.0) {
                Ut.at<float>(y, x) = 0.000001;
                Vt.at<float>(y, x) = 0.000001;
                //Maskt.at<float>(y, x) = 0;      //uncomment if you want to plot with the plotterThread Maskt (with copyM)
            }
            else {
                //Maskt.at<float>(y, x) = 255;   //uncomment if you want to plot with the plotterThread Maskt (with copyM)
            }
        }
     }
}



void opfExtractorThread::thresholding(cv::Mat Ut_1, cv::Mat Vt_1, cv::Mat& Ut, cv::Mat& Vt, cv::Mat& maskThresholding, std::vector<float>& descr, int& computed){ 
    cv::Mat MAGt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat THETAt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat MAGt_1 = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat THETAt_1 = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);  

    int h=Ut_1.depth();
    int i=Ut.depth();
    // from u-v to Magnitude-Theta
    cv::cartToPolar(Ut, Vt, MAGt, THETAt, false);
  
    cv::Mat Probt = cv::Mat::zeros(MAGt.rows, MAGt.cols, CV_32FC1);
    int DELTA = 10;
    int LATO = DELTA*2+1;
    for(int i = DELTA; i < Probt.rows-DELTA; ++i) {
        for(int j = DELTA; j < Probt.cols-DELTA; ++j) {
            //std::cout <<  MAGt.at<float>(i,j) << " " << TH1_ << " " << std::endl;  
            if(MAGt.at<float>(i,j)> TH1_ ) {
                cv::Mat Q = MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1));
                cv::Mat MQ = Q >= TH2_;	   //>= returns a map of 0 and 255 instead of 1
                MQ = MQ/255.;
                Probt.at<float>(i,j) = cv::sum(MQ).val[0]/((float)(LATO*LATO));		  // divide by lato*lato in such a way to have 1 as maximum
                
                //Probt.at<float>(i,j) = ((float)(cv::sum(MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1)) >= TH2_).val[0]))/((float)(LATO*LATO));
        
                Q.release();
                MQ.release();
            }
        }
    }
    
    

    // double minValt, maxValt;
    // cv::Point maxPost;
    // cv::minMaxLoc(MAGt, &minValt, &maxValt, NULL, &maxPost);
  
    //   cv::Mat Maskt = MAGt >= TH_; // identificazione della roi
    cv::Mat Maskt = (Probt >= PTH_)/255.;
    int n=Maskt.depth();
    
    
    bool COND = cv::sum(Maskt).val[0] >= 0.01*(Ut.rows*Ut.cols);	//global statistics: we consider just what has a movement greater than the 1%  of  the total of the image
    //bool COND = cv::sum(Maskt).val[0] >= 0.01*(255); 
    //   std::cout << cv::sum(Maskt).val[0] << " " << 0.1*(Ut.rows*Ut.cols) << " " << COND << std::endl;

    
    /*
    if(!COND) {  
        //if(DEBUG_) {
        //  cv::Mat Im = cv::Mat::zeros(Ut.rows, Ut.cols, CV_8UC1);
        //  cv::imshow("DEBUG", Im);
        //  cv::waitKey(10);
        //  Im.release();
        //  
        //}
        MAGt.release();
        THETAt.release();
        MAGt_1.release();
        THETAt_1.release();
        Maskt.release();
        //computed = 0;                                   
        //return;               // REMEMBER: this makes it exit from the function
    }
    */
    
    
    // identify the connected components and visualize them
    //yInfo("identifying the connect components");
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat MaskClone = Maskt.clone();
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(MaskClone, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);	//MaskClone because findContours admit to write on the input
    MaskClone.release();
  
    
    //yInfo("iterating the pixels");
    for(std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ) {						 //contours.begin() is the pointer to the initial element
        if(cv::contourArea(*it)/(Maskt.rows*Maskt.cols)>0.005)  //??? why 0.005
            ++it; //we are at the level of each bounding box,then the threshold is less
        else
            it = contours.erase(it);
    }

    
    //if(DEBUG_) {
    //  cv::Mat Im = cv::Mat::zeros(Probt.rows, Probt.cols, CV_8UC1);
    //  Probt_norm.convertTo(Im, CV_8UC1);
    //  cv::Mat Im3 = cv::Mat::zeros(Probt.rows, Probt.cols, CV_8UC3);
    //  cv::cvtColor(Im, Im3, CV_GRAY2RGB);
    //  cv::drawContours(Im3, contours, -1, CV_RGB(255,255,0), -1);
    //  cv::imshow("MAP", Im3);
    //  cv::waitKey(10);
    //  Im.release();
    //  
    //}

    //yInfo("drawing contours");
    cv::drawContours(maskThresholding, contours, -1, CV_RGB(255,255,255), -1);			 //it puts zeros where the optical flow is low and 255 where the optical   flow is high             -1 vuol   dire che  devo  colarle tutti  i bounding box. l'altro -1 che devo colarli  pieno (controlla)    

    //yInfo("removing the UV in the mask");
    for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
            if (maskThresholding.at<float>(y, x)==0.0) {
                Ut.at<float>(y, x) = 0.000001;
                Vt.at<float>(y, x) = 0.000001;
            }             
		}
    }
//}


//void opfExtractorThread::computeFeatures(cv::Mat Ut_1, cv::Mat Vt_1, cv::Mat Ut, cv::Mat Vt, std::vector<float>& descr, int& computed){   //I eliminated CUM
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

    cv::Mat VEL = cv::Mat::zeros(1,3,CV_32FC1), ACC = cv::Mat::zeros(1,3,CV_32FC1);
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

    cv::cartToPolar(Ut_1, Vt_1, MAGt_1, THETAt_1, false);

    Probt_1 = cv::Mat::zeros(MAGt_1.rows, MAGt_1.cols, CV_32FC1);

      for(int i = DELTA; i < Probt_1.rows-DELTA; ++i) {        //as  before  during segmentation -> Probt_1 will be a matrix of values of probabilities between 0 and 1 
        for(int j = DELTA; j < Probt_1.cols-DELTA; ++j) {
            if(MAGt_1.at<float>(i,j)> TH1_)
                Probt_1.at<float>(i,j) = ((float)(cv::sum(MAGt_1(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1)) >= TH2_).val[0]))/((float)(LATO*LATO));
        }
      }
      
      Maskt_1 = Probt_1 >= PTH_;

    COND = cv::sum(Maskt_1).val[0] >= 0.05*(255*Ut.rows*Ut.cols);   //?rimane sempre a 0, quindi  ho  tolto l'if successivo  
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

    float u=Ut.depth();
    float v=Vt.depth();
    float z=Maskt.depth();
	
    //Maskt.convertTo(Maskt8U, CV_8U);
	VEL.at<float>(0,0) = cv::mean(Ut, Maskt).val[0]; 
	VEL.at<float>(0,1) = cv::mean(Vt, Maskt).val[0]; 
	VEL.at<float>(0,2) = 0.1;  //10 fps
	
    float roba =  sqrt(VEL.at<float>(0,0) * VEL.at<float>(0,0) + VEL.at<float>(0,1) * VEL.at<float>(0,1) + VEL.at<float>(0,2)*VEL.at<float>(0,2))  ;
	
    countDescr++;
    descr.push_back(countDescr);   
    descr.push_back(1);
    
    descr.push_back(roba);         //V  is the norm of

	ACC.at<float>(0,0) =  cv::mean(Ut, Maskt).val[0] - cv::mean(Ut_1, Maskt_1).val[0]; 
	ACC.at<float>(0,1) = cv::mean(Vt, Maskt).val[0] - cv::mean(Vt_1, Maskt_1).val[0]; 
	ACC.at<float>(0,2) = 0.;
	
	descr.push_back(cv::norm(VEL.cross(ACC))/pow(cv::norm(VEL),3));
	descr.push_back(1/descr[1]);
	descr.push_back(descr[0]/descr[2]);
	
	descr.push_back(maxPost.x);
	descr.push_back(maxPost.y);


    std::cout  << "  Ut Vt  " <<  cv::mean(Ut, Maskt).val[0] << " " <<  cv::mean(Vt, Maskt).val[0] << " "  << std::endl;  

    std::cout  << "Descriptor    " <<  descr[0] << " " << descr[1] << " " << descr[2] << " " << descr[3]  << std::endl;  
	computed = 1;

    Bottle b;
    b.add(descr[0]);           
    b.add(descr[1]);           
    b.add(descr[2]);           
    b.add(descr[3]);           


    //  } else {
	//computed = 0;
    //  }

}



bool opfExtractorThread::processing(){
	//yDebug("processing");
    numberProcessing++;
    //std::cout  << "numberProcessing    " << numberProcessing << " " << std::endl;
	cv::Mat Matrix((IplImage*) inputImage->getIplImage(), false);
    
	//cv::Mat outputMatrix((IplImage*) inputImage->getIplImage(), false);
	//cv::Mat currentMatrix;
	if(!firstProcessing) {
		previousMatrix=currentMatrix.clone();
	}

	cv::cvtColor(Matrix, currentMatrix, CV_RGB2GRAY);
	
	//--------------------------------------------------------------------------

	if(!firstProcessing){

		cv::Mat flow;
		//U = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);  //I have already use U and V to put them in U_1 and V_1  //?or is it better to initialize U, V, U_1, V_1 in the threadInit?
        //V = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);

        cv::Mat U_1 = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
        cv::Mat V_1 = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);

        cv::Mat maskThresholding = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);   //??prima CV_32FC1
		cv::Mat MV[2]={U,V};
        std::vector<float> descr;   //length=4
        int computed=0;

        if(numberProcessing>=3){
            U_1=U.clone();
            V_1=V.clone();                
        }

		switch (ofAlgo) {
		case ALGO_FB:{
                double timeStartOF = Time::now();
				cv::calcOpticalFlowFarneback(previousMatrix, currentMatrix, flow, 0.2, 3, 19, 10, 7, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
               // int  deppp=previousMatrix.depth();
                double timeStopOF = Time::now();
                double timeDiffOF =timeStopOF-timeStartOF;
                yDebug("timeDiffOF %f", timeDiffOF);
                //cv::calcOpticalFlowFarneback(previousMatrix, currentMatrix, flow, 0.2, 3, 19, 10, 5, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
				cv::split(flow, MV);
		}break;

		case ALGO_LK:{
				yDebug("just entered in  Lk");
				std::vector<cv::Point2f> prevPts;
				std::vector<cv::Point2f> currPts;
				cv::Size winSize(5,5); 

				int delta_=1;
				vector<uchar> status;
				vector<float> err;
				yDebug("before the cycle for");
				for(int x = 0; x < previousMatrix.cols; x=x+delta_)	{
					for(int y = 0; y < previousMatrix.rows; y=y+delta_)	{
						prevPts.push_back(cv::Point2f((float)x,(float)y));				//in prevPts there are the xs and the ys of the image
					}
				}
				cv::calcOpticalFlowPyrLK(previousMatrix, currentMatrix, prevPts, currPts, status, err, winSize, 3);
				
				for(int k = 0; k < prevPts.size(); k++)	{  //I is a vector and each  element is a point (therefore with two fields)
					//std::cout  << currPts.at(k).x << " "  << currPts.at(k).y << std::endl;
					if (status.at(k))	{
						U.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  currPts.at(k).x - prevPts.at(k).x;
						V.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  currPts.at(k).y - prevPts.at(k).y;
					}
				}
				
				//flow.create(previousMatrix.rows, previousMatrix.cols, CV_32FC2);			  //create(rows,cols,type)					   in teoria qui e nelle prox 2 righe ci  va I.rows,I.cols  (ma devi usare size)
  
      
				//int P = 0;
				//for(int k = 0; k < I.size(); k++)	  
				//	if (status.at(k))	{
				//		U.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  I.at(k).x;
				//		V.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  I.at(k).y;
				//	}
				//  
				// cv::Mat mv[2] = {U, V};
				//cv::merge(mv, 2, flow);
      
				 prevPts.clear();
	  			 currPts.clear();
				 status.clear();
				 err.clear();
		}break;																							
		}  // switch optical  flow


		/* computing colorcode */
		cv::Mat colorcodeMatrix =  cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_8UC3);
		//motionToColor(U, V, colorcodeMatrix);
		////colorcodeMatrix.convertTo(outputMatrix,CV_8UC3);

        /*taking a point (x,y) with a flow magnitude more than a threshold (th1) and then 
        computing the number of points around the point (x,y) with  the magnitude of the flow more than a threshold (th2) and taking the number of points with a flow magnitude more than a threshold(th3)*/
        //fakethresholding(U, V);


	    //?metto qui?
        if(numberProcessing>=3) {        //you can compute the acceleration when you have 3 frames
            double timeStartThresholding =Time::now();

            thresholding(U, V, maskThresholding);

            double timeStopThresholdind =Time::now();
            double timeDiffThresholding =timeStopThresholdind-timeStartThresholding;
            yDebug("timeDiffThresholding %f", timeDiffThresholding);

            //thresholding(U_1, V_1, U, V, maskThresholding, descr, computed);
            //U = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
            //V = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
            motionToColor(U, V, colorcodeMatrix);            
            //computeFeatures(U_1, V_1, U, V, descr, computed);          //U_1 is the U at time t-1, U is the U at the time t  (sicuri??)
        }

		/* computing min and max */	
		/*
		double minval, maxval;	
		cv::Point  minLoc, maxLoc;
		cv::minMaxLoc(U,&minval, &maxval, &minLoc, &maxLoc);
		std::cout  << minval << " " << maxval << std::endl;
		cv::minMaxLoc(V,&minval, &maxval, &minLoc, &maxLoc);
		std::cout  << minval << " " << maxval << std::endl;
		*/		
				

		/* of magnitude visualization */
		/*
		cv::Mat M, MN;
		cv::sqrt(U.mul(U)+V.mul(V),M);
		//double minval, maxval;
		//cv::Point  minLoc, maxLoc;
		cv::minMaxLoc(M,&minval, &maxval, &minLoc, &maxLoc);
		std::cout  << minval << " " << maxval << std::endl;
		if(maxval>0.0){
			MN=255*(M-(float)minval)/((float)maxval-(float)minval);
		}
		else { 
			MN=cv::Mat::zeros(U.rows,U.cols, CV_32FC1);
		}
		MN.convertTo(outputMatrix,CV_8UC1);
		*/

		//IplImage Ipl=(IplImage)outputMatrix;
		IplImage Ipl = (IplImage) colorcodeMatrix; 
		processingImage-> wrapIplImage(&Ipl);
        if (outputPort.getOutputCount()) {
            V.convertTo(outputMatrix,CV_8UC1);              //it is plotted just V
            IplImage tempIpl = (IplImage) outputMatrix;
            outputImage-> wrapIplImage(&tempIpl);
            //outputImage-> wrapIplImage(&((IplImage)(outputMatrix)));
        }

		flow.release();
        maskThresholding.release();
		colorcodeMatrix.release();

        U_1.release();
		V_1.release();
		//M.release();			  //if the block /* of magnitude visualization */ is uncommented, then uncomment these 2 lines also
		//MN.release();
		
	} // if(processing!= firstProcessing)
	else {
        U = cv::Mat::zeros(height, width, CV_32FC1);  //I have already use U and V to put them in U_1 and V_1  //?or is it better to initialize U, V, U_1, V_1 in the threadInit?
        V = cv::Mat::zeros(height, width, CV_32FC1);

        Maskt = cv::Mat::zeros(height, width,  CV_32FC1);   //??prima   CV_32FC1

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                U.at<float>(y, x) = 0.000001;
                V.at<float>(y, x) = 0.000001;
                //Maskt.at<float>(y, x) = 0.000001;
              }
         }
		firstProcessing=false;
	}
    //--------------------------------------------------------------------------
	Matrix.release();

    return true;
}

void opfExtractorThread::threadRelease() {
    // nothing    
}

void opfExtractorThread::onStop() {
    delete inputImage;
	delete outputImage;
	delete processingImage;
    U.release();
    V.release();
    if(pt!=NULL){
        //yDebug("stopping the plotter thread");
        //printf("stopping the plotter thread \n");
        pt->stop();
    }

    if(fet!=NULL){
        //yDebug("stopping the plotter thread");
        //printf("stopping the plotter thread \n");
        fet->stop();
    }
    yInfo("closing the ports");
    //printf("closing the ports \n");

	inputPort.interrupt();
	inputPort.close();

    outputPort.interrupt();   
    outputPort.close();
}
