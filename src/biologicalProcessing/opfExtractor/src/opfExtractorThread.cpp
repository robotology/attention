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
    setAlgorithm(ALGO_FB);

    inputImage = new ImageOf<PixelRgb>();
    inputImage->resize(width, height);

    outputImage = new ImageOf<PixelMono>();
    outputImage->resize(width, height);

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

    //starting the plotterThread
    pt = new plotterThread();
    pt->setName(getName("").c_str());
    pt->start();

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
                               
                    result = processing();               // generates the outputImage which is what we want to plot                                
                    pt->copyImage(processingImage); 
                            
                    if (outputPort.getOutputCount()) {
                        //yDebug("debug");
                        ImageOf<PixelMono> &b = outputPort.prepare();
                        b.resize(320,240);
                        b.copy(*outputImage);
                        outputPort.write();
                        //yDebug("debug2");
                    }
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
	double minval, maxval;	
	cv::Point  minLoc, maxLoc;
	cv::minMaxLoc(U,&minval, &maxval, &minLoc, &maxLoc);
	//std::cout  << minval << " " << maxval << std::endl;
	cv::minMaxLoc(V,&minval, &maxval, &minLoc, &maxLoc);
	//std::cout  << minval << " " << maxval << std::endl;


	uchar *ccMatrixPointer = colorcodeMatrix.data;					  //data is pointing to the Red of the 1st pixel

	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			float fx = U.at<float>(y, x);//motim.Pixel(x, y, 0);
			float fy = V.at<float>(y, x);//motim.Pixel(x, y, 1);
			 //.Pixel(x, y, 0);		
			computeColor(fx/maxval, fy/maxval, ccMatrixPointer);
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
                U.at<float>(y, x)=1.0;
                V.at<float>(y, x)=2.0;            
		}
    }
    printf("faketh end \n");
}


  void opfExtractorThread::thresholding(cv::Mat& U, cv::Mat& V, cv::Mat& maskThresholding){ 
    cv::Mat MAGt = cv::Mat::zeros(U.rows, U.cols, CV_32FC1);
    cv::Mat THETAt = cv::Mat::zeros(U.rows, U.cols, CV_32FC1);
    cv::Mat MAGt_1 = cv::Mat::zeros(U.rows, U.cols, CV_32FC1);
    cv::Mat THETAt_1 = cv::Mat::zeros(U.rows, U.cols, CV_32FC1);  

    

    // from u-v to Magnitude-Theta
    cv::cartToPolar(U, V, MAGt, THETAt, false);
  
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
                Probt.at<float>(i,j) = cv::sum(MQ).val[0]/((float)(LATO*LATO));		  // divide by lato*lato in such a way to have maximum 1
                
                // 	Probt.at<float>(i,j) = ((float)(cv::sum(MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1)) >= TH2_).val[0]))/((float)(LATO*LATO));
        
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
    
    
    
    bool COND = cv::sum(Maskt).val[0] >= 0.01*(U.rows*U.cols);	//global statistics: we consider just what has a movement greater than the 1%  of  the total of the image
    //bool COND = cv::sum(Maskt).val[0] >= 0.01*(255); 
    //   std::cout << cv::sum(Maskt).val[0] << " " << 0.1*(Ut.rows*Ut.cols) << " " << COND << std::endl;

    
    /*
    if(!COND) {  
        //if(DEBUG_) {
        //  cv::Mat Im = cv::Mat::zeros(U.rows, U.cols, CV_8UC1);
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
    yInfo("identifying the connect components");
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat MaskClone = Maskt.clone();
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(MaskClone, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);	//MaskClone because findContours admit to write
    MaskClone.release();
  
    
    yInfo("iterating the pixels");
    for(std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ) {						 //contours.begin() é puntatore all elem iniziale
        if(cv::contourArea(*it)/(Maskt.rows*Maskt.cols)>0.005)
            ++it; //siamo a livello di ogni bounding box ,  quindi  la treshold é piu piccola
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

    yInfo("drawing contours");
    cv::drawContours(maskThresholding, contours, -1, CV_RGB(255,255,255), -1);			 //it puts zeros where the optical flow is low and 255 where the optical   flow is high             -1 vuol   dire che  devo  colarle tutti  i bounding box. l'altro -1 che devo colarli  pieno (controlla)    
    
    yInfo("removing the UV in the mask");
    for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
            if (!maskThresholding.at<float>(y, x)==0.0) {
                U.at<float>(y, x) = 0.000001;
                V.at<float>(y, x) = 0.000001;
            }             
		}
    }
}


bool opfExtractorThread::processing(){
	//	yDebug("processing");
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
		cv::Mat U =cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
        cv::Mat V=cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
        cv::Mat maskThresholding = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
		cv::Mat MV[2]={U,V};
		switch (ofAlgo) {
		case ALGO_FB:{
				cv::calcOpticalFlowFarneback(previousMatrix, currentMatrix, flow, 0.2, 3, 19, 10, 5, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);		//fare quello che abbiamo fatto in fondo
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
						prevPts.push_back(cv::Point2f((float)x,(float)y));				//in prevPts ci sono le x e le y dell'immagine
					}
				}
				cv::calcOpticalFlowPyrLK(previousMatrix, currentMatrix, prevPts, currPts, status, err, winSize, 3);
				
				for(int k = 0; k < prevPts.size(); k++)	{  //I é un vettore ogni elemento é un punto (quindi con 2 campi)
					//std::cout  << currPts.at(k).x << " "  << currPts.at(k).y << std::endl;
					if (status.at(k))	{
						U.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  currPts.at(k).x - prevPts.at(k).x;
						V.at<float>(prevPts.at(k).y, prevPts.at(k).x)	=  currPts.at(k).y - prevPts.at(k).y;
					}
				}
				
				//flow.create(previousMatrix.rows, previousMatrix.cols, CV_32FC2);			  //create(rows,cols,type)					   in teoria qui e nelle prox 2 righe ci  va I.rows,I.cols  (ma devi usare size)
  
      
				//int P = 0;
				//for(int k = 0; k < I.size(); k++)	  //I é un vettore ogni elemento é un punto (quindi con 2 campi=
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
		}  // switch


		/* computing colorcode */
		cv::Mat colorcodeMatrix =  cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_8UC3);	
		//motionToColor(U, V, colorcodeMatrix);
		////colorcodeMatrix.convertTo(outputMatrix,CV_8UC3);

        /*step1  :  taking a point (x,y) with a flow magnitude more than a threshold and then 
        computing the number of points around the point (x,y) with  the magnitude of the flow and taking the number of points with a flow magnitude more than a threshold*/
        //fakethresholding(U, V);
        thresholding(U, V, maskThresholding);
        //U = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
        //V = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
        motionToColor(U, V, colorcodeMatrix);
        /*step2 	:  segmentation	*/
        //segmentation();
		
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
            V.convertTo(outputMatrix,CV_8UC1);
            IplImage tempIpl = (IplImage) outputMatrix;
            outputImage-> wrapIplImage(&tempIpl);
            //outputImage-> wrapIplImage(&((IplImage)(outputMatrix)));
        }

		flow.release();
		U.release();
		V.release();
        maskThresholding.release();
		colorcodeMatrix.release();
		//M.release();			  //if the block /* of magnitude visualization */ is uncommented, then uncomment these 2 lines also
		//MN.release();
		
	} // if(processing!= firstProcessing)
	else {
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
    if(pt!=NULL){
        //yDebug("stopping the plotter thread");
        //printf("stopping the plotter thread \n");
        pt->stop();
    }
    yInfo("closing the ports");
    //printf("closing the ports \n");

	inputPort.interrupt();
	inputPort.close();

    outputPort.interrupt();   
    outputPort.close();
}

