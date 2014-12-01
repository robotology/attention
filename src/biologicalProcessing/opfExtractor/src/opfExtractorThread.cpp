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
#include <cstring>

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
	firstProcessing=true;
	setAlgorithm(ALGO_FB);

    inputImage = new ImageOf<PixelRgb>();
    inputImage->resize(width, height);

	outputImage = new ImageOf<PixelMono>();
	outputImage->resize(width, height);

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
                checkImage.wait();
                inputImage = inputPort.read(true);   //blocking reading for synchr with the input
                result = processing();
                checkImage.post();
                
                //passing the image to the plotter
                checkImage.wait();
                pt->copyLeft(inputImage);
                checkImage.post();            

                if (outputPort.getOutputCount()) {
                  //  yDebug("debug");
                    ImageOf<PixelMono> &b = outputPort.prepare();
                    b.resize(320,240);
                    b.copy(*outputImage);
                    outputPort.write();
                   // yDebug("debug2");
				}
            }
            else {
                result = 0;
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
		//cv::Mat M, MN;
		//double minval, maxval;
		//cv::Point  minLoc, maxLoc;
		//cv::minMaxLoc(M,&minval, &maxval, &minLoc, &maxLoc);
		//cv::Mat flow;
		//cv::Mat U =cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1),V=cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
		//cv::Mat MV[2]={U,V};
		//cv::split(flow, MV);
		switch (ofAlgo) {
			case ALGO_FB:
				yDebug("inside the FB");
				cv::Mat flow;
				cv::calcOpticalFlowFarneback(previousMatrix, currentMatrix, flow, 0.2, 3, 19, 10, 5, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
				cv::Mat U =cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1),V=cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
				cv::Mat MV[2]={U,V};
				cv::split(flow, MV);

				/* of magnitude visualization */
				cv::Mat M, MN;
				cv::sqrt(U.mul(U)+V.mul(V),M);
				double minval, maxval;
				cv::Point  minLoc, maxLoc;
				cv::minMaxLoc(M,&minval, &maxval, &minLoc, &maxLoc);

				if(maxval>0.)
					MN=255*(M-(float)minval)/((float)maxval-(float)minval);
				else 
					MN=cv::Mat::zeros(U.rows,U.cols, CV_32FC1);
				MN.convertTo(outputMatrix,CV_8UC1);

				outputImage-> wrapIplImage(&((IplImage)(outputMatrix)));		  

				flow.release();
				U.release();
				V.release();
				M.release();
				MN.release();
				break;

			//case ALGO_TV:
				//cv::Mat flow;
				//cv::Ptr<cv::DenseOpticalFlow>  DOF=cv::createOptFlow_DualTVL1();
			 //   DOF->calc(previousMatrix, currentMatrix, flow);
				//cv::Mat U =cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1),V=cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
				//cv::Mat MV[2]={U,V};
				//cv::split(flow, MV);
				//cv::Mat M/*=cv::Mat::zeros(U.rows,U.cols, CV_32FC1)*/, MN;
				////std::cout <<  M.depth() <<std::endl;
				//cv::sqrt(U.mul(U)+V.mul(V),M);
				////std::cout <<  (cv::sum(cv::abs(M)>0.5).val[0])/255<<std::endl;
				//double minval, maxval;
				//cv::minMaxLoc(M,&minval, &maxval);
				//if(maxval>0.)
				//	MN=255*(M-(float)minval)/((float)maxval-(float)minval);
				//else 
				//	MN=cv::Mat::zeros(U.rows,U.cols, CV_32FC1);
				////outputMatrix=cv::Mat::zeros(U.rows,U.cols, CV_8UC1);
				////yDebug("beforeConversion");
				//MN.convertTo(outputMatrix,CV_8UC1);
				////yDebug("afterConversion");
				//outputImage-> wrapIplImage(&((IplImage)(outputMatrix)));
				////cv::namedWindow("window");
				////cv::imshow("window",  out8);
				////cv::waitKey(5);
				//flow.release();
				//U.release();
				//V.release();
				//M.release();
				//MN.release();
				//break;

	//		case ALGO_LK:
	//			std::vector<cv::Point2f> prevPts;
	//			std::vector<cv::Point2f> currPts;
	//			cv::Size winSize(10,10); 
	//			int delta_=1;
	//			vector<uchar> status;
	//			vector<float> err;
	//			  for(int x = 0; x < previousMatrix.cols; x=x+delta_)
	//				for(int y = 0; y < previousMatrix.rows; y=y+delta_)
	//					prevPts.push_back(cv::Point2f((float)x,(float)y));				//in prevPts ci sono le x e le y dell'immagine
	//			cv::calcOpticalFlowPyrLK(previousMatrix, currentMatrix, prevPts, currPts, status, err, winSize, 3);
	//			prevPts.clear();

 //std::vector<cv::Point2f> I;
 // for (std::vector<cv::Point2f>::iterator it1 = prevPts.begin(), it2 = currPts.begin();
 //        it1 != prevPts.end() && it2 != currPts.end();
 //        ++it1,  ++it2 )
 // {
 //   I.push_back( *it1 - *it2 );
 // }

	//			flow.create(previousMatrix.rows, previousMatrix.cols, CV_32FC2);			  //create(rows,cols,type)					   in teoria qui e nelle prox 2 righe ci  va I.rows,I.cols  (ma devi usare size)
 //     
	//			cv::Mat F1 = cv::Mat::zeros(previousMatrix.rows, previousMatrix.cols, CV_32FC1);
	//			cv::Mat F2 = cv::Mat::zeros(previousMatrix.rows, previousMatrix.cols, CV_32FC1);
 //     
	//			//int P = 0;
	//			//for(int x = 0; x < I1.cols; x=x+delta_) 
	//			//	for(int y = 0; y < I1.rows; y=y+delta_, ++P) {
	//			//		F1.at<float>(y,x) = status_.at<uchar>(y,x) ? -prevPts[P].x + currPts_[P].x : pow(10,10);
	//			//		F2.at<float>(y,x) = status_.at<uchar>(y,x) ? -prevPts[P].y + currPts_[P].y : pow(10,10);
	//			//		}
 //     
	//			 cv::Mat mv[2] = {F1, F2};
	//			 cv::merge(mv, 2, flow);



	//	cv::split(flow, MV);

	///* of magnitude visualization */
	////cv::Mat M, MN;
	//cv::sqrt(U.mul(U)+V.mul(V),M);
	////double minval, maxval;
	////cv::Point  minLoc, maxLoc;
	//cv::minMaxLoc(M,&minval, &maxval, &minLoc, &maxLoc);

	//if(maxval>0.)
	//	MN=255*(M-(float)minval)/((float)maxval-(float)minval);
	//else 
	//	MN=cv::Mat::zeros(U.rows,U.cols, CV_32FC1);
	//MN.convertTo(outputMatrix,CV_8UC1);

	//outputImage-> wrapIplImage(&((IplImage)(outputMatrix)));		  

	//flow.release();
	//U.release();
	//V.release();
	//M.release();
	//MN.release();
 //     
	//			 F1.release();
	//			 F2.release();
	//  

	//			break;

		}  // switch
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

