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
 * @file opfExtractorThread.h
 * @brief Definition of a thread that receives an RGB image from input port and sends it to the output port.
 */


#ifndef _OPF_EXTRACTOR_THREAD_H_
#define _OPF_EXTRACTOR_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/os/RateThread.h>
#include <yarp/os/Log.h>
#include <iostream>
#include <fstream>
#include <time.h>
#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
#include <cstring>

#include <iCub/plotterThread.h>
#include <iCub/featExtractorThread.h>
#include <iCub/colorcode.h>

#define WIDTH  320
#define HEIGHT 240
#define	ALGO_FB 1
#define	ALGO_TV 2
#define	ALGO_LK 3

class opfExtractorThread : public yarp::os::Thread {
private:
    int width, height;
	int ofAlgo;						// integer code to identify the optical flow algorithm
    bool idle;                      // flag that interrupts the processing 
	bool firstProcessing;
    int numberProcessing;
    bool throwAway;                 // flag that throws away one image out of two
	double TH1_,TH2_, PTH_;         // prefixed level of threshold in segmentation
    yarp::os::Semaphore idleLock;   // semaphore that checks access to the resource

    std::string robot;              // name of the robot
    std::string configFile;         // name of the configFile where the parameter of the camera are set
    std::string inputPortName;      // name of input port for incoming events, typically from aexGrabber

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;       // image generated in the processing
    yarp::sig::ImageOf<yarp::sig::PixelMono>* outputImage;     //

	yarp::sig::ImageOf<yarp::sig::PixelRgb>* processingImage;  // image resulting from the processing step
	yarp::sig::ImageOf<yarp::sig::PixelMono>* processingMonoImage;  // image resulting from the processing step
	
	cv::Mat currentMatrix;
	cv::Mat previousMatrix;
	cv::Mat outputMatrix;
    cv::Mat MaskThresholding;
    cv::Mat U;
    cv::Mat V;
    cv::Mat Maskt;
    cv::Mat MAGt;
    //cv::Mat THETAt;
    //cv::Mat Probt;
    //int DELTA;
    //int LATO;
	//IplImage ipl_currentMatrix;
    bool dataready;                  //shared flag for saying to the featExtractorThread that U,V,Maskt are ready //it is initialized to 0

    plotterThread* pt;                                       //z rateThread responsible for the visualization of the generated images
    featExtractorThread* fet;                                       // rateThread responsible for the visualization of the generated images


    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort;  //inputport for processing
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outputPort;     // output port , result of the processing
    yarp::os::Semaphore checkImage;                          // semaphore responsible for the access to the inputImage     
    std::string name;                                                                // rootname of all the ports opened by this thread

    int countDescr;

public:
    /**
    * constructor default
    */
    opfExtractorThread();

    /**
    * constructor 
    * @param robotname name of the robot
    */
    opfExtractorThread(std::string robotname,std::string configFile);

    /**
     * destructor
     */
    ~opfExtractorThread();

    /**
    *  initialises the thread
    */
    bool threadInit();

    /**
    *  correctly releases the thread
    */
    void threadRelease();

    /**
    *  active part of the thread
    */
    void run(); 

    /**
    *  on stopping of the thread
    */
    void onStop();

    /*
    * function that sets the rootname of all the ports that are going to be created by the thread
    * @param str rootnma
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /*
    * function that sets the inputPort name
    */
    void setInputPortName(std::string inpPrtName);


    /**
	*
	*/
	void setThreshold1Segmentation(double th1)  {
		TH1_ = th1; yInfo("TH1: %f",TH1_);
	}

    /**
	*
	*/
	void setThreshold2Segmentation(double th2)  {
		TH2_ = th2; yInfo("TH2: %f",TH2_);
	}

    /**
	*
	*/
	void setThreshold3Segmentation(double th3)  {
		PTH_ = th3; yInfo("PTH: %f",PTH_);
	}

    /**
     * @brief function of main processing in the thread
     */
    bool processing();

	/**
     * @brief function to  find a point (x,y) with an optical flow greater than a threshold
     */
	void thresholding(cv::Mat& Ut, cv::Mat& Vt, cv::Mat& maskThresholding);
    void thresholding(cv::Mat Ut_1, cv::Mat Vt_1, cv::Mat& Ut, cv::Mat& Vt, cv::Mat& maskThresholding, std::vector<float>& descr, int& computed); 
	void fakethresholding(cv::Mat& U, cv::Mat& V);
    
    /**
     * @brief function to  extract the features
     */
    //void computeFeatures(cv::Mat Ut_1, cv::Mat Vt_1, cv::Mat Ut, cv::Mat Vt, std::vector<float>& descr, int& computed);

	/**
     * @brief function to  find a point (x,y) with an optical flow greater than a threshold
     */
	bool segmentation(cv::Mat U, cv::Mat V);

    /**
     * @brief suspend the processing of the module
     */
    void suspend(){idleLock.wait(); idle=true; idleLock.post();};

    /**
     * @brief resume the processing of the module
     */
    void resume(){idleLock.wait(); idle=false; idleLock.post();};

    /**
     * @brief visualization suspend method
     */
    void visualizationSuspend();

    /**
     * @brief visualization resume method
     */
    void visualizationResume();

    /**
     * @brief function that test the network and feature of the thread
     * @return the result of the analysis true/false for success/unsuccess in the test
     */
    bool test();

	/**
	* @brief ...
	* @param U	...
	* @param V	...
	* @param colorcodeMatrix ...
	*/
	void motionToColor(cv::Mat U,cv::Mat V,cv::Mat& colorcodeMatrix);

	/**
	* @brief function to set the algorithm to compute the  optical flow
	*/
	inline void setAlgorithm(const int algo){
		ofAlgo=algo;
	};
};

#endif  //_OPF_EXTRACTOR_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

