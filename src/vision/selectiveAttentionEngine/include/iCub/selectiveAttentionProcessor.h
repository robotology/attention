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
 * @brief Implementation of the thread which performs computation in the selectiveAttentionModule
 */

#ifndef _selectiveAttentionProcessor_H_
#define _selectiveAttentionProcessor_H_

//ipp include
//#include <ippi.h>

//openCV includes
#include <cv.h>
#include <highgui.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

/* Log-Polar includes */
#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>

#include <string>
#include <time.h>

#include <iCub/observer.h>
#include <iCub/observable.h>
#include <iCub/prioCollectorThread.h>

const int THREAD_RATE=30;

/**
 *This code groups together a series of useful functions that can be used for ImageProcessing
 */

class selectiveAttentionProcessor:public yarp::os::RateThread,public observer {
private:
    int psb;                          //width step of 8u images
    int psb32;                        //width step of 32f images
    int psb_border;                   //width step of the image with border for 3x3 operator convolution
    
    yarp::sig::ImageOf<yarp::sig::PixelMono> *tmp;               // temporary mono image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *tmp2;              // temporary rgb image
    yarp::sig::ImageOf<yarp::sig::PixelRgb>  *intermCartOut;     // temporary rgb image
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > thImagePort;              // port for the output the WTA
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb > > imageCartOut;             // port for sending cartesian image result
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map1Port;                 // input port for the 1st saliency map
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map2Port;                 // input port for the 2nd saliency map
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map3Port;                 // input port for the 3rd saliency map
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map4Port;                 // input port for the 4th saliency map
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map5Port;                 // input port for the 5th saliency map
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > map6Port;                 // input port for the 6th saliency map
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > motionPort;               // input port for the flow motion
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > cart1Port;                // input port for the 1st cartesian saliency map
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > cart2Port;                // input port for the 2nd cartesian saliency map
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > cart3Port;                // input port for the 3rd cartesian saliency map

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > linearCombinationPort;    // output port that represent the linear combination of different maps
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > facilitPort;              // where the image of salient features in spatiocentric reference frame are sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > inhiCartPort;             // where the image of cuncurrent inhibition of return can be sent (cartesian)
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > inhiPort;                 // where the image of cuncurrent inhibition of return can be sent 
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > testPort;                 // debug mode port for testing the appearance of a particular image in the process
        
    yarp::os::BufferedPort<yarp::os::Bottle > vergencePort;                                      //port dedicated to the communication with the vergence
    yarp::os::BufferedPort<yarp::os::Bottle > centroidPort;                                     // output port where the centroid coordinate is sent
    yarp::os::BufferedPort<yarp::os::Bottle > gazeCoordPort;                                    // port that is dedicated to the streaming out gaze coordinates
    yarp::os::BufferedPort<yarp::os::Bottle > outputCmdPort;                                    // port that is dedicated to sending the typology of the gaze behaviour and some params
    yarp::os::BufferedPort<yarp::os::Bottle > vergenceCmdPort;                                  // port that is dedicated to command of vergence, this helps to calculated the relative depth of the object
    yarp::os::BufferedPort<yarp::os::Bottle > magnoCellFeedback;
    
    yarp::os::Port facilitRequestPort;                                                          // port where the correct portion of facilitation is requested
    yarp::os::Port portionRequestPort;                                                          // port dedicated to the process of requestion the correct portion in the mosaic
    yarp::os::Port feedbackPort;                                                                // port necessary to send back command to the preattentive processors
    
    
    //yarp::sig::ImageOf<yarp::sig::PixelMono>* outputImagePlane; //temp variable for plane extraction;
    int cLoop;                                                  //counter of the loop
    int camSel;                                                 //select the image plane: left or right ( 0: left, 1: right )
    int counterMotion; 
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *image_out;         // temp variable for plane extraction;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *image_tmp;        // temp variable for plane extraction;
    int targetRED;          // value read from the blobFinder component (red intensity of the target)
    int targetGREEN;        // value read from the blobFinder component (green intensity of the target)
    int targetBLUE;         // value read from the blobFinder component (blue intensity of the target)
    int width;              // width of the input image
    int height;             // height of the input image
    int xSizeValue;         // x dimension of the remapped cartesian image
    int ySizeValue;         // y dimension of the remapped cartesian image
    double overlap;         // overlap in the remapping
    int numberOfRings;      // number of rings in the remapping
    int numberOfAngles;     // number of angles in the remapping
    int jointNum;           // number of joints controlled
    int saccadicInterval;   // time interval between two different saccadic event
    double salienceTD;      // value of the weight of top-down approach in the blobFinder
    double salienceBU;      // value of the weight of bottom-up approach in the blobFinder
    unsigned char targetRed;        // colour information passed back for the reinforcement
    unsigned char targetGreen;      // colour information passed back for the reinforcement
    unsigned char targetBlue;       // colour information passed back for the reinforcement
    std::string name;               // name of the module and rootname of the connection
    std::string robotName;          // name of the robot
    bool reinit_flag;               // flag that is set after the dimension of the images is defined
    bool interrupted;               // flag set when the interrputed function has already be called
    bool idle;                      // flag for the idle state of the processor   
    bool gazePerform;               // flag that allows the processor to ask the gazeControl for saccadic movements
    bool directSaccade;             // when it is on the module sends commands directly to the iKinGazeCtrl
    bool maxResponse;               // flag that is set to TRUE when the response in combination of saliency maps is max
    bool handFixation;              // flag that indicates whether the robot fixates its hand
    bool earlystage;                // flag that allows the early stage and eventually the reduction in response
    bool secondstage;               // flag that allows the second stage and eventually the reduction in response
    bool interruptJump;
    yarp::os::Semaphore mutex;      // semaphore for the respond function
    yarp::os::Semaphore mutexInter; // semaphore for the interruptJump
    
    double z;                                   // distance [m]
    double xm, ym;                              // position of the most salient object in the combination
    double timing;                              // priority timing response
    yarp::dev::IGazeControl *igaze;             // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;      // polydriver for the gaze controller
    iCub::logpolar::logpolarTransform trsf;     // reference to the converter for logpolar transform
    yarp::dev::PolyDriver* armRobotDevice;      // device necessary to control the robot arm
    yarp::dev::PolyDriver* cartCtrlDevice;      // device necessary to control in the cartesian frame of reference
    
    yarp::dev::IPositionControl *armPos;        // element that encodes the arm position
    yarp::dev::IEncoders *armEnc;               // element that encodes the encoder values of the arm
    yarp::dev::ICartesianControl *armCart;      // cartesian control of the arm
    
    double startInt;                        // time variable for saccade activation
    double endInt;                          // time variable for saccade activation
    double saccadeInterv;                   // time costant between two different saccades (milliseconds)
    double habituationStop;                 // stop time counter for habituation
    double habituationStart;                // start time counter for habituation
    time_t start2, end2;
    
    double k1;          // coefficient for the linear combination of log-polar maps
    double k2;          // coefficient for the linear combination of log-polar maps
    double k3;          // coefficient for the linear combination of log-polar maps
    double k4;          // coefficient for the linear combination of log-polar maps
    double k5;          // coefficient for the linear combination of log-polar maps
    double k6;          // coefficient for the linear combination of log-polar maps
    double bu;          // weigth of the bottom-up contribution
    double td;          // weight of the top-down contribution
    double kmotion;     // coefficient of the linear combination of the motion
    double kc1;         // coeffiencient for the linear combination of the 1 cartesian maps
    double kc2;         // coeffiencient for the linear combination of the 2 cartesian maps
    double kc3;         // coeffiencient for the linear combination of the 3 cartesian maps

    yarp::sig::ImageOf<yarp::sig::PixelMono>* habituationImage; // mono image for habituation process
    float* habituation; // mono image for habituation process


    yarp::sig::ImageOf<yarp::sig::PixelRgb >* inImage;          // input image  of the processing
    yarp::sig::ImageOf<yarp::sig::PixelRgb >* inColourImage;    // input image  of the processing
    yarp::sig::ImageOf<yarp::sig::PixelRgb >* inputLogImage;    // 3channel image representing the saliencymap in logpolar
    
    yarp::sig::ImageOf<yarp::sig::PixelMono>* hueMap;           // hue map reference
    yarp::sig::ImageOf<yarp::sig::PixelMono>* satMap;           // saturation map reference
    yarp::sig::ImageOf<yarp::sig::PixelMono>* map1_yarp;        // saliency map coming from the 1st source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* map2_yarp;        // saliency map coming from the 2nd source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* map3_yarp;        // saliency map coming from the 3rd source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* map4_yarp;        // saliency map coming from the 4th source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* map5_yarp;        // saliency map coming from the 5th source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* map6_yarp;        // saliency map coming from the 6th source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* motion_yarp;      // saliency map coming from the 6th source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* cart1_yarp;       // saliency map coming from the 1st Cartesian source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* cart2_yarp;       // saliency map coming from the 2nd Cartesian source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* cart3_yarp;       // saliency map coming from the 3rd Cartesian source
    yarp::sig::ImageOf<yarp::sig::PixelMono>* inhicart_yarp;    // cartesian input of the inhibition of return
    yarp::sig::ImageOf<yarp::sig::PixelMono>* facilit_yarp;     // cartesian image of the facilitation paradigm
    yarp::sig::ImageOf<yarp::sig::PixelMono>* inhi_yarp;        // logpolar input of the inhibition of return
    yarp::sig::ImageOf<yarp::sig::PixelMono>* edges_yarp;       // yarp image of the composition of all the edges
    yarp::sig::ImageOf<yarp::sig::PixelMono>* faceMask;         // yarp image of regions of skin colour
    yarp::sig::ImageOf<yarp::sig::PixelMono>* linearCombinationImage;   //image result of linear combination of all the feature maps
    
    IplImage *cvImage16; // tmp IPLImage necessary for edge detection 16 bit
    IplImage *cvImage8; //tmp IPLImage necessary for edge detection 16 bit
    //Ipp8u* im_out;

    int inputImage_flag;  //processor flag
    
    static const int CONVMAX_TH = 100; //parameter of the findEdges function
    static const int CONVSEQ_TH = 500; //parameter of the findEdges function
    
    yarp::sig::ImageOf<yarp::sig::PixelMono>* linearCombinationPrev;      //result of the combination (previous time sample)
    int centroid_x; //center of gravity of the selective attention (x position)
    int centroid_y; //center of gravity of the selective attention (y position)
   
    prioCollectorThread* earlyTrigger;

    static const int thresholdHabituation = 240;                          // static threashold for the habituation
    
public:
    /**
     * constructor
     */
    selectiveAttentionProcessor(int rateThread);//
    
    /**
     * default destructor
     */
    ~selectiveAttentionProcessor();//
    
    /**
     * constructor 
     * @param inputImage image where the selectiveAttentionProcessor is started from
     */
    selectiveAttentionProcessor(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage );//
    
    /**
     *initialization of the thread 
     */
    bool threadInit();
    
    /**
     * active loop of the thread
     */
    void run();
    
    /**
     * releases the thread
     */
    void threadRelease();
    
    /**
     * function that suspends the processing thread
     */
    void suspend();
    
    /**
     * resumes the processing thread previously suspended
     */
    void resume();

    /**
    * function that defines what has to be done once any observeble interrupts
    * @param o observable that has just interrupted the observer
    * @param arg Bottle that has passed from the observable to the observer
    */
    void update(observable* o, yarp::os::Bottle * arg);
    
    /**
     * method that resize images once the processor knows the dimesions of the input
     * @param width width dimension the image is resized to
     * @param height height dimension the image is resized to
     */
    void resizeImages(int width, int height);
    
    /**
     * function called when the module is poked with an interrupt command
     */
    void interrupt();
    
    /**
     * function that reinitiases some attributes of the class
     */
    void reinitialise(int width, int height);
    
    /**
     * function that gives reference to the name of the module
     * @param name of the module
     */
    void setName(std::string name);
    
    /**
     * function that returns the name of the module
     * @param str string to be added
     * @return name of the module
     */
    std::string getName(const char* str);
    
    /**
     * function that set the robotname
     * @param str robotname as a string
     */
    void setRobotName(std::string str);
    
    
    /**
     * opens all the ports necessary for the module
     * @return return whether the operation was successful
     */
    bool openPorts();
    
    /**
     * closes all the ports opened when the module started
     * @return return whether the operation was successful
     */
    bool closePorts();
    
    /**
     * streams out data on ports
     * @return return whether the operation was successful
     */
    bool outPorts();
    
    /**
     * set the flag idle locking the resource
     */
    void setIdle(bool value);
    
    /*
     * function that sets which one the two attentive chain (left or right) drives the gaze
     * @param value is the value of the cam selection ( 0: left, 1: right )
     */
    void setCamSelection(int value);
    
    /**
     * function that set the dimension of the remapped cartesian image
     * @param xSize dimension in x
     */
    void setXSize(int xSize);
    
    /**
     * function that set the dimension of the remapped cartesian image
     * @param ySize dimension in y
     */
    void setYSize(int ySize);
    
    /**
     * function that declare the overlap needed in the reconstruction
     * @param overlap value of the overlapping
     */
    void setOverlap(double overlap);

    /**
     * set the counter of the motion feature maps. Activates the motion feature
     * extraction after a predefined lapse of time. This avoids responses after discontin.
     * @param value integer associated with counter
     */
    void setCounterMotion(double value) {counterMotion = value; };
    
    /**
     * function that declares the number of rings of the image has to be remapped
     * @param numberOfRings number of rings
     */
    void setNumberOfRings(int numberOfRings);
    
    /**
     * function that declares the number of angles of the image has to be remapped
     * @param numberOfAngles number of angles
     */
    void setNumberOfAngles(int numberOfAngles);
    
    /**
     * function that activates/deactivates the fixation point attached to the hands
     * @param true/false when the action is set active/notactive
     */
    void setHandFixation (bool value){ handFixation=value; } ;
    
    /**
     * function that sets the time interval between two different saccadic events
     * @param interval milliseconds between two different saccadic events
     */
    void setSaccadicInterval(int interval);
    
    /*
     * function that declares whether to perform saccadic movement with gazeControl
     * @param value It is true when the saccadic movement is performed
     */
    void setGazePerform(bool value);

    /*
     * function that declares whether to earlyStage is active
     * @param value It is true when earlyStage is active
     */
    void setEarlyStage(bool value) {earlystage = value;secondstage = value; };

    /*
     * function that declares whether to secondStage is active
     * @param value it is true when secondStage is active
     */
    void setSecondStage(bool value) {secondstage = value;secondstage = value; };
    
    /**
     * function that returns the value of the saccadicInterval
     */
    int getSaccadicInterval() {return saccadicInterval; };
    
    /**
     * function that returns the z dimension of the saccadic event
     */
    double getZ() { return z; };
    
    /**
     * function that returns the value of the parameter k1
     */
    double getK1() { return k1; };
    
    /**
     * function that returns the value of the parameter k2
     */
    double getK2() { return k2; };
    
    /**
     * function that returns the value of the parameter k3
     */
    double getK3() { return k3; };
    
    /**
     * function that returns the value of the parameter k4
     */
    double getK4() { return k4; };
    
    /**
     * function that returns the value of the parameter k5
     */
    double getK5() { return k5; };
    
    /**
     * function that returns the value of the parameter k6
     */
    double getK6() { return k6; };
    
    /**
     * function that returns the value of the parameter kc1
     */
    double getKC1() { return kc1; };

    /**
     * function that returns the value of the parameter kc1
     */
    double getKC2() { return kc2; };

    /**
    * function that returns the value of the parameter kc1
    */
    double getKC3() { return kc3; };
    
    /**
     * function that returns the value of the parameter kmotion
     */
    double getKMotion() { return kmotion; };
    
    /**
     * function that sets the z dimension of the saccadic event
     */
    void setZ(double parameterZ) { z = parameterZ; };
    
    /**
     * function that sets the value of the parameter k1
     */
    void setK1(double p) { k1=p; };
    
    /**
    * function that sets the value of the parameter k2
    */
    void setK2(double p) { k2=p; };
    
    /**
     * function that sets the value of the parameter k3
     */
    void setK3(double p) { k3=p; };
    
    /**
     * function that sets the value of the parameter k4
     */
    void setK4(double p) { k4=p; };
    
    /**
     * function that sets the value of the parameter k5
     */
    void setK5(double p) { k5=p; };
    
    /**
     * function that sets the value of the parameter k6
     * @param p value that is going to be set
     */
    void setK6(double p) { k6=p; };

    /**
     * @brief function that sets the weight of the bottom-up
     * @param p value that is going to be set
     */
    void setBU(double p) { bu = p;};

    /**
     * @brief function that sets the weight of the top-down 
     * @param p value that is going to be set
     */
    void setTD(double p) { td = p;};
    
    /**
     * function that sets the reference of the hue Map
     * @param p value that is going to be set
     */
    void setHueMap(int p);
    
    /**
     * function that sets the reference of the saturation Map
     * @param p value that is going to be set
     */
    void setSatMap(int p);
    
    /**
     * function that sets the value of the parameter kc1
     */
    void setKC1(double p) { kc1 = p; };

    /**
     * function that sets the value of the parameter kc2
     */
    void setKC2(double p) { kc2 = p; };

    /**
    * function that sets the value of the parameter kc2
    */
    void setKC3(double p) { kc3 = p; };
    
    /**
     * function that sets the value of the parameter kMotion
     */
    void setKMotion(double p) { kmotion=p; };

    /**
     * function that suppress magno cells contribution
     */
    void magnoCellularSuppression(bool value);

    /**
     * function that navigates in the logpolar image looking for maxima
     * @param map1 map coming from contrast feature map
     * @param map2 map coming from motion feature map
     * @param linearCombination combinationof the linear maps
     */
    bool earlyFilter(yarp::sig::ImageOf<yarp::sig::PixelMono>* map1,yarp::sig::ImageOf<yarp::sig::PixelMono>* map2, yarp::sig::ImageOf<yarp::sig::PixelMono>* linearCombination );

    /**
     * function that extract the contour and the center of gravity
     * @param inputImage input image where the contours are extracted from
     * @param outImage representation of the contours
     * @param inputColourImage image where the colour are extracted
     * @param x x position of the center of mass of the contours
     * @param y y position of the center of mass of the contours
     */
    void extractContour(yarp::sig::ImageOf<yarp::sig::PixelMono>* inputImage,yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputColourImage ,int& x,int& y);

    /**
     * function that extracts the colour of a region around a pixel given the input image
     * @param inputColourImage input image where the region colour is read.
     * @param x position in the image plane
     * @param y position in the image plane
     * @param redIntensity intensity of the red plane of the region colour
     * @param greenIntensity intensity of the red plane of the region colour
     * @param blueIntensity intensity of the red plane of the region colour
     */
    void getPixelColour(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputColourImage,int x ,int y, unsigned char &redIntensity, unsigned char &greenIntensity, unsigned char &blueIntensity);
    
    /**
     * function that copies two images and check whether the output image is hue map or sat map.
     * It creates a preprocessing stage for face colour detection
     * @param src source image copy from
     * @param dest dest image coy to
     */
    void copy_C1R(yarp::sig::ImageOf<yarp::sig::PixelMono>* src, yarp::sig::ImageOf<yarp::sig::PixelMono>* dest);
    
    
    
};


#endif // _selectiveAttentionModule_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

