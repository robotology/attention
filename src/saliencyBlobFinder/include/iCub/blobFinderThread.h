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
 * @file blobFinderThread.h
 * @brief module class definition for the blob finder thread (this is the module actual processing engine).
 */

#ifndef _BLOBFINDERTHREAD_H_
#define _BLOBFINDERTHREAD_H_

//#include <ippi.h>
//#include <ippcore.h>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/dev/all.h>

#include <iCub/WatershedOperator.h>
#include <iCub/SalienceOperator.h>
#include <iCub/iKin/iKinInv.h>
#include <iCub/iKin/iKinIpOpt.h>
#include <iCub/iKin/iKinFwd.h>
#include <iCub/ctrl/pids.h>



/*
 * This a class that actually reads the input from the input image (edges),
 * performs in sequence watershed, blob extraction, colour quantization.
 * Finally it builds a feature map of the proto-objects defined and sends it
 * to the output image
 */
class blobFinderThread : public yarp::os::RateThread {
private:
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inputPort;         // port where the input image is read from
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > edgesPort;        // port where the edges image is read
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > saliencePort;     // port that returns the salience map
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outputPort3;       // port that returns the image output 3channels
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > rgbPort;          // port where RGB image is streamed
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > rgPort;           // port where the difference of gaussian R+G- is streamed
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > grPort;           // port where the difference of gaussian G+R- is streamed
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > byPort;           // port where the difference of gaussian B+Y- of the image is streamed
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > rgOut;           // port where the difference of gaussian R+G- is streamed
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > grOut;           // port where the difference of gaussian G+R- is streamed
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > byOut;           // port where the difference of gaussian B+Y- of the image is streamed
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > yellowPort;       // port where the yellow plane of the image is streamed
    yarp::os::Port blobDatabasePort;                               // port where all the blobs as bottles are sent to the objectPropertiesCollector

    yarp::sig::ImageOf<yarp::sig::PixelMono> *outContrastLP;       // image result of the function outContrastLP
    yarp::sig::ImageOf<yarp::sig::PixelMono> *tmpImage;            // buffer image for received image
    yarp::sig::ImageOf<PixelBgr> *outMeanColourLP;                 // image result of the function meanColourLP;
    
    IppiSize srcsize;                                   // ipp reference to the size of the input image
    int width;                                          // width of the input image
    int height;                                         // height of the input image
    int nBlobs;                                         // number of blobs extracted
    int memoryPos;                                      // number of saved positions
    
    std::string configFile;                             // configuration file of both the cameras
    std::string name;                                   // name of the module and rootname of the connection
    std::string robot;                                  // name of the robot for connecting polydrive
    bool reinit_flag;                                   // flag that indicates when the reinitiazation has already be done
    bool resized_flag;                                  // flag that indicates if the images have been resized
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *img;                             // input image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *edges;                          // edges image
   
    WatershedOperator *wOperator;                       // pointer to the watershed operator
    char* blobList;                                     // vector of boolean which tells whether there is a blob or not
    char* memory;                                       // memory of the 3d locations attemded
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb>  *_outputImage3;                  // pointer to the 3 channels output image of the watershed algorithm
    yarp::sig::ImageOf<yarp::sig::PixelMono> *_inputImgRGS;                   // input image of the opponency R+G-
    yarp::sig::ImageOf<yarp::sig::PixelMono> *_inputImgGRS;                   // input image of the opponency G+R-
    yarp::sig::ImageOf<yarp::sig::PixelMono> *_inputImgBYS;                   // input image of the opponency B+Y-
    yarp::sig::ImageOf<PixelInt>  *ptr_tagged;                                // pointer to the image of tags

    yarp::dev::IGazeControl *igaze;                         // Ikin controller of the gaze
    yarp::dev::PolyDriver* clientGazeCtrl;                  // polydriver for the gaze controller
    yarp::dev::PolyDriver *polyTorso, *drvHead;             // polydriver for the control of the torso and head
    iCub::iKin::iCubEye *eyeL;
    iCub::iKin::iCubEye *eyeR;
    yarp::dev::IEncoders   *encTorso,*encHead;              // encoders of the torso and head
    yarp::sig::Matrix *invPrjL, *invPrjR;                   // inverse of prjection matrix
    yarp::sig::Matrix *PrjL, *PrjR;                         // projection matrix


    SalienceOperator *salience;                             // reference to the salience operator
    YARPBox* max_boxes;                                     // pointer to the most salient blob

    yarp::sig::ImageOf<yarp::sig::PixelMono> *image_out;                          // image which is plotted in the drawing area
    yarp::sig::ImageOf<yarp::sig::PixelRgb>  *image_out2;                          // image which is plotted in the drawing area

    yarp::sig::ImageOf<yarp::sig::PixelRgb>  *ptr_inputImg;                       // pointer to the input image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgRed;                    // pointer to the red plane input image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgGreen;                  // pointer to the green plane input image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgBlue;                   // pointer to the input blue plane image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgYellow;                  // pointer to the input blue plane image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_tmpRplus, *ptr_tmpRpluss;       // temp image to hold convolution result
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_tmpRminus, *ptr_tmpRminuss;     // temp image to hold convolution result
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_tmpGplus, *ptr_tmpGpluss;       // temp image to hold convolution result
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_tmpGminus, *ptr_tmpGminuss;     // temp image to hold convolution result
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_tmpBplus, *ptr_tmpBpluss;       // temp image to hold convolution result
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_tmpYminus, *ptr_tmpYminuss;     // temp image to hold convolution result
    
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgRG;                     // pointer to the input image R+G-
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgGR;                     // pointer to the input image G+R-
    yarp::sig::ImageOf<yarp::sig::PixelMono> *ptr_inputImgBY;                     // pointer to the input image B+Y-
    yarp::sig::ImageOf<yarp::sig::PixelRgb>  *_procImage;                         // pointer to the output image of the watershed algorithm

    int maxBLOB;                                            // maxBLOB dimension
    int minBLOB;                                            // minBLOB dimension
    int max_tag;                                            // number of blobs
    int minBoundingArea;                                    // dimension of the bounding area in saliency BU algorithm
    int saddleThreshold;                                    // threshold necessary to determine the saddle point of rain falling watershed

    PixelMono targetRG;                                     // target colour in colour opponency map 
    PixelMono targetGR;                                     // target colour in colour opponency map
    PixelMono targetBY;                                     // target colour in colour opponency map
    PixelMono targetRed;                                    // target colour in RGB 
    PixelMono targetGreen;                                  // target colour in RGB
    PixelMono targetBlue;                                   // target colour in RGB
    
    double weightTD;                                        // weight of the Top-Down contribution
    double weightBU;                                        // weight of the Bottom-Up contribution

private: 
    /**
    * resizes all the needed images
    * @param width width of the input image
    * @param height height of the input image
    */
    void resizeImages(int width, int height);

    /**
    * function that extracts characteristics of all the blobs in the catalogue and save them
    * @param stable parameters that enable some lines of code for the stable version
    */
    void drawAllBlobs(bool stable);
    
    /**
    * function that free memory allocated for the look-up table
    */
    bool freeLookupTables();

public:
    /**
    * constructor
    * param rateThread period of the processing thread
    */
    blobFinderThread(int rateThread, std::string configFile);

    /**
    * destructor
    */
    ~blobFinderThread();

    /**
    *initialization of the thread 
    */
    bool threadInit();

    /**
    * active loop of the thread
    */
    void run();

    /**
    *releases the thread
    */
    void threadRelease();

    /**
    * function that reinitiases some attributes of the class
    * @param height height of the input image
    * @param width width of the input image
    */
    void reinitialise(int width,int height);

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();

    /**
    * function that defines the name of the robot
    * @param name  name of the robot
    */
    void setRobotName(std::string name);
    
    /**
    * function that gives reference to the name of the module
    * @param name name of the module
    */
    void setName(std::string name);
    
    /**
    * function that returns the name of the module
    * @param str string to be added
    * @return name of the module
    */
    std::string getName(const char* str);
    
    /**
    * function the applies the watershed (rain falling) algorithm
    * @param edgesImage image representing the edges extracted from colourOpponency maps
    */
    void rain(yarp::sig::ImageOf<yarp::sig::PixelMono>* edgesImage);
    
    /**
    * function that resets all the flags for the desired output
    */
    void resetFlags();
    
    /**
    * function that reads the ports for colour RGB opponency maps
    */
    bool getOpponencies();
    
    /**
    * function that reads the ports for the RGB planes
    * @param inputImage image where we can extract planes from
    */
    bool getPlanes(yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage);

    /**
    * function which applies a linear kernel (2D kernel must be seperated, if possible, via SVD method) to a given image
    * The image need not be enlarged as this function takes care of edges efficiently.
    * @param vecSize size of the linear kernel
    * @param vec pointer to linear kernel
    * @param img pointer to IplImage on which kernel is to be applied
    * @param factor the value by which the sum must be scaled in resulting image. default: 1.0
    * @param direction the direction of application of kernel (0 for horizontal else vertical). default: 0
    * @param maxVal maximum value of the convolution operator, so as to avoid overflow. default: 255
    */
    void convolve1D(int vecSize, float* vec, yarp::sig::ImageOf<yarp::sig::PixelMono>* img, yarp::sig::ImageOf<yarp::sig::PixelMono>* resImg, float factor=1.0,int shift =0,int direction=0,int maxVal=255);

    /**
     * set the minimum bounding area of the blob
     */
    inline bool setMinBoundingArea(int w) { minBoundingArea = w; return true; }

    /** 
     * get the minumum bounding area of the blob
     */
    inline int getMinBoundingArea() const { return minBoundingArea; }

    /**
     * set saliency percentage area
     */
    inline bool setSaliencyPercentageArea(double p) { return salience->setPercentageArea(p); }

    /**
     * get saliency percentage area
     */
    inline double getSaliencyPercentageArea() const { return salience->getPercentageArea(); }

    /**
     * set maximum size of blobs
     */
    inline bool setMaxBlobSize(int s) { maxBLOB = s; return true; }

    /**
     * get maximum size of blobs
     */
    inline int getMaxBlobSize() const { return maxBLOB; }

    /**
     * set minimum size of blobs
     */
    inline bool setMinBlobSize(int s) { minBLOB = s; return true; }

    /**
     * get minimum size of blobs
     */
    inline int getMinBlobSize() const { return minBLOB; }

    /**
     * set weight of top-down contribution
     */
    inline bool setWeightTD(double w) { weightTD = w; return true; }
    
    /**
     * set weight of bottom-up contribution
     */
    inline bool setWeightBU(double w) { weightBU = w; return true; }

    /**
     * set color value of the top-down target
     */
    inline bool setTargetRed(double t)   { targetRed = t; return true; }
    
    /**
     * set color value of the top-down target
     */
    inline bool setTargetGreen(double t) { targetGreen = t; return true; }
    
    /**
     * set color value of the top-down target
     */
    inline bool setTargetBlue(double t)  { targetBlue = t; return true; }

    /**
     * get color value of the top-down contribution
     */
    inline int getTargetRG()   { return targetRG; }

    /**
     * get color value of the top-down contribution
     */
    inline int getTargetGR()   { return targetGR; }
    
    /**
     * get color value of the top-down contribution
     */
    inline int getTargetBY()   { return targetBY; }
    
    /**
     * get color value of the top-down contribution
     */
    inline int getTargetRed()   { return targetRed; }

    /**
     * get color value of the top-down contribution
     */
    inline int getTargetGreen()   { return targetGreen; }
    
    /**
     * get color value of the top-down contribution
     */
    inline int getTargetBlue()   { return targetBlue; }
 
    /**
     * @brief get rgb colour of the fovea blob 
     * @param redValue value extract from the red channel
     * @param greenValue value extracted from the green channel
     * @param blueValue value extracted from the blue channel
     */
    void getFoveaRgb(int& redValue, int& greenValue, int& blueValue);
};

#endif //__BLOBFINDERTHREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------
