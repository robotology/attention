// -*- mode:C++; tab-width():4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file opticFlowComputer.h
 * @brief Definition of a thread that receives images and does the computation for the
 * early vision module via two other threads (see earlyVisionModule.h).
 */

#ifndef _OPTIC_FLOW_COMPUTER_H_
#define _OPTIC_FLOW_COMPUTER_H_

#include <iostream>
#include <stdlib.h>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Semaphore.h>
#include <yarp/sig/Matrix.h>
#include <yarp/math/SVD.h>
#include <yarp/math/Math.h>

/* Log-Polar includes */
#include <iCub/RC_DIST_FB_logpolar_mapper.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

//#include <iCub/logPolar.h>
#include <iCub/convolve.h>
#include <iCub/config.h>
#include <iCub/plotterThread.h>


//#define PI 3.1415
#define Na 252
#define MONO_PIXEL_SIZE 1

// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif
 

class opticFlowComputer : public yarp::os::Thread  {
private:
    int id;                             // identification number of the computer
    int posXi, posGamma;                // center position of the processing
    int neigh;                          // dimension of the neighborhood
   
    int width, height;                  // dimension of the computation domain of the computer
    int rowSize;                        // row size of the image
    int calculusRowSize;                // row size of the calculus image
        
    float lambda;                       // costant for the temporal filter
    double wHorizontal;                 // value of the weight of orizontal orientation
    double wVertical;                   // value of the weight of vertical orientation
    double w45Degrees;                  // value of the weight of 45 degrees orientation
    double wM45Degrees;                 // value of the weight of minus 45 degrees orientation    

    double a,F,q,rho0;                  // variable of the computation
    int halfNeigh;                      // half of the neighboorhood pixels
    int calcHalf;                       // half of the calculus area
    int gammaStart, gammaEnd;           // gamma limits
    int xiStart, xiEnd;                 // xi value limits
    int dimComput;                      // dimension of the image area of any computer

    yarp::sig::Matrix *Grxi;            // gradient along the xi direction
    yarp::sig::Matrix *Grgamma;         // gradient along the gamma axis
    yarp::sig::Matrix *Grt;             // temporal gradient
    yarp::sig::Matrix *H,*G;
    yarp::sig::Matrix *s;
    yarp::sig::Matrix *B,*A,*K,*V;      // matrix of the tranformation from opticflow in log to opticflow in cart
    yarp::sig::Matrix *Kt,*Km;
    yarp::sig::Matrix *c,*b,*bwMat;
    yarp::sig::Matrix *u,*v;
    yarp::sig::Matrix *wMat;
    yarp::sig::Vector *S;
    yarp::sig::Matrix *of;
    
    //static double const q       = 0.5 * (Na / PI);
    static double const qdouble = Na / PI;

    float fcos[262];                    // LUT of cos(gamma)
    float fsin[262];                    // LUT of sin(gamma)

    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* filteredInputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* extendedInputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *represenImage;

    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gradientHorConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gradientVerConvolution;
        
    yarp::sig::ImageOf<yarp::sig::PixelMono>* intensImg;              //yarp intensity image

    IplImage *represenIpl;   // image that is going to be represented

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imagePortOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > intenPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > intensityCSPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > chromPort;
        
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > colorOpp1Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > colorOpp2Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > colorOpp3Port;   


    //yarp::os::Semaphore* semCalculusX;    // semaphore that controls access to the assigned portion of image
    //yarp::os::Semaphore* semCalculusY;    // semaphore that controls access to the assigned portion of image

    yarp::os::Semaphore* semCalculus;     // semaphore that controls access to the assigned portion of image
    yarp::os::Semaphore* semRepresent;    // semaphore that controls access to the assigned portion of image
    yarp::os::Semaphore* semTemporal;     // semaphore that controls access to the assigned portion of image

    plotterThread* pt;                    // reference to the plotter 

    //unsigned char* calculusPointerX;    // pointer to the image which the computation takes place from
    //unsigned char* calculusPointerY;    // pointer to the image which the computation takes place from
    unsigned char* calculusPointer;       // pointer to the image which the computation takes place from
    unsigned char* represPointer;         // pointer to the image which the flow is represented
    unsigned char* temporalPointer;       // pointer to the previous monocromatic image    
        
    bool isYUV;   
    FILE *fout;
     
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized   
    bool hasStartedFlag;    // flag that indicates whether the thread has started
    
public:
    /**
    * default constructor
    */
    opticFlowComputer();

    /**
    * constructor
    * @param posXi position of the computer in Xi axis
    * @param posGamma position of the computer in the gamma axis
    * @param neighborhood dimension of the neighborhood
    */
    opticFlowComputer(int id, int posXi,int posyGamma,int neighborhood);

    /**
     * destructor
     */
    ~opticFlowComputer();

    bool threadInit();     
    void threadRelease();
    void run(); 
    void onStop();

    /**
    * function that set the rootname for the ports that will be opened
    * @param str rootname as a string
    */
    void setName(std::string str);
    
    /**
    * function that returns the original root name and appends another string iff passed as parameter
    * @param p pointer to the string that has to be added
    * @return rootname 
    */
    std::string getName(const char* p);

    /**
    * function that resizes the necessary and already allocated images
    * @param width width of the input image
    * @param height height of the input image
    */
    void resize(int width, int height);
    
    /**
    * function that resizes the cartesian image
    * @param width width of the input image
    * @param height height of the input image
    */
    void resizeCartesian(int width, int height);

    /**
     * @brief function that indicates whether the thread has started
     */
    bool hasStarted() { return hasStartedFlag; };

    /**
     * @brief function that sets the hasStarted flag
     */
    void setHasStarted(bool value) { hasStartedFlag =  value; };

    /*
    void setCalculusPointerX(unsigned char* pImage){ calculusPointerX = pImage; };
    void setCalculusPointerY(unsigned char* pImage){ calculusPointerY = pImage; };
    */

    /**
     * @brief function that sets the pointer to the plotter
     */
    void setPlotterPointer(plotterThread* p){ pt = p; };

    /**
     * @brief function that declares which image the computer is working on
     */
    void setTemporalPointer(unsigned char* pImage){ temporalPointer = pImage; };

    /**
     * @brief function that declares which image the computer is working on
     */
    void setCalculusPointer(unsigned char* pImage){ calculusPointer = pImage; };

    
    /**
     * @brief function that declares which image the computer is working on
     */
    void setCalculusRowSize(int value){ calculusRowSize = value; };

    /**
     * @brief function that declares which image the computer is working on
     */
    void setRepresenPointer(unsigned char* pImage){ represPointer = pImage; };

    /**
     * @brief set a representation pointer of type IplImage
     */
    void setRepresenImage(yarp::sig::ImageOf<yarp::sig::PixelRgb>* image) ;  

    /*
   
    void setCalculusXSem(yarp::os::Semaphore* sem) { semCalculusX = sem; };

   
    void setCalculusYSem(yarp::os::Semaphore* sem) { semCalculusY = sem; };
    */

    /**
     * @brief function that associate a semaphore to the portion of image where computiong
     */
    void setCalculusSem(yarp::os::Semaphore* sem) { semCalculus = sem; };
    

    /**
     * @brief function that associate a semaphore to the portion of image to represent
     */
    void setRepresentSem(yarp::os::Semaphore* sem) { semRepresent = sem; };

    /**
     * @brief function that associate a semaphore to the previoud monocromatic input image
     */
    void setTemporalSem(yarp::os::Semaphore* sem) { semRepresent = sem; };
   
    /**
     * @brief represent information directly in the image
     */
    //void setRepresenImage(IplImage* i) {represenImage = i;};

    /**
     * @brief estimate the optical flow 
     */
    void estimateOF();
    
    /**
     * @brief represent the optical flow 
     */
    void representOF();

    /**
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    void extender(int extDimension); 

    /**
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param origImage originalImage
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    void extender(yarp::sig::ImageOf<yarp::sig::PixelMono>* origImage,int extDimension);

    /**
    * Center-surrounding
    */
    void centerSurrounding();  

     /**
    * function that maps logpolar image to cartesian
    * @param cartesianImage cartesian image to remap
    * @param logpolarImage  result of the remapping
    */
    void cartremap(yarp::sig::ImageOf<yarp::sig::PixelRgb>* cartesianImage,yarp::sig::ImageOf<yarp::sig::PixelRgb>* logpolarImage);

    /**
    * function that filters the input image in time 
    */
    void filterInputImage();

    /**
    * extracting RGB and Y planes
    */
    void extractPlanes();

    /**
    * gaussing filtering of the of image planes extracted
    */
    void filtering();

   
    /**
    * Creating color opponency maps
    */
    void colorOpponency(); 
    
    /**
    * Adding two images in-place with an element-wise weightage factor and shift factor A(I) = A(I) + multFactor.*B(I) .+ shiftFactor
    * @param sourceImage to which other image will be added
    * @param toBeAddedImage the image which will be added
    * @param multFactor factor of multiplication
    * @param shiftFactor value added to each pixel
    */
    void addFloatImage(IplImage* sourceImage, CvMat* toBeAddedImage, double multFactor, double shiftFactor);
      
    
    
};

#endif  //_OPTIC_FLOW_COMPUTER_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

