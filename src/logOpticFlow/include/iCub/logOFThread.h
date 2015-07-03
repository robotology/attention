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
 * @file logOFThread.h
 * @brief Definition of a thread that receives images and does the computation for the
 * early vision module via two other threads (see earlyVisionModule.h).
 */

#ifndef _LOG_OF_THREAD_H_
#define _LOG_OF_THREAD_H_

#include <iostream>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/Stamp.h>

/* Log-Polar includes */
#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <iCub/convolve.h>
#include <iCub/config.h>
#include <iCub/opticFlowComputer.h>
#include <iCub/plotterThread.h>

#define MONO_PIXEL_SIZE 1
#define COUNTCOMPUTERSX 21  //21 columns
#define COUNTCOMPUTERSY 13  //13 rows

// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif

//#define DEBUG_OPENCV
 
class logOFThread : public yarp::os::Thread  {
private:
    int count;
    
    int width_orig, height_orig;        // dimension of the input image (original)
    int width, height;                  // dimension of the extended input image (extending)
    int width_cart, height_cart;        // dimension of the cartesian width and height    
    float lambda;                       // costant for the temporal filter

    opticFlowComputer* ofComputer[COUNTCOMPUTERSX * COUNTCOMPUTERSY];   // array of optic flow computers
    yarp::os::Semaphore** calcXSem;     // array of semaphore for calculus image gradient X
    yarp::os::Semaphore** calcYSem;     // array of semaphore for calculus image gradient X
    yarp::os::Semaphore** calcSem;     // array of semaphore for calculus image gradient X
    yarp::os::Semaphore** reprSem;      // array of semaphore for representation
    yarp::os::Semaphore** tempSem;      // array of semaphore for temporal representation of images
    
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* outputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* flowImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* finalOutputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* filteredInputImage;
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* extendedInputImage;
    
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Rplus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Rminus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Gplus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Gminus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Bplus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Bminus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Yminus;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *YofYUV;
    
    // these RGB planes are calculated via YUV, hence as float images rounded to uchar in last step
    yarp::sig::ImageOf<yarp::sig::PixelMono>* YofYUVpy;
    yarp::sig::ImageOf<yarp::sig::PixelMono>* UofYUVpy;
    yarp::sig::ImageOf<yarp::sig::PixelMono>* VofYUVpy;
    yarp::sig::ImageOf<yarp::sig::PixelMono>* RplusUnex;
    yarp::sig::ImageOf<yarp::sig::PixelMono>* GplusUnex;
    yarp::sig::ImageOf<yarp::sig::PixelMono>* BplusUnex;
    
    // a set of LUT for YUV to RGB conversion (on stack)
    //float YUV2RGB[3][256];
    //bool setYUV2RGB;
    
    yarp::sig::ImageOf<yarp::sig::PixelMono> *tmpMonoLPImage;      // temporary mono logpolar image

    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborPosHorConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborPosVerConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborNegHorConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gaborNegVerConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gradientHorConvolution;
    convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelMono> ,uchar >* gradientVerConvolution;
    
    yarp::sig::ImageOf<yarp::sig::PixelMono>* intensImg;            //yarp intensity image
    yarp::sig::ImageOf<yarp::sig::PixelMono>* intensXGrad;          //yarp intensity image
    yarp::sig::ImageOf<yarp::sig::PixelFloat>*intensYGrad;          //yarp intensity image
    yarp::sig::ImageOf<yarp::sig::PixelMono>* intXgrad8u;           //yarp gradientX image 8u
    yarp::sig::ImageOf<yarp::sig::PixelMono>* intYgrad8u;           //yarp gradientY image 8u

    yarp::sig::ImageOf<yarp::sig::PixelMono> *gradientImgXCopy;     //copy of intensity image    
    yarp::sig::ImageOf<yarp::sig::PixelMono> *gradientImgYCopy;     //copy of intensity image    
    yarp::sig::ImageOf<yarp::sig::PixelMono> *intensImgCopy;     //copy of intensity image    

    yarp::sig::ImageOf<yarp::sig::PixelMono>* prevIntensImg;        //intensity image at the previous temporal frame
    yarp::sig::ImageOf<yarp::sig::PixelMono>* unXtnIntensImg;       //yarp intensity image
      
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane;             // image of the red channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane;           // image of the green channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane;            // image of the blue channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane;          // image of the yellow channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Yplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Uplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *Vplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *unXtnYplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *unXtnUplane;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *unXtnVplane;

    IplImage *cs_tot_32f;          // extended
    IplImage *int_gradx_32f;
    IplImage *cs_tot_8u; 
    IplImage *ycs_out;             // final extended intensity center surround image
    IplImage *scs_out;             // final extended intensity center surround image
    IplImage *vcs_out;             // final extended intensity center surround image
    IplImage *colcs_out;           // final extended coulour center surround image
    IplImage *img;
    IplImage *calculusIpl;         //Ipl image for the calculus
    IplImage *temporalIpl;         //Ipl image for temporaral gradient
    IplImage *calculusIpl32f;      //Ipl image for the calculus 32float
    IplImage *temporalIpl32f;      //Ipl image for temporaral gradient 32float
    IplImage *calculusIpl32f_copy; //Ipl image for the calculus 32float
    IplImage *temporalIpl32f_copy; //Ipl image for temporaral gradient 32float
    

    //CenterSurround *centerSurr;    

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  imagePortIn;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  imagePortOut;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> >  flowPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > intenPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > intensityCSPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > chromPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > VofHSVPort;  
    
        
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > colorOpp1Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > colorOpp2Port;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > colorOpp3Port;   
        
    bool isYUV;   
    
    yarp::os::Stamp St;
    plotterThread* pt;                    // thread in charge of plotting the flow
    
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized   
    
public:
    /**
    * constructor
    */
    logOFThread();

    /**
     * destructor
     */
    ~logOFThread();

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
     * @brief reinstantiate an convolve object with a new parameter
     */
    void setGradientHoriz(double value) {
        delete gradientHorConvolution;
        gradientHorConvolution = new convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,
            uchar,yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar>
            (3,3,Sobel2DXgrad_small,value,-50,0);
    };
   
    /**
     * @brief reinstantiate an convolve object with a new parameter
     */
    void setGradientVert(double value, int bias) {
        delete gradientVerConvolution;
        gradientVerConvolution = new convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,
            uchar,yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar>
            (3,3,Sobel2DYgrad_small,value,bias,0);
    };
    
    /**
     * @brief reinstantiate an convolve object with a new parameter
     */
    void setGradientHoriz(double value, int bias) {
        delete gradientHorConvolution;
        gradientHorConvolution = new convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,
            uchar,yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar>
            (3,3,Sobel2DXgrad_small,value,bias,0);
    };
   
    /**
     * @brief reinstantiate an convolve object with a new parameter
     */
    void setGradientVert(double value) {
        delete gradientVerConvolution;
        gradientVerConvolution = new convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,
            uchar,yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar>
            (3,3,Sobel2DYgrad_small,value,-50,0);
    };
    
    /**
     * @brief function that converts images into ipl32f
     */
    void convertImages(yarp::sig::ImageOf<yarp::sig::PixelMono> *srcInt, yarp::sig::ImageOf<yarp::sig::PixelMono> *srcTemp );
    
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
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    void extender(int extDimension); 

    /**
     * @brief function that force on wait all the semaphores in a list 
     */
    void waitSemaphores(yarp::os::Semaphore** pointer);

    /**
     * @brief function that force on post all the semaphores in a list 
     */
    void postSemaphores(yarp::os::Semaphore** pointer);
    
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
      
    
    /**
     * @brief function that initialise a specific optic flow computer
     * @param index id number of the computer to be initialised
     */
    void initFlowComputer(int index);

    
    //edgesThread *edThread;                 // thread that extract edges
    //chrominanceThread *chromeThread;       // thread that extract orientation information 
    
};

#endif  //_LOG_OF_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

