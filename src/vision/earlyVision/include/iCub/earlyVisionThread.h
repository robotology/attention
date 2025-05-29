// -*- mode:C++; tab-width():4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Shashank Pathak
 * email:   francesco.rea@iit.it, shashank.pathak@iit.it
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
 * @file earlyVisionThread.h
 * @brief Definition of a thread that receives images and does the computation for the
 * early vision module via two other threads (see earlyVisionModule.h).
 */

#ifndef _VISUAL_FEATURE_THREAD_H_
#define _VISUAL_FEATURE_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Stamp.h>
/* Log-Polar includes */
#include <iCub/logpolar/RC_DIST_FB_logpolar_mapper.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/core/core_c.h>

#include <iCub/logPolar.h>

#include <iCub/convolve.h>
#include <iCub/centerSurround.h>
#include <iCub/chrominanceThread.h>
#include <iCub/edgesThread.h>
#include <iCub/config.h>


#define MONO_PIXEL_SIZE 1

// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif
 
using namespace yarp::sig;
class earlyVisionThread : public yarp::os::PeriodicThread  {
private:
    
    logpolarTransformVisual lpTrans;
    int width_orig, height_orig;        // dimension of the input image (original)
    int width, height;                  // dimension of the extended input image (extending)
    int width_cart, height_cart;        // dimension of the cartesian width and height    
    float lambda;                       // costant for the temporal filter
    double wHorizontal;                 // value of the weight of orizontal orientation
    double wVertical;                   // value of the weight of vertical orientation
    double w45Degrees;                  // value of the weight of 45 degrees orientation
    double wM45Degrees;                 // value of the weight of minus 45 degrees orientation    

    ImageOf<PixelRgb>* inputImage;
    ImageOf<PixelRgb>* filteredInputImage;
    ImageOf<PixelRgb>* extendedInputImage;
    
    ImageOf<PixelMono> *Rplus;
    ImageOf<PixelMono> *Rminus;
    ImageOf<PixelMono> *Gplus;
    ImageOf<PixelMono> *Gminus;
    ImageOf<PixelMono> *Bplus;
    ImageOf<PixelMono> *Bminus;
    ImageOf<PixelMono> *Yminus;
    ImageOf<PixelMono> *YofYUV;
    
    // these RGB planes are calculated via YUV, hence as float images rounded to uchar in last step
    ImageOf<PixelMono>* YofYUVpy;
    ImageOf<PixelMono>* UofYUVpy;
    ImageOf<PixelMono>* VofYUVpy;
    ImageOf<PixelMono>* RplusUnex;
    ImageOf<PixelMono>* GplusUnex;
    ImageOf<PixelMono>* BplusUnex;
    
    // a set of LUT for YUV to RGB conversion (on stack)
    //float YUV2RGB[3][256];
    //bool setYUV2RGB;

    ImageOf<PixelMono> *tmpMonoLPImage;
    ImageOf<PixelMono16> *tmpMono16LPImage;
    ImageOf<PixelMono16> *tmpMono16LPImage1;
    ImageOf<PixelMono16> *tmpMono16LPImage2;
    
    
    convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono> ,uchar >* gaborPosHorConvolution;
    convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono> ,uchar >* gaborPosVerConvolution;
    convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono> ,uchar >* gaborNegHorConvolution;
    convolve<ImageOf<PixelMono>,uchar,ImageOf<PixelMono> ,uchar >* gaborNegVerConvolution;
        
    ImageOf<PixelMono>* intensImg;              //yarp intensity image
    ImageOf<PixelMono>* unXtnIntensImg;              //yarp intensity image
    convolve<ImageOf<PixelFloat>,float,ImageOf<PixelFloat> ,float >* gaborFiveByFive[4];
    cv::Mat *gaborizedImg[GABOR_ORIS*GABOR_SCALES];                                        // assuming 4 orientations with 4 scales each
    ImageOf<PixelMono>* visualizingImage;
    
  
    ImageOf<PixelMono> *redPlane;             // image of the red channel
    ImageOf<PixelMono> *greenPlane;           // image of the green channel
    ImageOf<PixelMono> *bluePlane;            // image of the blue channel
    ImageOf<PixelMono> *yellowPlane;          // image of the yellow channel
    ImageOf<PixelMono> *Yplane;
    ImageOf<PixelMono> *Uplane;
    ImageOf<PixelMono> *Vplane;
    ImageOf<PixelMono> *unXtnYplane;
    ImageOf<PixelMono> *unXtnUplane;
    ImageOf<PixelMono> *unXtnVplane;

    cv::Mat *cs_tot_32f;  // extended
    cv::Mat *cs_tot_8u;
    cv::Mat *ycs_out;     // final extended intensity center surround image
    cv::Mat *scs_out;     // final extended intensity center surround image
    cv::Mat *vcs_out;     // final extended intensity center surround image
    cv::Mat *colcs_out;   // final extended coulour center surround image
    
    
    CenterSurround *centerSurr;    


    yarp::os::BufferedPort<ImageOf<PixelRgb> > imagePortIn;
    yarp::os::BufferedPort<ImageOf<PixelRgb> > imagePortOut;
    yarp::os::BufferedPort<ImageOf<PixelMono> > intenPort;
    yarp::os::BufferedPort<ImageOf<PixelMono> > intensityCSPort;
    yarp::os::BufferedPort<ImageOf<PixelMono> > chromPort;
    yarp::os::BufferedPort<ImageOf<PixelMono> > VofHSVPort;
        
    yarp::os::BufferedPort<ImageOf<PixelMono> > colorOpp1Port;
    yarp::os::BufferedPort<ImageOf<PixelMono> > colorOpp2Port;
    yarp::os::BufferedPort<ImageOf<PixelMono> > colorOpp3Port;
        
    bool isYUV;   
    
    yarp::os::Stamp St;

    
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized   
    
public:
    /**
    * constructor
    */
    earlyVisionThread();

    /**
     * destructor
     */
    ~earlyVisionThread();

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
    * function that set the value for the weight horizontal orientation in linear combination
    * @param value double value of the weight
    */
    void setWHorizontal(double value) { wHorizontal = value; };

    /**
    * function that set the value for the weight vertical orientation in linear combination
    * @param value double value of the weight
    */
    void setWVertical(double value) { wVertical = value; };

    /**
    * function that set the value for the weight 45 degrees orientation in linear combination
    * @param value double value of the weight
    */
    void setW45Degrees(double value) { w45Degrees = value; };

    /**
    * function that set the value for the weight minus 45 degrees orientation in linear combination
    * @param value double value of the weight
    */
    void setWM45Degrees(double value) { wM45Degrees = value; };
    
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
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param origImage originalImage
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    void extender(ImageOf<PixelMono>* origImage,int extDimension);

    /**
    * Center-surrounding
    */
    void centerSurrounding();  

     /**
    * function that maps logpolar image to cartesian
    * @param cartesianImage cartesian image to remap
    * @param logpolarImage  result of the remapping
    */
    void cartremap(ImageOf<PixelRgb>* cartesianImage,ImageOf<PixelRgb>* logpolarImage);

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
    void addFloatImage(cv::Mat* sourceImage, cv::Mat* toBeAddedImage, double multFactor, double shiftFactor);
      
    
    edgesThread *edThread;                 // thread that extract edges
    chrominanceThread *chromeThread;       // thread that extract orientation information 
    
};

#endif  //_VISUAL_FEATURE_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

