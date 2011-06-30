// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file visualFilterThread.h
 * @brief Definition of a thread that receives images and does the computation for the
 * visual filter module (see visualFilterModule.h).
 */

#ifndef _VISUAL_FEATURE_THREAD_H_
#define _VISUAL_FEATURE_THREAD_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Stamp.h>
/* Log-Polar includes */
#include <iCub/RC_DIST_FB_logpolar_mapper.h>
//#include <yarp/sig/IplImage.h>

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
//#include <Eigen/Dense>


#define KERNEL_ROW 7
#define KERNEL_COL 7
#define CHAR_LIMIT 256
#define PI 3.1415926535897932384626433832795
#define MONO_PIXEL_SIZE 1
#define DWN_SAMPLE 2
#define DWN_SAMPLE2 4
#define NBR_OF_FILTERS 4

// patches for now
#ifndef YARP_IMAGE_ALIGN
#define YARP_IMAGE_ALIGN 8
#endif

// Some general purpose definitions for logpolar transformation
enum {
    RADIAL = 0, /** \def RADIAL Each receptive field in fovea will be tangent to a receptive field in the previous ring and one in the next ring */
    TANGENTIAL = 1, /** \def TANGENTIAL Each receptive field in fovea will be tangent to the previous and the next receptive fields on the same ring. */
    ELLIPTICAL = 2 /** \def ELLIPTICAL Each receptive field in fovea will be tangent to all its neighbors, having then an elliptical shape. */
};

enum {
   C2L = 1,     // 2^0
   L2C = 2,     // 2^1
   BOTH = 3,    // 2^0 + 2^1
};

/**
 * \struct cart2LpPixel 
 * \brief It contains the look-up table for the creation of a log polar image. 
 *
 */
struct cart2LpPixel
{
    int divisor;    /**< Number of cartesian pixels corresponding to the current log polar one.*/
    int *iweight;   /**< Array containing the weight of each cartesian pixel.*/
    int *position;  /**< Array containing the position of each cartesian pixel. \n
                         Note that the plane information is included in this field 
                         (i.e. when only one plane is present it contains
                         the value  (\p y*xSize+x), while in case of three planes, 
                         the value will be \p 3*(\p y*xize+x) ).*/
};

/**
 * \struct lp2CartPixel 
 * \brief It contains the look-up table for the remapping of a log polar image into a cartesian one. 
 *
 */
struct lp2CartPixel
{
    int iweight;    /**< Number of pixels in the array of positions */
    int *position;  /**< Array containing the cartesian position of each log 
                         polar pixel. \n
                         Note that the plane information is included in this 
                         field (i.e. the value will be \p 3*(\p y*xize+x) ).*/
};

/**
 * replicate borders on a logpolar image before filtering (similar in spirit to IPP or OpenCV replication).
 * @param dest is the image with replicated borders of size w+2*maxkernelsize, h+maxkernelsize
 * @param src is the input image
 * @param maxkernelsize is half of the kernel size (i.e. ceil(kernel/2))
 * @return true iff the replication is possible (images must be correctly sized and allocated)
 */
bool replicateBorderLogpolar(yarp::sig::Image& dest, const yarp::sig::Image& src, int maxkernelsize);

/**
 * make an image of the fovea starting from the original cartesian image.
 * @param dest is the foveal image (the size of the fovea is given by the destination image size
 * @param src is the input image
 * @return true iff the image sizes are compatible with the operation requested
 */
bool subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelRgb>& dst, const yarp::sig::ImageOf<yarp::sig::PixelRgb>& src);

// End of general purpose definitions for logpolar transformations


// Class for log polar transformation, as in logpolar library (RC_DIST_FB_logpolar_mapper)
class logpolarTransformVisual {
private:
    cart2LpPixel *c2lTable;
    lp2CartPixel *l2cTable;
    int necc_;
    int nang_;
    int width_;
    int height_;
    int mode_;

    /*
    * \arg \b 0 All the RF's are tangent each other. 
    * \arg \b -1 All their sizes are 1 pixel
    * \arg \b 1 The edges of the RF's pass through the centers of neighboring RF's
    * Allowed values: float numbers between -1 and +infinity
    */
    double overlap_;

    // forbid copies.
    logpolarTransformVisual(const logpolarTransformVisual& x);
    void operator=(const logpolarTransformVisual& x);

    /**
    * \brief Frees the memory from the look-up table.
    */
    void RCdeAllocateC2LTable ();

    /**
    * \brief Frees the memory from the look-up table. 
    */
    void RCdeAllocateL2CTable ();

    /**
    * \brief Generates the look-up table for the transformation from a cartesian image to a log polar one, both images are color images
    * @param scaleFact the ratio between the size of the smallest logpolar pixel and the cartesian ones
    * @param mode is one of the following : RADIAL, TANGENTIAL or ELLIPTICAL
    * @param padding is the input image row byte padding (cartesian image paddind)
    * @return 0 when there are no errors
    * @return 1 in case of wrong parameters
    * @return 2 in case of allocation problems
    */
    int RCbuildC2LMap (double scaleFact, int mode, int padding);

    /**
    * \brief Generates the look-up table for the transformation from a log polar image to a cartesian one.
    * @param scaleFact the ratio between the size of the smallest logpolar pixel and the cartesian ones
    * @param hOffset is the horizontal shift in pixels
    * @param vOffset is the vertical shift in pixels
    * @param mode is one of the following : RADIAL, TANGENTIAL or ELLIPTICAL
    * @param padding is the number of pad bytes of the input image (logpolar)
    * @return 0 when there are no errors
    * @return 1 in case of wrong parameters
    * @return 2 in case of allocation problems
    */
    int RCbuildL2CMap (double scaleFact, int hOffset, int vOffset, int mode, int padding);

    /**
    * \brief Generates a log polar image from a cartesian one
    * @param lpImg is the output LogPolar image
    * @param cartImg is the input Cartesian image
    * @param Table is the LUT used for the transformation
    * @param padding is the padding of the logpolar image (output)
    is generated otherways
    */
    void RCgetLpImg (unsigned char *lpImg,
                     unsigned char *cartImg,
                     cart2LpPixel * Table, 
                     int padding);

    /**
    * \brief Remaps a log polar image to a cartesian one
    * @param cartImg is the output Cartesian image
    * @param lpImg is the input LogPolar image
    * @param Table is the LUT used for the transformation
    * @param cartSize is the size of the log polar image
    */
    void RCgetCartImg (unsigned char *cartImg, unsigned char *lpImg, lp2CartPixel * Table, int cartSize);

    /**
    * \brief Computes the logarithm index
    * @param nAng is the number of pixels per ring 
    * @return the logarithm index.
    */
    double RCgetLogIndex ();

    /**
    * \brief Computes the ratio between the size of the smallest logpolar pixel and the cartesian ones
    * @return the scale factor.
    */
    double RCcomputeScaleFactor ();

public:
    /**
     * default constructor.
     */
    logpolarTransformVisual() {
        c2lTable = 0;
        l2cTable = 0;
        necc_ = 0;
        nang_ = 0;
        width_ = 0;
        height_ = 0;
        overlap_ = 0.;
        mode_ = BOTH;
    }

    /** destructor */
    virtual ~logpolarTransformVisual() {
        freeLookupTables();
    }

    /** 
     * check whether the LUT have been previously allocated.
     * @return true iff one or both LUTs are different from zero.
     */
    virtual const bool allocated() const {
        if (c2lTable != 0 || l2cTable != 0)
            return true;
        else
            return false;
    }

    /**
     * alloc the lookup tables and stores them in memory.
     * @param necc is the number of eccentricities of the logpolar image.
     * @param nang is the number of angles of the logpolar image.
     * @param w is the width of the original rectangular image.
     * @param h is the height of the original rectangular image.
     * @param overlap is the degree of overlap of the receptive fields (>0.).
     * @return true iff successful.
     */
    virtual bool allocLookupTables(int mode = BOTH, int necc = 152, int nang = 252, int w = 640, int h = 480, double overlap = 1.);

    /**
    * free the lookup tables from memory.
    * @return true iff successful.
    */
    virtual bool freeLookupTables();

    /**
     * converts an image from rectangular to logpolar.
     * @param lp is the logpolar image (destination).
     * @param cart is the cartesian image (source data).
     * @return true iff successful. Beware that tables must be
     * allocated in advance.
     */
    virtual bool cartToLogpolar(yarp::sig::ImageOf<yarp::sig::PixelMono>& lp, 
                                const yarp::sig::ImageOf<yarp::sig::PixelMono>& cart);

    /**
     * converts an image from logpolar to cartesian (rectangular).
     * @param cart is the cartesian image (destination).
     * @param lp is the logpolar image (source).
     * @return true iff successful. Beware that tables must be
     * allocated in advance.
     */
    virtual bool logpolarToCart(yarp::sig::ImageOf<yarp::sig::PixelMono>& cart,
                                const yarp::sig::ImageOf<yarp::sig::PixelMono>& lp);

    /**
     * check the number of eccentricities (rings).
     * @return the number of rings in the logpolar mapping (default 152).
     */
    int necc(void) const { return necc_; }

    /**
     * check the number of angles (radii).
     * @return the number of radii in the logpolar mapping (default 252).
     */
    int nang(void) const { return nang_; }

    /**
     * check the width of the original or remapped cartesian image.
     * @return the width of the cartesian image (defualt = 640).
     */
    int width(void) const { return width_; }

    /**
     * check the height of the original or remapped cartesian image.
     * @return the height of the cartesian image (default = 480).
     */
    int height(void) const { return height_; }

    /**
     * return the desired overlap btw receptive fields.
     * @return the desired overlap (default = 1.0).
     */
    double overlap(void) const { return overlap_; }

    /**
     * return the operating mode, one of BOTH, C2L, L2C.
     * @return the value of mode (default = BOTH).
     */
    int mode(void) const { return mode_; }

    /**
     * computes the ratio between the size of the smallest logpolar pixel and the cartesian one.
     * WARNING: this method is deprecated and maintained only for compatibility with old code.
     * @return the scale factor.
     */
    double computeScaleFactor(void) {
        return RCcomputeScaleFactor();
    }

};

// end of log polar transformation class



class visualFilterThread : public yarp::os::Thread 
{
private:
    

    int psb;
    int width_orig, height_orig;        // dimension of the input image (original)
    int width, height;                  // dimension of the extended input image (extending)
    int width_cart, height_cart;        // dimension of the cartesian width and height
    int size1;                          // size of the buffer
    int psb16s;                         // step size of the Ipp16s vectors
    float lambda;                       // costant for the temporal filter
   
    // parameters for Gabor filter
    double sigma[NBR_OF_FILTERS];
    double gLambda[NBR_OF_FILTERS];
    double psi[NBR_OF_FILTERS];
    double gamma[NBR_OF_FILTERS];
    double filScale[NBR_OF_FILTERS];
    double filShift[NBR_OF_FILTERS];

    int intSigma[NBR_OF_FILTERS];
    int intLambda[NBR_OF_FILTERS];
    int intPsi[NBR_OF_FILTERS];
    int intGamma[NBR_OF_FILTERS];
    int intFilScale[NBR_OF_FILTERS];
    int intFilShift[NBR_OF_FILTERS];
    
    double dwnSam,whichScale;
    int kernelUsed;
    //int kernelSize[2];
    CvMat* gabKer[4];

    yarp::sig::ImageOf<yarp::sig::PixelMono> *yImage;               // y of YUV image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yFilImage;               // y of YUV image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *uvOrigImage;              // uv of YUV image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *uvImage;              // extended uv of YUV image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *uvFilImage;              // extended uv of YUV image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *cartImage; 
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *inputImageFiltered;    // time filtered input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb> *logPolarImage;
    
    yarp::sig::ImageOf<yarp::sig::PixelMono>* intensImg;              //yarp intensity image
    yarp::sig::ImageOf<yarp::sig::PixelMono>* cartIntensImg;              //yarp cartesian intensity image 
  
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane;             // image of the red channel
    IplImage *cvRedPlane;
    //yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane2;
    //yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlane3;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane;           // image of the green channel
     IplImage *cvGreenPlane;
    //yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane2;
    //yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlane3;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane;            // image of the blue channel
     IplImage *cvBluePlane;
    //yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane2;           // image of the blue channel
    //yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlane3;           // image of the blue channel
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane;          // image of the yellow channel
     IplImage *cvYellowPlane;
    //yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowPlane2;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *redPlus;              // positive gaussian-convolved red image 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *redMinus;             // negative gaussian-convolved red image
    IplImage *cvRedPlus, *tmpRedPlus;
    IplImage *cvRedMinus, *tmpRedMinus;
    CvMat *kernel;
    double gK[4][KERNEL_ROW][KERNEL_COL];


    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenPlus;            // positive gaussian-convolved green image 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenMinus;           // negative gaussian-convolved green image
    IplImage *cvGreenPlus, *tmpGreenPlus;
    IplImage *cvGreenMinus, *tmpGreenMinus;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *bluePlus;             // positive gaussian-convolved red image 
    yarp::sig::ImageOf<yarp::sig::PixelMono> *yellowMinus;          // negative gaussian-convolved red image
    IplImage *cvBluePlus, *tmpBluePlus;
    IplImage *cvYellowMinus, *tmpYellowMinus;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *redGreen;             // colour opponency map (R+G-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *cartRedGreen;             // colour opponency map (R+G-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *upSampleRGyarp;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *greenRed;             // colour opponency map (G+R-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *cartGreenRed;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *upSampleGRyarp;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *blueYellow;           // colour opponency map (B+Y-)
    yarp::sig::ImageOf<yarp::sig::PixelMono> *cartBlueYellow;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *upSampleBYyarp;
    yarp::sig::ImageOf<yarp::sig::PixelMono> *edges;                // edges of colour opponency maps 
    
    yarp::sig::ImageOf<yarp::sig::PixelMono> *gabor0;               // image with Gabor filter oriented to zero
    yarp::sig::ImageOf<yarp::sig::PixelMono> *gabor45;              // image with Gabor filter oriented to 45 deg
    yarp::sig::ImageOf<yarp::sig::PixelMono> *gabor90;              // image with Gabor filter oriented to 90 deg
    yarp::sig::ImageOf<yarp::sig::PixelMono> *gaborM45;             // image with Gabor filter oriented to -45 deg

    IplImage *cvGabor0, *cvGabor45, *cvGabor90, *cvGaborM45;
    

    IplImage* redG;
    IplImage* greenR;
    IplImage* blueY;
    IplImage* totImg;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > yPortIn;           // Y component from YUV module
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > uvPortIn;          // UV component from YUV module
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imagePortExt;      // extended uv image port
    
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > gaborPort0;               // port for Gabor filtered image with orientation 0
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > gaborPort45;              // port for Gabor filtered image with orientation
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > gaborPort90;              // port for Gabor filtered image with orientation
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > gaborPortM45;             // port for Gabor filtered image with orientation

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > pyImgPort;        // image after pyramid approach

    //IplImage for horizontal and vertical components after Sobel operator on color opponents
    IplImage* hRG;
    IplImage* vRG;
    IplImage* hGR;
    IplImage* vGR;
    IplImage* hBY;
    IplImage* vBY;

    //16 bit image to avoid overflow in Sobel operator
    IplImage* tempHRG;
    IplImage* tempVRG;
    IplImage* tempHGR;
    IplImage* tempVGR;
    IplImage* tempHBY;
    IplImage* tempVBY;

    //down-sampled and up-sampled images for applying Gabor filter 
    IplImage* dwnSampleRG;
    IplImage* dwnSampleGR;
    IplImage* dwnSampleBY;
    IplImage* dwnSampleRGFil;
    IplImage* dwnSampleGRFil;
    IplImage* dwnSampleBYFil;
    IplImage* upSampleRG;
    IplImage* upSampleGR;
    IplImage* upSampleBY;

    /******************/
    //down-sampled and up-sampled images for applying Gabor filter 
    IplImage* dwnSampleRGa;
    IplImage* dwnSampleGRa;
    IplImage* dwnSampleBYa;
    IplImage* dwnSampleRGFila;
    IplImage* dwnSampleGRFila;
    IplImage* dwnSampleBYFila;
    IplImage* upSampleRGa;
    IplImage* upSampleGRa;
    IplImage* upSampleBYa;
    //down-sampled and up-sampled images for applying Gabor filter 
    IplImage* dwnSampleRGb;
    IplImage* dwnSampleGRb;
    IplImage* dwnSampleBYb;
    IplImage* dwnSampleRGFilb;
    IplImage* dwnSampleGRFilb;
    IplImage* dwnSampleBYFilb;
    IplImage* upSampleRGb;
    IplImage* upSampleGRb;
    IplImage* upSampleBYb;

    
    iCub::logpolar::logpolarTransform trsf; //reference to the converter for logpolar transform
    logpolarTransformVisual lpMono;
    int xSizeValue;         // x dimension of the remapped cartesian image
    int ySizeValue;         // y dimension of the remapped cartesian image
    double overlap;         // overlap in the remapping
    int numberOfRings;      // number of rings in the remapping
    int numberOfAngles;     // number of angles in the remapping
    int orient0[2];
    int orient45[2];
    int orient90[2];
    int orientM45[2];
    

    /*** To convert to cartesian **/
    IplImage* tmpRedGreen;
    IplImage* tmpGreenRed;
    IplImage* tmpBlueYellow;

    IplImage* intensityImage;
    IplImage* filteredIntensityImage;
    IplImage* filteredIntensityImage1;
    IplImage* filteredIntensityImage2;
    IplImage* totImage;
    IplImage* dwnImage;

    yarp::os::Stamp St;

    
    std::string name;       // rootname of all the ports opened by this thread
    bool resized;           // flag to check if the variables have been already resized

    int minVal;

public:
    /**
    * constructor
    */
    visualFilterThread();

    /**
     * destructor
     */
    ~visualFilterThread();

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
    * function that extendes the original image of the desired value for future convolutions (in-place operation)
    * @param origImage originalImage
    * @param extDimension dimension of the extention on each of the sides of the image
    */
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* extender(yarp::sig::ImageOf<yarp::sig::PixelRgb>* origImage,int extDimension); 

    yarp::sig::ImageOf<yarp::sig::PixelMono>* extender(yarp::sig::ImageOf<yarp::sig::PixelMono>* origImage,int extDimension);

     /**
    * function that maps logpolar image to cartesian
    * @param cartesianImage cartesian image to remap
    * @param logpolarImage  result of the remapping
    */
    void cartremap(yarp::sig::ImageOf<yarp::sig::PixelRgb>* cartesianImage,yarp::sig::ImageOf<yarp::sig::PixelRgb>* logpolarImage);

    /**
    * extracting RGB and Y planes
    */
    void extractPlanes();

    /**
    * function that filters the input image in time 
    */
    void filterInputImage();

    /**
    * gaussing filtering of the of RGBY
    */
    void filtering();

    /**
    * function which does Gabor filtering
    */
    void orientation();

    /**
    * applying sobel operators on the colourOpponency maps and combining via maximisation of the 3 edges
    */
    void edgesExtract();

    void getKernels();

    void setPar(int ,double);

    /**
    * function that downsamples an image by a given factor, by taking average over a factor-by-factor window
    * @param originalImage Original image
    * @param retImage Returned image
    * @param factor Factor by which it is downsampled
    */
    void downSampleImage(IplImage* originalImage, IplImage* retImage,int factor);

    /**
    * function that upsamples an image by a given factor, by taking replicating over a factor-by-factor window
    * @param originalImage Original image
    * @param retImage Returned image
    * @param factor Factor by which it is upsampled
    */
    void upSampleImage(IplImage* originalImage, IplImage* retImage,int factor);

    void downSampleMultiScales(IplImage* );

    void upSampleMultiScales(IplImage* );

    /**
    * function that given a list of images adds them according to some defined weightage
    * @param imageList List of images
    * @param numOfImages Number of images
    * @param retImage Returned image, after adding them
    * @param weights Weightage of the additions
    */
    void addImages(IplImage** imageList, int numOfImages,IplImage* retImage, float* weights);

    /**
    * function that given a list of images combines taking maximum for that location
    * @param imageList List of images
    * @param numOfImages Number of images
    * @param retImage Returned image, after combining them thus
    */
    void maxImages(IplImage** imageList, int numOfImages,IplImage* retImage);
    
    /**
    * function to convert IplImage to YARP image. The usual warpIplImage doesnt work for mono to mono.
    * @param origImage OpenCV or IplImage
    * @param yarpdImage YARP image to be sent to port
    * @param channel Currently works for only mono channel
    */
    void openCVtoYARP(IplImage* origImage,yarp::sig::ImageOf<yarp::sig::PixelMono>* yarpdImage, int channel);

    /**
    * function which crops the image
    * @param corners int array defining the crop boundaries in (left-top,right-bottom) fashion
    * @param imageToBeCropped source image that needs to be cropped
    * @param retImage The final cropped image
    */
    void cropImage(int* corners, IplImage* imageToBeCropped, IplImage* retImage);

    

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
    void convolve1D(int vecSize, float* vec, IplImage* img, IplImage* resImg, float factor=1.0,int shift =0,int direction=0,int maxVal=255);

    /**
    * function which applies a 2D kernel to a given image
    * The image need not be enlarged as this function takes care of edges efficiently.
    * @param rowSize size of the row of kernel
    * @param colSize size of the column of kernel
    * @param ker pointer to 2D kernel
    * @param img pointer to IplImage on which kernel is to be applied
    * @param factor the value by which the sum must be scaled in resulting image. default: 1.0
    * @param maxVal maximum value of the convolution operator, so as to avoid overflow. default: 255
    */
    void convolve2D(int rowSize,int colSize, float* ker, IplImage* img, IplImage* resImg, float factor=1.0,int shift=0,int* range = NULL,int maxVal=255);


    void cropCircleImage(int* center, float radius, IplImage* srcImg);
    
};




#endif  //_VISUAL_FEATURE_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

