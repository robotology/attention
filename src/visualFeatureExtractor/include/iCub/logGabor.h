// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Shashank Pathak
 * email:   shashank.pathak@iit.it
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
 * @file logGabor.h
 * @brief Implementation of log-Gabor filters. This is useful in implementing phase congruency methods (eg. by Peter
 *  Kovesi).Peter Kovesi,  "Image Features From Phase Congruency". Videre: A Journal of Computer Vision Research. MIT
 * Press. Volume 1, Number 3, Summer 1999.
 */

#ifndef _LOG_GABOR_H_
#define _LOG_GABOR_H_

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <iostream>
#include <yarp/os/Stamp.h>


#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

// GNU scientific libarary includes
#include <gsl/gsl_math.h>
#include <gsl/gsl_fft_complex.h>
#include <gsl/gsl_sort_double.h>
#include <gsl/gsl_statistics.h>

#define LOG_GABOR_SCALE 4
#define LOG_GABOR_ORIENTATION 6
#define ROW_SIZE 320
#define COL_SIZE 240
#define PI_Gab 3.1415926535897932

typedef double logGaborComplexArray[COL_SIZE][2*ROW_SIZE];                  // Complex image corresponding to image size
typedef double logGaborOrientArray[LOG_GABOR_SCALE][COL_SIZE][ROW_SIZE];  // Array of filtered images for an 
                                                                            // orientation, across any scale(first index)
typedef double logGaborComplexRow[2*ROW_SIZE];                              // To store image in complex notation

class logGabor{

    

    logGaborOrientArray* logGaborFilter;                            // to store LG filter for each scale and orientation
    logGaborComplexRow* imgInComplex;                               // to store image in complex notation
    
    // For non-radix 2 FFTs
    gsl_fft_complex_wavetable * wt_row;
    gsl_fft_complex_workspace * wk_row;
    gsl_fft_complex_wavetable * wt_col;
    gsl_fft_complex_workspace * wk_col;

    bool logGaborReady;

    

public:

    /**
    * Constructor
    */
    logGabor();

    /**
    * Destructor
    */
    ~logGabor();

    /**
    * function that implements 2D FFT/iFFT as two iterative 1D FFTs (not implemented in gsl)
    * @param input2DArray 2D array of whose FFT or iFFT is to be taken
    * @param FFTed output array
    * @param forward flag which is true/false for FFT/iFFT respectively, default is FFT
    */
    void FFT2D(double input2DArray[COL_SIZE][2*ROW_SIZE], double FFTed[COL_SIZE][2*ROW_SIZE], bool forward = true);

    /**
    * function that sets up log-Gabor filters
    * @param m factor of scaling between the filters
    * @param sigmaF std deviation of the Gaussian of the filters
    * @param minWav minimum wavelength
    * @param cuttoff_butterworth cutoff frequency for Butterworth filter
    * @param sharpness_butterworth order by which the filter decays higher frequencies  
    */
    void setLogGabor(int m=2, double sigmaF=.65, double minWave=3, double cuttoff_butterworth=.45, int sharpness_butterworth=15);

    /**
    * function that applies a particular (given orientation and given scale) log Gabor on image
    * @param inputImage input image
    * @param outputImage monochromatic output image
    * @param scale integer for scale index
    * @param orient integer for orientation index  
    */
    void getOneLogGabor(yarp::sig::ImageOf<yarp::sig::PixelMono>* inputImage,yarp::sig::ImageOf<yarp::sig::PixelFloat>* outputRealImage,yarp::sig::ImageOf<yarp::sig::PixelFloat>* outputImgImage, int scale =0, int orient=0);

    /**
    * function that applies a particular (given orientation across all scales) log-Gabor on image
    * @param inputImage input image
    * @param outputImage monochromatic output image
    * @param weightage weightage given to each scale while summing them, default is equal-normalised weightage
    * @param orient integer for orientation index  
    */
    void getLogGaborForOrient(yarp::sig::ImageOf<yarp::sig::PixelMono>* inputImage,yarp::sig::ImageOf<yarp::sig::PixelFloat>* outputRealImage,yarp::sig::ImageOf<yarp::sig::PixelFloat>* outputImgImage, double weightage[LOG_GABOR_SCALE]=NULL,int orient=0);

    /**
    * function that applies log-Gabor on image across all scales and all orientations
    * @param inputImage input image
    * @param outputImage monochromatic output image
    * @param weightScale weightage given to each scale while summing them, default is equal-normalised weightage
    * @param weightOrient weightage given to each orientation while summing them, default is equal-normalised weightage  
    */
    void getAllLogGabor(yarp::sig::ImageOf<yarp::sig::PixelMono>* inputImage,yarp::sig::ImageOf<yarp::sig::PixelFloat>* outputRealImage,yarp::sig::ImageOf<yarp::sig::PixelFloat>* outputImgImage, double weightScale[LOG_GABOR_SCALE]=NULL,double weightOrient[LOG_GABOR_ORIENTATION]=NULL);


};



#endif  //_LOG_GABOR_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

