// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
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
 * @file colourSaliencyThread.h
 * @brief Definition of a thread that receive images, processes them and creates a saliency map highlighting the region where the colour pattern is more likely to be
 * (see colourSaliencyModule.h).
 */

#ifndef _COLOUR_SALIENCY_THREAD_H_
#define _COLOUR_SALIENCY_THREAD_H_

#include <yarp/os/BufferedPort.h>
#include <yarp/os/RateThread.h>
#include <yarp/sig/all.h>
#include <iostream>


class colourSaliencyThread : public yarp::os::RateThread {
private:
    static const int dimVVector= 255;           // dimension of the vector due to the V
    static const int dimUVector= 255;           // dimension of the vector due to the U
    int trainingX, trainingY;                   // dimension of the training image
    int* trainingVector;                        // bucket collection of tokens in the colour combination
    int count;                                  // loop counter of the thread
    int width, height;                          // dimension of the extended input image (extending)
    int height_orig, width_orig;                // original dimension of the input and output images
    float lambda;                              // coefficient for temporal filtering
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > inPort;        // port where the input image is received
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > outPort;      // port where the output is sent
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > trainingPort;  // port where the images for learning are sent
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* inImage;                               // image reference to the input image
    yarp::sig::ImageOf<yarp::sig::PixelRgb>* trainingImage;                         // image reference to the training image
    std::string name;                                   // rootname of all the ports opened by this thread
    bool resized;                                       // flag to check if the variables have been already resized
    bool trained;                                       // flag that changes when the trained has occured
    unsigned char targetRed, targetGreen, targetBlue;   // target in rgb colours
public:
    /**
    * default constructor
    */
    colourSaliencyThread();

    /**
     * destructor
     */
    ~colourSaliencyThread();

    /**
    * function that initialise the thread
    */
    bool threadInit();

    /**
    * function called when the thread is stopped
    */
    void threadRelease();

    /**
    * function called every time constant defined by rateThread
    */
    void run(); 

    /**
    * function called when the module is poked with an interrupt command
    */
    void interrupt();

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
    * function that sets the width and the height of the images based on the dimension of the input image
    * @param width width of the input image
    * @return height height of the input image
    */
    void resize(int width, int height);

    /**
    * function that calculates the correlation function between the targetColour and any pixel of the input image
    * @param imageIn colour image representing the input 
    * @param imageOut gray scale image reprensent the level of correlation between the input and the training images
    */
    void makeCorrImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> imageIn, yarp::sig::ImageOf<yarp::sig::PixelMono> imageOut);

    /**
    * function that creates an grayscale image made of the distances between target colour and inputimage colour
    * @param imageIn colour image representing the input 
    * @param imageOut gray scale image reprensent the level of distance between the input and the training images
    */
    void makeDistanceImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> imageIn, yarp::sig::ImageOf<yarp::sig::PixelMono>& imageOut);

    /**
    * function that creates a probability map in which any subpart corresponds of a degree of matching with the histogram of colour trained
    * @param imageIn colour image representing the input 
    * @param imageOut gray scale image reprensent the probability of matching between image and histogram of colour
    */
    void makeProbImage(yarp::sig::ImageOf<yarp::sig::PixelRgb> imageIn, yarp::sig::ImageOf<yarp::sig::PixelMono>* imageOut);

    /**
    * function which  updates the vector for successive computation of correlation
    * @param pixel pointer to the image
    * @param num vector of RGB accumulators (numerator)
    * @param den_P vector of RGB accumulators (denominator)
    * @param den_T vector of RGB accumulators (denominator)
    */
    void pixelCorr(unsigned char* pixel,yarp::sig::Vector& num, yarp::sig::Vector& den_P, yarp::sig::Vector& den_T);
    
    /**
    * function that updates the target colour reading the training image
    * @param trainingImage image that updates the target colour for search
    */
    void training(yarp::sig::ImageOf<yarp::sig::PixelRgb>* trainingImage);

    /**
    * applies the bucket leak algorithm to the vector of the training
    */
    void bucketLeak();

};

#endif  //_COLOUR_SALIENCY_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

