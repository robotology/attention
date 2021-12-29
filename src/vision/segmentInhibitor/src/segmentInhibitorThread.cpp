// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/*
  * Copyright (C) 2021  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author: Omar Eldardeer
  * email: omar.eldardeer@iit.it
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


#include "iCub/segmentInhibitorThread.h"
#define THPERIOD 0.01   
segmentInhibitorThread::segmentInhibitorThread(string moduleName):PeriodicThread(THPERIOD){
    //initialize names
    this->moduleName = moduleName;

    cartCombinedImageInPortName =  getName("/cartCombinedImage:i");
    segmentedImageOutPortName =  getName("/segmentedImage:o");
    inhibitionImageOutPortName =  getName("/inhibitionImage:o");
    cartWithInhibitionImageOutPortName =  getName("/cartWithInhibitionImage:o");


    cartCombinedImage = new ImageOf<PixelRgb>;
    segmentedImage = new ImageOf<PixelMono>;
    inhibitionImage = new ImageOf<PixelMono>;
    cartWithInhibitionImage = new ImageOf<PixelRgb>;


}

segmentInhibitorThread::~segmentInhibitorThread(){

    delete cartCombinedImage ;
    delete segmentedImage ;
    delete inhibitionImage ;
    delete cartWithInhibitionImage ;

}


string segmentInhibitorThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}


bool segmentInhibitorThread::configure(yarp::os::ResourceFinder &rf){


    return true;
}

void segmentInhibitorThread::run() {

    // Reading The input Image
    if(cartCombinedImageInPort.getInputCount()){
        cartCombinedImage = cartCombinedImageInPort.read(true);
        if(cartCombinedImage!=nullptr){
            // transforming to opencv type
            cartInMat = toCvMat(*cartCombinedImage);


            // transforming to one channel image;

            cvtColor(cartInMat, cartInputGray, COLOR_BGR2GRAY);


            //thresholding

            adaptiveThreshold(cartInputGray, thrsholdedImg,  255, ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY,11,2);


            // transforming to rgb
            cvtColor(thrsholdedImg, thrsholdedRGBImg, COLOR_GRAY2RGB);
            *cartWithInhibitionImage = fromCvMat<PixelRgb>(thrsholdedImg);


        }

    }

    // thresholding the image


    // countour finding the image

    // publishing the images
    publishImagesOnPorts();


}

void segmentInhibitorThread::threadRelease() {

    cartCombinedImageInPort.interrupt();
    segmentedImageOutPort.interrupt();
    inhibitionImageOutPort.interrupt();
    cartWithInhibitionImageOutPort.interrupt();


    cartCombinedImageInPort.close();
    segmentedImageOutPort.close();
    inhibitionImageOutPort.close();
    cartWithInhibitionImageOutPort.close();
 
}


bool segmentInhibitorThread::threadInit() {

    /* ===========================================================================
	 *  Initialize all ports. If any fail to open, return false to
	 *    let RFModule know initialization was unsuccessful.
	 * =========================================================================== */

    if (!cartCombinedImageInPort.open(cartCombinedImageInPortName)) {
        yError("Unable to open %s port ",cartCombinedImageInPortName.c_str());
        return false;
    }

    if (!segmentedImageOutPort.open(segmentedImageOutPortName)) {
        yError("Unable to open %s port ",segmentedImageOutPortName.c_str());
        return false;
    }

    if (!inhibitionImageOutPort.open(inhibitionImageOutPortName)) {
        yError("Unable to open %s port ",inhibitionImageOutPortName.c_str());
        return false;
    }

    if (!cartWithInhibitionImageOutPort.open(cartWithInhibitionImageOutPortName)) {
        yError("Unable to open %s port ",cartWithInhibitionImageOutPortName.c_str());
        return false;
    }

    yInfo("Initialization of the processing thread correctly ended.");

    return true;
}

void segmentInhibitorThread::publishImagesOnPorts() {

    if(cartWithInhibitionImageOutPort.getOutputCount()){
        cartWithInhibitionImageOutPort.prepare() = *cartWithInhibitionImage;
        cartWithInhibitionImageOutPort.write();
    }

}


