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
    hotPointInPortName =  getName("/hotPoint:i");
    segmentedImageOutPortName =  getName("/segmentedImage:o");
    inhibitionImageOutPortName =  getName("/inhibitionImage:o");
    cartWithInhibitionImageOutPortName =  getName("/cartWithInhibitionImage:o");


    cartCombinedImage = new ImageOf<PixelRgb>;
    segmentedImage = new ImageOf<PixelRgb>;
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

    if( hotPointInPort.getInputCount()) {
        hotPointBottle = hotPointInPort.read(false);
        if(readHotPointBottle()){
            addContourFromPoint(hotPoint);
        }
    }


    // Reading The input Image
    if(cartCombinedImageInPort.getInputCount()){
        cartCombinedImage = cartCombinedImageInPort.read(true);
        if(cartCombinedImage!=nullptr){
            // transforming to opencv type
            cartInMat = toCvMat(*cartCombinedImage);

            cartWithInhMat = cartInMat.clone();
            drawContours(cartWithInhMat, inhibitedContours, 1, Scalar(0, 0, 0), FILLED);

            *cartWithInhibitionImage = fromCvMat<PixelRgb>(cartWithInhMat);

        }

    }
    if(segmentedImageOutPort.getOutputCount()){
        computeSegmentedImage();
    }

    if(inhibitionImageOutPort.getOutputCount()){
        computeInhImage();
    }

    publishImagesOnPorts();


}

void segmentInhibitorThread::threadRelease() {

    cartCombinedImageInPort.interrupt();
    hotPointInPort.interrupt();
    segmentedImageOutPort.interrupt();
    inhibitionImageOutPort.interrupt();
    cartWithInhibitionImageOutPort.interrupt();


    cartCombinedImageInPort.close();
    hotPointInPort.close();
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

    if (!hotPointInPort.open(hotPointInPortName)) {
        yError("Unable to open %s port ",hotPointInPortName.c_str());
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

    if(segmentedImageOutPort.getOutputCount()){
        segmentedImageOutPort.prepare() = *segmentedImage;
        segmentedImageOutPort.write();
    }
    if(inhibitionImageOutPort.getOutputCount()){
        inhibitionImageOutPort.prepare() = *inhibitionImage;
        inhibitionImageOutPort.write();
    }
    if(cartWithInhibitionImageOutPort.getOutputCount()){
        cartWithInhibitionImageOutPort.prepare() = *cartWithInhibitionImage;
        cartWithInhibitionImageOutPort.write();
    }

}
bool segmentInhibitorThread::readHotPointBottle() {
    if(hotPointBottle!=nullptr){
        if(hotPointBottle->size() >= 4){
            Bottle* hotPointCoordinatesList = hotPointBottle->get(0).asList();
            if(hotPointCoordinatesList!=nullptr && hotPointCoordinatesList->size()==2){
                hotPoint.x = hotPointCoordinatesList->get(0).asInt32();
                hotPoint.y  =  hotPointCoordinatesList->get(1).asInt32();
            }
            return true;
        }
    }
    return false;
}
void segmentInhibitorThread::addContourFromPoint(Point &point) {

    // transforming to one channel image;
    Mat cartInputGray;
    Mat cartGrayBlured;
    Mat thrsholdedImg;
    cvtColor(cartInMat, cartInputGray, COLOR_BGR2GRAY);


    GaussianBlur(cartInputGray,cartGrayBlured, Size( 11, 11 ), 0, 0 );

    //thresholding
    //adaptiveThreshold(cartGrayBlured, thrsholdedImg,  255, ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,11,2);

    threshold(cartGrayBlured, thrsholdedImg,  70,255,THRESH_BINARY_INV);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thrsholdedImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);


    vector<vector<Point>> contoursOfPoint;
    for(int i = 0;i<contours.size();i++){
        if( pointPolygonTest(contours[i],point, false) >= 0 ){
            contoursOfPoint.push_back(contours[i]);

        }
    }

}

void segmentInhibitorThread::removeAllContours() {
    inhibitedContours.clear();
}

void segmentInhibitorThread::computeSegmentedImage() {
    Mat cartInputGray;
    Mat cartGrayBlured;
    Mat thrsholdedImg;
    Mat thrsholdedRgbImg;

    // transforming to one channel image;

    cvtColor(cartInMat, cartInputGray, COLOR_BGR2GRAY);

    // bluring
    GaussianBlur(cartInputGray,cartGrayBlured, Size( 11, 11 ), 0, 0 );

    //thresholding
    //adaptiveThreshold(cartGrayBlured, thrsholdedImg,  255, ADAPTIVE_THRESH_GAUSSIAN_C,THRESH_BINARY,11,2);

    threshold(cartGrayBlured, thrsholdedImg,  70,255,THRESH_BINARY_INV);

    // drawing contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thrsholdedImg, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);


    // transforming to rgb
    cvtColor(thrsholdedImg, thrsholdedRgbImg, COLOR_GRAY2RGB);

    sagMat = thrsholdedRgbImg.clone();
    drawContours(sagMat, contours, -1, Scalar(0, 0, 255), 2);


    *segmentedImage = fromCvMat<PixelRgb>(sagMat);

}

void segmentInhibitorThread::computeInhImage() {
    inhMat = Mat(cartInMat.rows,cartInMat.cols,CV_8U,Scalar(255));
    drawContours(inhMat, inhibitedContours, -1, Scalar(0, 0, 255), 2);
    *inhibitionImage = fromCvMat<PixelMono>(inhMat);

}


