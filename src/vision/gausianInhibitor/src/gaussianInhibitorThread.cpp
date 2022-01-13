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


#include "iCub/gaussianInhibitorThread.h"
#define THPERIOD 0.01   
gaussianInhibitorThread::gaussianInhibitorThread(string moduleName): PeriodicThread(THPERIOD){
    //initialize names
    this->moduleName = moduleName;

    cartCombinedImageInPortName =  getName("/cartCombinedImage:i");
    hotPointInPortName =  getName("/hotPoint:i");
    inhibitionImageOutPortName =  getName("/inhibitionImage:o");
    cartWithInhibitionImageOutPortName =  getName("/cartWithInhibitionImage:o");


    cartCombinedImage = new ImageOf<PixelRgb>;
    inhibitionImage = new ImageOf<PixelMono>;
    cartWithInhibitionImage = new ImageOf<PixelRgb>;





    inhibitedPoints.push_back(Point(100,100));


}

gaussianInhibitorThread::~gaussianInhibitorThread(){

    delete cartCombinedImage ;
    delete inhibitionImage ;
    delete cartWithInhibitionImage ;

}


string gaussianInhibitorThread::getName(const char* p) const{
    string str(moduleName);
    str.append(p);
    return str;
}


bool gaussianInhibitorThread::configure(yarp::os::ResourceFinder &rf){
    //imageSize
    rowSize = rf.findGroup("imageSize").check("rowSize", yarp::os::Value(300)).asInt32();
    colSize = rf.findGroup("imageSize").check("colSize", yarp::os::Value(200)).asInt32();

    //inhibitionZone
    xMargin = rf.findGroup("inhibitionZone").check("xMargin", yarp::os::Value(30)).asInt32();
    yMargin = rf.findGroup("inhibitionZone").check("yMargin", yarp::os::Value(30)).asInt32();

    //distributionParams
    sigma = rf.findGroup("distributionParams").check("sigma", yarp::os::Value(90)).asFloat64();


    yInfo( " " );
    yInfo( "\t                  [imageSize]                 ");
    yInfo( "\t ============================================ ");
    yInfo("\t rowSize                  : %d pixels", rowSize );
    yInfo("\t colSize                  : %d pixels", colSize );
    yInfo( "\t           [inhibitionZone]            "       );
    yInfo( "\t ============================================ ");
    yInfo("\t xMargin                  : %d pixels", xMargin );
    yInfo("\t yMargin                  : %d pixels", yMargin );
    yInfo( "\t           [distributionParams]            "   );
    yInfo( "\t ============================================ ");
    yInfo("\t sigma                    : %.3f    ", sigma   );
    yInfo( " " );
    yInfo( " " );



    cartInMat = Mat(rowSize,colSize,CV_8UC3,Scalar(0,0,0));
    inhMat = Mat(rowSize,colSize,CV_8UC1,Scalar(255));
    cartWithInhMat = Mat(rowSize,colSize,CV_8UC3,Scalar(0,0,0));

    for (int i = 0;i<30;i++){
        for(int j=0; j<30;j++){
            inhMat.at<uchar>(i,j) = (unsigned char )  0;
        }
    }

    return true;
}

void gaussianInhibitorThread::run() {

    if( hotPointInPort.getInputCount()) {
        hotPointBottle = hotPointInPort.read(false);
        if(readHotPointBottle()){
            addGaussianCircleFromPoint(hotPoint);
        }
    }


    // Reading The input Image
    if(cartCombinedImageInPort.getInputCount()){
        cartCombinedImage = cartCombinedImageInPort.read(true);
        if(readCartImage()){
            yInfo("good Image");
        }
    }

    computeImages();
    publishImagesOnPorts();


}

void gaussianInhibitorThread::threadRelease() {

    cartCombinedImageInPort.interrupt();
    hotPointInPort.interrupt();
    inhibitionImageOutPort.interrupt();
    cartWithInhibitionImageOutPort.interrupt();


    cartCombinedImageInPort.close();
    hotPointInPort.close();
    inhibitionImageOutPort.close();
    cartWithInhibitionImageOutPort.close();
 
}


bool gaussianInhibitorThread::threadInit() {

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

void gaussianInhibitorThread::publishImagesOnPorts() {

    if(inhibitionImageOutPort.getOutputCount()){
        inhibitionImageOutPort.prepare() = *inhibitionImage;
        inhibitionImageOutPort.write();
    }
    if(cartWithInhibitionImageOutPort.getOutputCount()){
        cartWithInhibitionImageOutPort.prepare() = *cartWithInhibitionImage;
        cartWithInhibitionImageOutPort.write();
    }

}
bool gaussianInhibitorThread::readHotPointBottle() {
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


void gaussianInhibitorThread::removeAllPoints() {
    inhibitedPoints.clear();
    inhMat = Mat(rowSize,colSize,CV_8U,Scalar(255));
}


void gaussianInhibitorThread::computeImages() {

    Mat cartInputGray;
    Mat cartWithInhGrayMat;
    cvtColor(cartInMat, cartInputGray, COLOR_BGR2GRAY);
    cartWithInhGrayMat = min(cartInputGray,inhMat);
    cvtColor(cartWithInhGrayMat, cartWithInhMat, COLOR_GRAY2RGB);


    *inhibitionImage = fromCvMat<PixelMono>(inhMat);
    *cartWithInhibitionImage = fromCvMat<PixelRgb>(cartWithInhMat);

}

double gaussianInhibitorThread::norm_pdf(double x, double sig)
{
    if(x  == 0){
        return 1.0 / (sig * sqrt(2.0 * M_PI)) * exp(-(pow((x) / sig, 2) / 2.0));
    }
    return (1.0 / (sig * sqrt(2.0 * M_PI)) * exp(-(pow((x) / sig, 2) / 2.0))) / norm_pdf(0, sig);

}

void gaussianInhibitorThread::addGaussianCircleFromPoint(Point &point) {

    inhibitedPoints.push_back(point);
    int x = max(0,point.x - xMargin);
    int y;
    double dist;
    for(x;(x<rowSize && x < point.x + xMargin );x++){
        for(y = max(0,point.y - yMargin);(y<colSize && y< point.y + yMargin);y++){
            dist = pow(x-point.x, 2) + pow(y-point.y, 2);       //calculating Euclidean distance
            dist = sqrt(dist);
            double pixVal = norm_pdf(dist,sigma) * 255;
            pixVal = 255 - pixVal;
            yInfo(" x: %d , y %d  val  = %0.4f",x,y,pixVal);
            inhMat.at<uchar>(x,y) = min(inhMat.at<uchar>(x,y),(unsigned char )pixVal)  ;

        }
        yInfo(" x:_________________________ %d",x);
    }
    inhMat.at<uchar>(point.x,point.y) = 0;

}

bool gaussianInhibitorThread::readCartImage() {
    if(cartCombinedImage!=nullptr){
        cartInMat = toCvMat(*cartCombinedImage);
        return true;
    }
    yError("Error in Reading the input image");
    return false;
}

void gaussianInhibitorThread::updateInhMat() {


    inhMat = Mat(rowSize,colSize,CV_8U,Scalar(255));;

    for(auto point : inhibitedPoints){
        int x = max(0,point.x - xMargin);
        int y;
        double dist;
        for(x;(x<rowSize && x < point.x + xMargin );x++){
            for(y = max(0,point.y - yMargin);(y<colSize && y< point.y + yMargin);y++){
                dist = pow(x-point.x, 2) + pow(y-point.y, 2);       //calculating Euclidean distance
                dist = sqrt(dist);
                double pixVal = norm_pdf(dist,sigma) * 255;
                pixVal = 255 - pixVal;
                yInfo(" x: %d , y %d  val  = %0.4f",x,y,pixVal);
                inhMat.at<uchar>(x,y) = min(inhMat.at<uchar>(x,y),(unsigned char )pixVal)  ;

            }
            yInfo(" x:_________________________ %d",x);
        }
        inhMat.at<uchar>(point.x,point.y) = 0;
    }

}

bool gaussianInhibitorThread::setValue(const Value& var, const Value& val) {
    if(var.asString() == "margin"){
        xMargin = val.asInt32();
        yMargin = val.asInt32();
        updateInhMat();
        return true;
    }else if (var.asString() == "xMargin"){
        xMargin = val.asInt32();
        updateInhMat();
        return true;

    }else if(var.asString() == "yMargin"){
        yMargin = val.asInt32();
        updateInhMat();
        return true;

    }else if(var.asString() == "sigma"){
        sigma = val.asFloat64();
        updateInhMat();
        return true;
    }
    yError("error in variable name");
    return false;
}

double gaussianInhibitorThread::getValue(const Value& var) const {
    if (var.asString() == "xMargin"){
        return xMargin;

    }else if(var.asString() == "yMargin"){
        return yMargin;

    }else if(var.asString() == "sigma"){
        return sigma;
    }
    yError("error in variable name");
    return -1;
}


