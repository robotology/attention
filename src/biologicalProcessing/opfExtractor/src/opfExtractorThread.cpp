// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*                                                                                                                                  
  * Copyright (C)2013  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file opfExtractorThread.cpp
 * @brief Implementation of the eventDriven thread (see opfExtractorThread.h).
 */

#include <iCub/opfExtractorThread.h>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
using namespace cv;

opfExtractorThread::opfExtractorThread() {
    robot = "icub";        
}

opfExtractorThread::opfExtractorThread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

opfExtractorThread::~opfExtractorThread() {
    // do nothing
}

bool opfExtractorThread::threadInit() {
    // initialization of the attributes
    width  = WIDTH;
    height = HEIGHT;
    idle   = false;
    throwAway = false;
    firstProcessing=true;
    numberProcessing=0;
    setAlgorithm(ALGO_FB);

    countDescr=0;

    inputImage = new ImageOf<PixelRgb>();
    inputImage->resize(width, height);

    processingImage = new ImageOf<PixelRgb>();
    processingImage->resize(width, height);

    // opening the port for direct input
    if (!inputPort.open(getName("/image:i").c_str())) {
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }  

    // opening the port for direct output
    if (!outputPort.open(getName("/result:o").c_str())) {
        yError("unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }   

    //starting the plotterThread
    pt = new plotterThread();
    pt->setName(getName("").c_str());
    pt->start();

    //starting the featExtractorThread
    fet = new featExtractorThread();
    fet->setName(getName("").c_str());
    fet->start();

    yInfo("Initialization of the processing thread correctly ended");

    return true;
}

void opfExtractorThread::setName(string str) {
    this->name=str;
}


std::string opfExtractorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void opfExtractorThread::setInputPortName(string InpPort) {
    
}

void opfExtractorThread::visualizationResume(){
    pt->resume();
}

void opfExtractorThread::visualizationSuspend(){
    pt->suspend();
}

bool opfExtractorThread::test(){
    bool reply = true;

    if(pt==NULL){
        reply = false;
    }
    else {
        reply &= pt->test();
    }

    reply  &= Network::exists(getName("/image:i").c_str());

    return reply;
}

/*In this block, the input port is read and the processing is done. 
During the processing, it is computed:
-the optical flow
-the mask of the meaningful optical flow (in the thresholding function)
-the optical flow is masked by the mask just found (in the thresholding function)
Moreover, it instantiates an object of the plotterThread and an object of the featExtractorThread*/
void opfExtractorThread::run() {
    while (isStopping() != true) {
        bool result;

        if(!idle){
            if(inputPort.getInputCount()) {
                inputImage = inputPort.read(true);   //blocking reading for synchr with the input
                if (throwAway){
                    throwAway = false;
                }
                else {
                    result = processing();               //generates the outputImage which is what we want to plot

                    pt->copyImage(processingImage);
                    pt->copyU(U);                       //I have instantiated an  object p of type plotterThread, and now I can call the function of this class (copyU)
                    
                    double minval, maxval;
                    cv::Point  minLoc, maxLoc;
                    cv::minMaxLoc(gradientMaskNorm,&minval, &maxval, &minLoc, &maxLoc);
                    cv::minMaxLoc(gradientMaskNorm,&minval, &maxval, &minLoc, &maxLoc);

                    pt->copyV(gradientMaskNorm);
                    pt->copyM(Maskt);

                    fet->copyAll(U,V,Maskt);
                    fet->setFlag();

                    throwAway = true;
                }
            }
            else {
                result = 0;
            }
        }
    }
}


bool opfExtractorThread::processing(){
    numberProcessing++;
    cv::Mat Matrix((IplImage*) inputImage->getIplImage(), false);
    if(!firstProcessing) {
        previousMatrix=currentMatrix.clone();
    }
    cv::cvtColor(Matrix, currentMatrix, CV_RGB2GRAY);
    
    //--------------------------------------------------------------------------

    if(!firstProcessing){

        cv::Mat flow;
        cv::Mat U_1 = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
        cv::Mat V_1 = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);

        cv::Mat maskThresholding = cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_32FC1);
        cv::Mat MV[2]={U,V};
        std::vector<float> descr;   //length=4
        int computed=0;

        if(numberProcessing>=3){
            U_1=U.clone();
            V_1=V.clone();
            BBinfo_1=BBinfo.clone();
        }

        switch (ofAlgo) {
            case ALGO_FB:{
                    cv::calcOpticalFlowFarneback(previousMatrix, currentMatrix, flow, 0.2, 3, 19, 10, 7, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
                    cv::split(flow, MV);
            }break;

            case ALGO_LK:{
                    yDebug("just entered in  Lk");
                    std::vector<cv::Point2f> prevPts;
                    std::vector<cv::Point2f> currPts;
                    cv::Size winSize(5,5); 

                    int delta_=1;
                    vector<uchar> status;
                    vector<float> err;
                    yDebug("before the cycle for");
                    for(int x = 0; x < previousMatrix.cols; x=x+delta_)    {
                        for(int y = 0; y < previousMatrix.rows; y=y+delta_)    {
                            prevPts.push_back(cv::Point2f((float)x,(float)y));                //in prevPts there are the xs and the ys of the image
                        }
                    }
                    cv::calcOpticalFlowPyrLK(previousMatrix, currentMatrix, prevPts, currPts, status, err, winSize, 3);
                
                    for(int k = 0; k < prevPts.size(); k++)    {  //I is a vector and each  element is a point (therefore with two fields)

                        if (status.at(k))    {
                            U.at<float>(prevPts.at(k).y, prevPts.at(k).x)    =  currPts.at(k).x - prevPts.at(k).x;
                            V.at<float>(prevPts.at(k).y, prevPts.at(k).x)    =  currPts.at(k).y - prevPts.at(k).y;
                        }
                    }
                    prevPts.clear();
                      currPts.clear();
                    status.clear();
                    err.clear();
            }break;
        }  // end of switch of optical flow algorithm

        cv::Mat colorcodeMatrix =  cv::Mat::zeros(previousMatrix.rows,previousMatrix.cols, CV_8UC3);
        if(numberProcessing>=3) {        //you can compute the acceleration when you have 3 frames
            thresholding(U, V, maskThresholding);
            ////NEW CODE HERE????  or inside thresholding ?
            motionToColor(U, V, colorcodeMatrix);   //computing colorcode
        }
        IplImage Ipl = (IplImage) colorcodeMatrix; 
        processingImage-> wrapIplImage(&Ipl);
        if (outputPort.getOutputCount()) {
            V.convertTo(outputMatrix,CV_8UC1);              //it is plotted just V
            IplImage tempIpl = (IplImage) outputMatrix;
            outputImage-> wrapIplImage(&tempIpl);
            //outputImage-> wrapIplImage(&((IplImage)(outputMatrix)));
        }

        flow.release();
        maskThresholding.release();
        colorcodeMatrix.release();

        U_1.release();
        V_1.release();
    } // if(processing!= firstProcessing)
    else {
        U = cv::Mat::zeros(height, width, CV_32FC1);  //I have already use U and V to put them in U_1 and V_1  //?or is it better to initialize U, V, U_1, V_1 in the threadInit?
        V = cv::Mat::zeros(height, width, CV_32FC1);

        Maskt = cv::Mat::zeros(height, width,  CV_32FC1);
        gradientMask = cv::Mat::zeros(height, width,  CV_32FC1);
        gradientMaskNorm = cv::Mat::zeros(height, width,  CV_32FC1);
        BBinfo = cv::Mat::zeros(4, 2, CV_32FC1);
        BBinfo_resized2blobs = cv::Mat::zeros(2, 2, CV_32FC1);
        BBinfo_sorted = cv::Mat::zeros(4, 2, CV_32FC1);

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                U.at<float>(y, x) = 0.000001;
                V.at<float>(y, x) = 0.000001;
                //Maskt.at<float>(y, x) = 0.000001;
              }
         }
        firstProcessing=false;
    }
    //--------------------------------------------------------------------------
    Matrix.release();

    return true;
}


/*this function takes the optical flow Ut and Vt computed and returns:
maskThresholding: a black image with white blobs where the optical flow is meaninguful
Ut and Vt: just inside the white mask, set to zero the optical flow vectors outside the mask
For building the mask, we take into consideration the point (x,y) if:
- it has a flow magnitude more than a threshold (TH1_),
- around the point (x,y), there is a number of points, with flow magnitude higher than a threshold (TH2_), higher than a certain threshold (related to PTH)*/
void opfExtractorThread::thresholding(cv::Mat& Ut, cv::Mat& Vt, cv::Mat& maskThresholding){
    
    cv::Mat MAGt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::Mat THETAt = cv::Mat::zeros(Ut.rows, Ut.cols, CV_32FC1);
    cv::cartToPolar(Ut, Vt, MAGt, THETAt, false);       // from u-v to Magnitude-Theta
  
    cv::Mat Probt = cv::Mat::zeros(MAGt.rows, MAGt.cols, CV_32FC1);
    int DELTA = 10;
    int LATO = DELTA*2+1;
    for(int i = DELTA; i < Probt.rows-DELTA; ++i) {
        for(int j = DELTA; j < Probt.cols-DELTA; ++j) {
            //std::cout <<  MAGt.at<float>(i,j) << " " << TH1_ << " " << std::endl;
            if(MAGt.at<float>(i,j)> TH1_ ) {
                cv::Mat Q = MAGt(cv::Range(i-DELTA,i+DELTA+1), cv::Range(j-DELTA,j+DELTA+1));
                cv::Mat MQ = Q >= TH2_;       //>= returns a map of 0 and 255 instead of 1
                MQ = MQ/255.;                 //to have a map of 0 and 1
                Probt.at<float>(i,j) = cv::sum(MQ).val[0]/((float)(LATO*LATO));          // divide by lato*lato in such a way to have 1 as maximum
                if(Probt.at<float>(i,j) >= PTH_) {
                    Maskt.at<float>(i,j) = 1.0;
                    float a = MAGt.at<float>(i,j); 
                    gradientMask.at<float>(i,j) = MAGt.at<float>(i,j); 
                }
                else {
                    Maskt.at<float>(i,j) =  0.0;
                    gradientMask.at<float>(i,j) = 0.0;
                }
                Q.release();
                MQ.release();
            }
            else {
                Maskt.at<float>(i,j) =  0;
                gradientMask.at<float>(i,j) = 0.0;
            }
        }
    }// so now we have a matrix Maskt of 0 and 1, with 1 in the points where we want to take into account the flow

        double minval, maxval;
        cv::Point  minLoc, maxLoc;
        cv::minMaxLoc(gradientMask,&minval, &maxval, &minLoc, &maxLoc);
        cv::minMaxLoc(gradientMask,&minval, &maxval, &minLoc, &maxLoc);
        
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                float fg = gradientMask.at<float>(y, x);//motim.Pixel(x, y, 0);

                float fgnorm;

                if (maxval == minval)
                    fgnorm=0.00000001;
                else
                    fgnorm = (fg-minval)/(maxval-minval);

                gradientMaskNorm.at<float>(y, x) = fgnorm*255;
            }
        }

        double minval_x, maxval_x, minval_y, maxval_y;
        cv::Point  minLoc_x, maxLoc_x, minLoc_y, maxLoc_y;
        cv::minMaxLoc(gradientMaskNorm,&minval_x, &maxval_x, &minLoc_x, &maxLoc_x);
        cv::minMaxLoc(gradientMaskNorm,&minval_x, &maxval_x, &minLoc_x, &maxLoc_x);

    // to see an image for debugging (without using yarp ports)
    //cv::imshow("Maskt", Maskt);
    //cv::waitKey(10); //it waits 10  and then goes on. If you want to make it wait for your signal, write cv::waitKey(10000) and push Enter to go on

    //identify the connected components and visualize them
    //yInfo("identifying the connect components");
    std::vector<std::vector<cv::Point> > contours;
    cv::Mat helpframe;
    Maskt.convertTo(helpframe, CV_8U);
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(helpframe, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);    //it returns the matrix contours

    //yInfo("iterating the pixels");
    for(std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ) {                         //contours.begin() is the pointer to the initial element
        if(cv::contourArea(*it)/(Maskt.rows*Maskt.cols)>0.005)  //I took the contour it into consideration if its area is higher then the 0.5% of the total area of the image
            ++it; //we are at the level of each bounding box,then the threshold is less
        else
            it = contours.erase(it);
    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////
    //NEW CODE

    BBinfo = cv::Mat::zeros(contours.size(), 2, CV_32FC1);
    cv::Moments BBmom;
    cv::Mat BBMask, BBMaskTMP;
      
    for(int c = 0; c < contours.size(); ++c) {      //contours.size() is the number of contours found
        BBMask = cv::Mat::zeros(height, width, CV_8UC1);
        cv::drawContours(BBMask, contours, c, CV_RGB(255,255,255), -1);  //BBMask is the mask of the c-th contours (it is just the mask  of one blob)

        BBmom = cv::moments(BBMask, true);
        BBinfo.at<float>(c, 0) = BBmom.m10/BBmom.m00;        //BBinfo will be a matrix of c rows and 2 columns, with the xy of each centroid per row
        BBinfo.at<float>(c, 1) = BBmom.m01/BBmom.m00;
    }

    ////NEW -- check it               aaa
    //n_BBinfo = BBinfo.rows;
    //while(n_BBinfo > 2){             //until we have more than 2 new centroids

    //    cout<<"BBinfo_1 \n" <<BBinfo_1<<endl;
    //    cout<<"BBinfo \n" <<BBinfo<<endl;

    //    float maxDistTemp  = 0;
    //    int i_res = 0;
    //    for (i_BB = 0; i_BB <  BBinfo.rows; ++i_BB){    //find the new blob with maximum distance wrt old blobs
    //        for (i_BB_1 = 0; i_BB_1 <  BBinfo_1.rows; ++i_BB_1){
    //            distCurr = distance(BBinfo.at<float>(i_BB, 0), BBinfo.at<float>(i_BB, 1), BBinfo_1.at<float>(i_BB_1, 0), BBinfo_1.at<float>(i_BB_1, 1));
    //            if(distCurr > maxDistTemp  ){
    //                maxDistTemp = distCurr;
    //                i_maxDistTemp = i_BB;
    //            }
    //        }
    //    }
    //    n_BBinfo--;
    //    //i_maxDist.push_back(i_maxDistTemp);    //to have a vector with the indeces of the vector BBinfo that I don t have to copy tu the BBinfo_resized2blobs
    //    i_maxDistt = i_maxDistTemp;
    ////}

    //    //if(i_maxDist.size()!=0){
    //    i_it  = 0;
    //    for(std::vector<std::vector<cv::Point> >::iterator it = contours.begin(); it != contours.end(); ) {                         //contours.begin() is the pointer to the initial element
    //        if (i_it == i_maxDistt ){
    //            it = contours.erase(it);
    //        }
    //        i_it++;   //check it, si alza di troppo invece io volevo qualcosa che andasse da 0 a 2
    //    }

    //    for(int i=0; i<BBinfo.rows;i++){
    //        if (i!=i_maxDistt){
    //            BBinfo_resized2blobs.at<float>(i_res, 0)=BBinfo.at<float>(i, 0);
    //            BBinfo_resized2blobs.at<float>(i_res, 1)=BBinfo.at<float>(i, 1);
    //            i_res++;
    //        }
    //    }
    //    BBinfo = BBinfo_resized2blobs.clone();

    //    cout<<"BBinfo_1 \n" <<BBinfo_1<<endl;
    //    cout<<"BBinfo \n" <<BBinfo<<endl;
    //    cout<<"BBinfo_resized2blobs \n" <<BBinfo_resized2blobs<<endl;


    //    //}
    //}





    //NO
    ////if BBinfo has more than 3 centroids, I will take just the two blobs with greater area
    //while (BBinfo.rows>2){
    //    BBinfo_resized2blobs;
    //    float minArea = width*height;
    //    int c_minArea = 0;
    //    for(int c = 0; c < contours.size(); ++c){
    //        cv::drawContours(BBMask, contours, c, CV_RGB(255,255,255), -1);
    //        float tempd =  cv::sum(BBMask/255.0).val[0];
    //        if (tempd<minArea){
    //            minArea = tempd;
    //            c_minArea = c;
    //        }
    //    }

    //    int i_res=0;
    //    for (int c = 0; c < contours.size(); c++){
    //            if (c != c_minArea) {
    //                BBinfo_resized2blobs.at<float>(i_res, 0)=BBinfo.at<float>(c, 0);
    //                BBinfo_resized2blobs.at<float>(i_res, 1)=BBinfo.at<float>(c, 1);
    //                i_res++;
    //            }
    //    }

    //    if(BBinfo_1.rows >2){
    //     cout<<"oo"<<endl;
    //    }
    //    cout<<"BBinfo_1 \n" <<BBinfo_1<<endl;
    //    cout<<"BBinfo \n" <<BBinfo<<endl;
    //    cout<<"BBinfo_resized2blobs \n" <<BBinfo_resized2blobs<<endl;
    //    BBinfo =  BBinfo_resized2blobs.clone();
    //}



    //sort BBinfo in such a way to be consistent with BBinfo_1
    //if (BBinfo.rows ==1 ){  //if I have just ne blob, I would assign it to the most closest one of t-1
    //    float dist_min = distance(BBinfo.at<float>(0, 0), BBinfo.at<float>(0, 1), BBinfo_1.at<float>(0, 0), BBinfo_1.at<float>(0, 1));
    //    for (int c_1 = 0;  c_1  < BBinfo_1.rows; ++c_1){
    //        float dist_new = distance(BBinfo.at<float>(0, 0), BBinfo.at<float>(0, 1), BBinfo_1.at<float>(c_1, 0), BBinfo_1.at<float>(c_1, 1));
    //        if (dist_new < dist_min) {
    //            dist_min = dist_new;
    //            c_1min=c_1;
    //        }                                    
    //    }
    //    BBinfo.at<float>(0, 0) =  BBinfo_1.at<float>(c_1min, 0);
    //    BBinfo.at<float>(0, 1) =  BBinfo_1.at<float>(c_1min, 1);
    //}

    //if (BBinfo_1.rows>=1){
    //    //BBinfo_sorted = cv::Mat::zeros(4, 2, CV_32FC1);
    //    cout<<"BBinfo_1 \n" <<BBinfo_1<<endl;
    //    cout<<"BBinfo \n" <<BBinfo<<endl;
    //    cout<<"BBinfo_sorted \n" <<BBinfo_sorted<<endl;

    //    if (BBinfo_1.rows > BBinfo.rows) {
    //        for(int c = 0; c < BBinfo.rows; ++c) {
    //            float dist_min = distance(BBinfo.at<float>(c, 0),  BBinfo.at<float>(c, 1),BBinfo_1.at<float>(0, 0),BBinfo_1.at<float>(0, 1));
    //            float c_1min = 0;
    //            for (int c_1 = 0;  c_1  < BBinfo_1.rows; ++c_1){
    //                float dist_new = distance(BBinfo.at<float>(c, 0), BBinfo.at<float>(c, 1), BBinfo_1.at<float>(c_1, 0), BBinfo_1.at<float>(c_1, 1));
    //                if (dist_new < dist_min) {
    //                    dist_min = dist_new;
    //                    c_1min=c_1;
    //                }
    //            }
    //            BBinfo_sorted.at<float>(c_1min, 0) = BBinfo.at<float>(c, 0);
    //            BBinfo_sorted.at<float>(c_1min, 1) = BBinfo.at<float>(c, 1);
    //        }
    //    }

    //    if (BBinfo_1.rows < BBinfo.rows) {
    //        for (int c_1 = 0;  c_1  < BBinfo_1.rows; ++c_1){
    //            float dist_min = distance(BBinfo_1.at<float>(c_1, 0),  BBinfo_1.at<float>(c_1, 1),BBinfo.at<float>(0, 0),BBinfo.at<float>(0, 1));
    //            float cmin = 0;
    //            for(int c = 0; c < BBinfo.rows; ++c) {
    //                float dist_new = distance(BBinfo_1.at<float>(c_1, 0), BBinfo_1.at<float>(c_1, 1), BBinfo.at<float>(c, 0), BBinfo.at<float>(c, 1));
    //                if (dist_new < dist_min) {
    //                    dist_min = dist_new;
    //                    cmin=c;
    //                }
    //            }
    //            BBinfo_sorted.at<float>(c_1, 0) = BBinfo.at<float>(cmin, 0);
    //            BBinfo_sorted.at<float>(c_1, 1) = BBinfo.at<float>(cmin, 1);
    //        }
    //    }
    //cout<<"BBinfo_sorted \n" <<BBinfo_sorted<<endl;
    //}
    //BBinfo = BBinfo_sorted.clone();


    ///////////////////////////////////////////////////////////////////////////////////////////////////



    //yInfo("drawing contours");
     cv::drawContours(maskThresholding, contours, -1, CV_RGB(255,255,255), -1);  //it returns maskThresholding, which is the output fo this function thresholding: an image with white where there are the contours found before; -1 for whitening all the bounding boxes; other -1 for whitening them also inside

    //yInfo("removing the UV outside the mask");
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            if (maskThresholding.at<float>(y, x)==0.0) {
                Ut.at<float>(y, x) = 0.000001;
                Vt.at<float>(y, x) = 0.000001;
                Maskt.at<float>(y, x) = 0.0;
            }
            else {
            }
        }
     }
}
//}

float opfExtractorThread::distance(float X0, float Y0, float X1, float Y1){
    return sqrt((X1 - X0)*(X1 - X0) + (Y1 - Y0)*(Y1 - Y0));
}

void opfExtractorThread::motionToColor(cv::Mat U, cv::Mat V, cv::Mat& colorcodeMatrix){
    double minval_x, maxval_x, minval_y, maxval_y;
    cv::Point  minLoc_x, maxLoc_x, minLoc_y, maxLoc_y;
    cv::minMaxLoc(U,&minval_x, &maxval_x, &minLoc_x, &maxLoc_x);
    //std::cout  << "maxval U     "<< maxval_x << std::endl;
    cv::minMaxLoc(V,&minval_y, &maxval_y, &minLoc_y, &maxLoc_y);
    //std::cout  << "maxval V     "<< maxval_y << std::endl;

    uchar *ccMatrixPointer = colorcodeMatrix.data;                  //data is pointing to the Red of the 1st pixel

    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            float fx = U.at<float>(y, x);//motim.Pixel(x, y, 0);
            float fy = V.at<float>(y, x);//motim.Pixel(x, y, 1);
            //.Pixel(x, y, 0);
            float fxnorm, fynorm;

            if (maxval_x == minval_x)
                fxnorm=0.00000001;
            else
                fxnorm = (fx-minval_x)/(maxval_x-minval_x);
            if (maxval_y == minval_y)
                fynorm=0.00000001;
            else
                fynorm = (fy-minval_y)/(maxval_y-minval_y); 

            assert(fxnorm >=0); assert(fxnorm <= 1.00);
            assert(fynorm >= 0); assert(fynorm <= 1.00);
            
            //computeColor(fxnorm, fynorm, ccMatrixPointer);

            computeColor(fx, fy, ccMatrixPointer);   //fx and fy are not normalized, therefore they are not between 0 and 1
            ccMatrixPointer += 3;
        }
    }
}

void opfExtractorThread::threadRelease() {
    // nothing    
}

void opfExtractorThread::onStop() {
    delete inputImage;
    delete outputImage;
    delete processingImage;
    U.release();
    V.release();
    if(pt!=NULL){
        //yDebug("stopping the plotter thread");
        //printf("stopping the plotter thread \n");
        pt->stop();
    }

    if(fet!=NULL){
        //yDebug("stopping the plotter thread");
        //printf("stopping the plotter thread \n");
        fet->stop();
    }
    yInfo("closing the ports");
    //printf("closing the ports \n");

    inputPort.interrupt();
    inputPort.close();

    outputPort.interrupt();   
    outputPort.close();
}