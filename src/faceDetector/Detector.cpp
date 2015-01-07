// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ali Paikan
 * email:  ali.paikan@iit.it
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

#include "Detector.h"
#include <opencv/cvaux.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>

using namespace yarp::sig;
using namespace yarp::os;
using namespace std;
using namespace cv;


void Detector::loop()
{    
    ImageOf<PixelRgb> *image = imagePort.read();  // read an image
    if (image != NULL) 
    { 
        IplImage *cvImage = (IplImage*)image->getIplImage();        
        ImageOf<PixelRgb> &outImage = outPort.prepare(); //get an output image
        outImage = *image;
        display = (IplImage*) outImage.getIplImage();
        Mat imgMat = display;
        circle_t c;
        detectAndDraw(imgMat, 1.3, c);

        if(withSaliency)
        {
            ImageOf<PixelMono> &outSaliency = saliencyPort.prepare(); //get an output saliency image
            outSaliency.resize(image->width(), image->height());
            outSaliency.zero();           
            saliency = (IplImage*) outSaliency.getIplImage();
        }            
   
        // check whether a right size face-bounded circle found
        if(c.r > 0)
        {            
           if(isIn(c, face))
            {
                // we found an stable face
               // if(++counter > certainty)
                {
                    cvCircle(display, cvPoint(cvRound(face.x), cvRound(face.y)), 2, CV_RGB(0,255,0), 3, 8, 0 );
                    cvCircle(display, cvPoint(cvRound(face.x), cvRound(face.y)), cvRound(face.r), CV_RGB(0,255,0), 2, 8, 0 );
                    if(withSaliency)
                        cvCircle(saliency, cvPoint(cvRound(face.x), cvRound(face.y)), cvRound(face.r), CV_RGB(255,255,255), -1, 8, 0 );
                        
                    yarp::sig::Vector uv(2);
                    yarp::sig::Vector posRoot;
                    uv[0] = face.x;
                    uv[1] = face.y;
                    bool ret = iGaze->get3DPoint((eye=="left")?0:1, uv, eyeDist, posRoot );
                    if(ret)
                    {
                        prev_x = -eyeDist;
                        prev_y = posRoot[1];
                        prev_z = posRoot[2];
                        
                        Bottle &target=targetPort.prepare();
                        target.clear();
                        target.addDouble(prev_x);
                        target.addDouble(prev_y+offsetY);
                        target.addDouble(prev_z+offsetZ);
                        for(int i=0; i<rotation.size(); i++)
                            target.addDouble(rotation[i]);
                        targetPort.write();
                      
                        Bottle &cmd=faceExpPort.prepare();
                        cmd.addVocab(Vocab::encode("set"));
                        cmd.addVocab(Vocab::encode("all"));
                        cmd.addVocab(Vocab::encode(faceExpression.c_str()));
                        faceExpPort.write();
                    }
                }
            }
           // else
          //  {
                face = c;
          //      counter = 0;
          //      cvCircle(display, cvPoint(cvRound(c.x), cvRound(c.y)), 2, CV_RGB(255,0,0), 2, 8, 0 );
         //       cvCircle(display, cvPoint(cvRound(c.x), cvRound(c.y)), cvRound(c.r), CV_RGB(255,0,0), 1, 8, 0 );
           // }
        }
        else
            counter = 0;
        outPort.write();   
        if(withSaliency)
            saliencyPort.write();
    }
}


bool Detector::isIn(circle_t& c1, circle_t& c2)
{
    float margin = c2.r/2.0;
    if(c1.x < c2.x-margin)
        return false;
    if(c1.x > c2.x+margin)
        return false;
    if(c1.y < c2.y-margin)
        return false;
    if(c1.y > c2.y+margin)
        return false;
    return true;
}



void Detector::detectAndDraw(cv::Mat& img, double scale, circle_t &c)
{
    int i = 0;
    double t = 0;
    vector<Rect> faces;
    const static Scalar colors[] =  { CV_RGB(0,0,255),
        CV_RGB(0,128,255),
        CV_RGB(0,255,255),
        CV_RGB(0,255,0),
        CV_RGB(255,128,0),
        CV_RGB(255,255,0),
        CV_RGB(255,0,0),
        CV_RGB(255,0,255)} ;
    Mat gray, smallImg( cvRound (img.rows/scale), cvRound(img.cols/scale), CV_8UC1 );

    cvtColor( img, gray, CV_BGR2GRAY );
    resize( gray, smallImg, smallImg.size(), 0, 0, INTER_LINEAR );
    equalizeHist( smallImg, smallImg );

    //t = (double)cvGetTickCount();
    cascade.detectMultiScale( smallImg, faces,
        1.1, 2, 0
        //|CV_HAAR_FIND_BIGGEST_OBJECT
        //|CV_HAAR_DO_ROUGH_SEARCH
        |CV_HAAR_SCALE_IMAGE
        ,
        Size(30, 30) );
    c.r = 0;
    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;
        //Scalar color = colors[i%8];
        int radius;
        center.x = cvRound((r->x + r->width*0.5)*scale);
        center.y = cvRound((r->y + r->height*0.5)*scale);
        radius = cvRound((r->width + r->height)*0.25*scale);
        //circle( img, center, radius, colors[0], 3, 8, 0 );
        if(c.r < radius)
        {
           c.r = radius;
           c.x = center.x;
           c.y = center.y; 
        }
        /*
        if( nestedCascade.empty() )
            continue;
        smallImgROI = smallImg(*r);
        nestedCascade.detectMultiScale( smallImgROI, nestedObjects,
            1.1, 2, 0
            //|CV_HAAR_FIND_BIGGEST_OBJECT
            //|CV_HAAR_DO_ROUGH_SEARCH
            //|CV_HAAR_DO_CANNY_PRUNING
            |CV_HAAR_SCALE_IMAGE
            ,
            Size(30, 30) );
        for( vector<Rect>::const_iterator nr = nestedObjects.begin(); nr != nestedObjects.end(); nr++ )
        {
            center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
            center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
            radius = cvRound((nr->width + nr->height)*0.25*scale);
            circle( img, center, radius, colors[3], 3, 8, 0 );
        }
        */
    }
}


bool Detector::open(yarp::os::ResourceFinder &rf)
{
    eye = rf.check("eye", Value("left")).asString().c_str();
    faceExpression = rf.check("expression", Value("ang")).asString().c_str();
    eyeDist = fabs(rf.check("eyeDist", Value(0.3)).asDouble());
	certainty = rf.check("certainty", Value(1.0)).asInt();
    offsetZ =  rf.check("offset_z", Value(-0.05)).asDouble();
    offsetY =  rf.check("offset_y", Value(0.0)).asDouble();
    withSaliency = rf.check("enable_saliency", Value(0)).asInt();

    if(Bottle *rot=rf.find("rotation").asList())
    {
        for(int i=0; i<rot->size(); i++)
            rotation.push_back(rot->get(i).asDouble());
    }


    Property optGaze("(device gazecontrollerclient)");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/faceDetector/gazeClient");

    if (!clientGaze.open(optGaze))
        return false;

    clientGaze.view(iGaze);
    iGaze->blockNeckRoll(0.0);
    iGaze->setSaccadesStatus(false);

    
    if (rf.check("disable_saccade", Value(0)).asInt())
    {
        //disabling saccade
        printf("\nDisabling saccade...\t");
        yarp::os::Port gazePort;
        gazePort.open("...");
        if( yarp::os::Network::connect(gazePort.getName().c_str(), "/iKinGazeCtrl/rpc"))
        {
            Bottle reply;
            Bottle bt;
            bt.addString("set");
            bt.addString("sacc");
            bt.addString("off");
            gazePort.write(bt, reply);
            if(reply.size() && (reply.get(0).asString() == "ack"))
                printf("[OK]\n");
            else
                printf("[FAILED]\n");
            yarp::os::Network::disconnect(gazePort.getName().c_str(), "/iKinGazeCtrl/rpc");
        }
        else
            printf("[FAILED]\n");
        gazePort.close(); 
    }

    bool ret = cascade.load( strCascade.c_str());
    //ret = ret && nestedCascade.load( strNestedCascade.c_str());
    if(!ret)
    {
        printf("Cannot load the cascade file : %s\n", strCascade.c_str());
        return false;
    }

    ret = imagePort.open("/faceDetector/image/in");  // give the port a name
    ret = ret && outPort.open("/faceDetector/image/out");
    ret = ret && targetPort.open("/faceDetector/gazeXd");
    ret = ret && faceExpPort.open("/faceDetector/face:rpc");  
    if(withSaliency){
        ret = ret && saliencyPort.open("/faceDetector/saliency/out");     
    }
    return ret;
}

bool Detector::close()
{
    clientGaze.close();
    imagePort.close();
    outPort.close();
    targetPort.close();
    faceExpPort.close();
    if(withSaliency)
        saliencyPort.close();
    return true;
}

bool Detector::interrupt()
{
    imagePort.interrupt();
    return true;
}


