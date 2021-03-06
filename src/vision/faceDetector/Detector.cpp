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
using namespace yarp::dev;
using namespace std;
using namespace cv;

void Detector::suspend() {
    idle = true;
}

void Detector::resume() {
    idle = false;
}

void Detector::loop()
{   

  if (!idle){

    ImageOf<PixelRgb> *image = imagePort.read();  // read an image
    if (image != NULL) 
    { 
        IplImage *cvImage = (IplImage*)image->getIplImage();        
        ImageOf<PixelRgb> &outImage = outPort.prepare(); //get an output image
        outImage = *image;
        display = (IplImage*) outImage.getIplImage();
        Mat imgMat = cvarrToMat(display);
        circle_t c;
        detectAndDraw(imgMat, 1.3, c);

        if(withSaliency)
        {
            ImageOf<PixelMono> &outSaliency = saliencyPort.prepare(); //get an output saliency image
            outSaliency.resize(image->width(), image->height());
            outSaliency.zero();           
            saliency = (IplImage*) outSaliency.getIplImage();
        }

        Bottle &faceFoundBottle = foundFacePort.prepare();
        faceFoundBottle.clear();

        // check whether a right size face-bounded circle found
        if(c.r > 0)
        {            
           if(isIn(c, face))
           {
               faceFoundBottle.addString("true");
               // we found an stable face
                // if(++counter > certainty)
                {
                    if(disableGazeControl){
                        cvCircle(display, cvPoint(cvRound(face.x), cvRound(face.y)), 2, CV_RGB(0,255,0), 3, 8, 0 );
                        cvCircle(display, cvPoint(cvRound(face.x), cvRound(face.y)), cvRound(face.r), CV_RGB(0,255,0), 2, 8, 0 );
                    
                    }

                    else{
                        cvCircle(display, cvPoint(cvRound(face.x), cvRound(face.y)), 2, CV_RGB(0,0,255), 3, 8, 0 );
                        cvCircle(display, cvPoint(cvRound(face.x), cvRound(face.y)), cvRound(face.r), CV_RGB(0,0,255), 2, 8, 0 );
                    
                    }

                    if(withSaliency){
                        cvCircle(saliency, cvPoint(cvRound(face.x), cvRound(face.y)), cvRound(face.r), CV_RGB(255,255,255), -1, 8, 0 );
                    }
                    double alfa = alpha;    
                    yarp::sig::Vector uv(2);
                    yarp::sig::Vector posRoot(3);


                    int w2 = (display->width)>>1;
                    int h2 = (display->height)>>1;
                    double distance = sqrt((face.x - w2) * (face.x - w2) + (face.y - h2) * (face.y - h2));                    
                    

                    if(distance > 40.0 && disableGazeControl){
                        yInfo("w2 %d h2 %d distance %f", w2, h2, distance);
                        uv[0] = face.x;
                        uv[1] = face.y;
                                        
                        //yDebug("position %f %f distance %f", face.x, face.y, eyeDist);
                        bool ret = iGaze->get3DPoint((eye=="left")?0:1, uv, eyeDist, posRoot );
                        //yDebug("got 3D position %s", posRoot.toString().c_str());
                        if(ret)
                            {
                        

                                yarp::sig::Vector posRootFinal(3);
                                posRootFinal[0] = alfa * posRoot[0] + (1 - alfa) * prev_x;
                                posRootFinal[1] = alfa * posRoot[1] + (1 - alfa) * (prev_y + offsetY);
                                posRootFinal[2] = alfa * posRoot[2] + (1 - alfa) * (prev_z + offsetZ);

                                prev_x = -eyeDist;
                                prev_y = posRootFinal[1];
                                prev_z = posRootFinal[2];
                        
                                //-------------------------------------
                                iGaze->lookAtFixationPoint(posRootFinal);
                                //--------------------------------------

                                //yDebug("get3DPoint %f %f %f sent in...ready to send",prev_x, prev_y+offsetY, prev_z+offsetZ);
                                Bottle &target=targetPort.prepare();
                                target.clear();
                                target.addDouble(prev_x);
                                target.addDouble(prev_y+offsetY);
                                target.addDouble(prev_z+offsetZ);
                                for(int i=0; i<rotation.size(); i++){
                                    target.addDouble(rotation[i]);
                                }
                                targetPort.write();
                      
                                Bottle &cmd=faceExpPort.prepare();
                                cmd.addVocab(Vocab::encode("set"));
                                cmd.addVocab(Vocab::encode("all"));
                                cmd.addVocab(Vocab::encode(faceExpression.c_str()));
                                faceExpPort.write();


                            }
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
        else{
            faceFoundBottle.addString("false");
            counter = 0;
        }
        if(withSaliency){
            saliencyPort.write();
        }

        outPort.write();

        if((Time::now() - currentTime) > refreshFaceFound){
            foundFacePort.write();
            currentTime = Time::now();
        }
    }
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
    Point rectTopLeft, rectBottomRight;
    for( vector<Rect>::const_iterator r = faces.begin(); r != faces.end(); r++, i++ )
    {
        Mat smallImgROI;
        vector<Rect> nestedObjects;
        Point center;

        rectTopLeft.x = r->x * scale;
        rectTopLeft.y = r->y * scale;

        rectBottomRight.x = ( r->x + r->width ) * scale;
        rectBottomRight.y = ( r->y + r->height ) * scale;
    
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

        //yInfo("c.x %f c.y %f",c.x, c.y);                                          
        
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

    if(rectTopLeft.x != 0 && rectTopLeft.y != 0 && rectBottomRight.x != 0 && rectBottomRight.y != 0){
        Bottle &faceRectCoordinateBottle = faceRectCoordinatePort.prepare();
        faceRectCoordinateBottle.clear();

        faceRectCoordinateBottle.addVocab( yarp::os::createVocab('s','e','t'));
        faceRectCoordinateBottle.addVocab( yarp::os::createVocab('t','r','a','c'));
        faceRectCoordinateBottle.addInt(rectTopLeft.x);
        faceRectCoordinateBottle.addInt(rectTopLeft.y);

        faceRectCoordinateBottle.addInt(rectBottomRight.x);
        faceRectCoordinateBottle.addInt(rectBottomRight.y);

        faceRectCoordinatePort.write();
    }
    
}


bool Detector::open(yarp::os::ResourceFinder &rf)
{
    yDebug("Opening the Detector");
    eye = rf.check("eye", Value("left")).asString().c_str();
    faceExpression = rf.check("expression", Value("ang")).asString().c_str();
    eyeDist = fabs(rf.check("eyeDist", Value(0.7)).asDouble());
    alpha = fabs(rf.check("alpha", Value(0.15)).asDouble());
    certainty = rf.check("certainty", Value(1.0)).asInt();
    offsetZ =  rf.check("offset_z", Value(-0.05)).asDouble();
    offsetY =  rf.check("offset_y", Value(0.0)).asDouble();
    withSaliency = rf.check("enable_saliency", Value(0)).asInt();
    withAttentionSystem = rf.check("with_attention", Value(0)).asBool();
    disableGazeControl  = rf.check("gaze_control", Value( !withAttentionSystem)).asBool();

    refreshFaceFound = rf.check("refresh_face_rate", Value( "2.")).asDouble();


    yInfo("eye: %s", eye.c_str());
    yInfo("faceExpression: %s", faceExpression.c_str());
    yInfo("eyeDistance: %f", eyeDist);
    yInfo("alpha: %f", alpha);
    yInfo("certainty: %d", certainty);
    yInfo("offsetZ: %f", offsetZ);
    yInfo("offsetY: %f", offsetY);
    yInfo("withSaliency: %d", withSaliency);
    

    if(Bottle *rot=rf.find("rotation").asList())
    {
        for(int i=0; i<rot->size(); i++)
            rotation.push_back(rot->get(i).asDouble());
    }

    //---------------------------------------------------------------
    yDebug("Opening the connection to the iKinGaze");
    Property optGaze; //("(device gazecontrollerclient)");
    optGaze.put("device","gazecontrollerclient");
    optGaze.put("remote","/iKinGazeCtrl");
    optGaze.put("local","/faceDetector/gaze");

    
    clientGaze = new PolyDriver();
    clientGaze->open(optGaze);
    iGaze=NULL;
    
    //yDebug("polydriver created");
    //if (!clientGaze->open(optGaze)) {
    //    yError("Error in connecting the iKinGaze");
    //    return false;
    //}
        
    yDebug("Connecting to the iKinGaze");
    if (clientGaze->isValid()) {
       clientGaze->view(iGaze);
    }
    else
        return false;

    iGaze->storeContext(&startup_context_id);
    
    if(!withAttentionSystem){
        // @Rea 29.11.17 : commented this out for SC demo 5.12 to avoid conflicts with PROVISION
        ////clientGaze.view(iGaze);
        iGaze->blockNeckRoll(0.0);
        ////iGaze->setSaccadesStatus(false);
        iGaze->setSaccadesMode(false);
        //// set trajectory time:
        iGaze->setNeckTrajTime(0.5);
        iGaze->setEyesTrajTime(0.2);
        iGaze->setTrackingMode(true);
        iGaze->setVORGain(1.3);
        iGaze->setOCRGain(1.0);

    }

    
   
                     

    //---------------------------------------------------------------

    
    if (rf.check("disable_saccade", Value(0)).asInt())
    {
        //disabling saccade
        printf("\nDisabling saccade...\t");
        yarp::os::Port gazePort;
        gazePort.open("/faceDetector/gazeInterface:o");
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

    yDebug("Looking for cascade in %s", strCascade.c_str());
    bool ret = cascade.load( strCascade.c_str());
    //ret = ret && nestedCascade.load( strNestedCascade.c_str());
    if(!ret)
    {
        printf("Cannot load the cascade file : %s\n", strCascade.c_str());
        return false;
    }

    ret = imagePort.open("/faceDetector/image/in");  // give the port a name
    ret = ret && outPort.open("/faceDetector/image/out");
    ret = ret && faceRectCoordinatePort.open("/faceDetector/face/rectCoord");
    ret = ret && targetPort.open("/faceDetector/gazeXd");
    ret = ret && faceExpPort.open("/faceDetector/face:rpc");  
    ret = ret && foundFacePort.open("/faceDetector/faceFound:o");
    if(withSaliency){
        ret = ret && saliencyPort.open("/faceDetector/saliency/out");     
    }

    prev_y = 0.0;
    prev_z = 0.35;

    currentTime = Time::now();
    yDebug("Initialization completed");
    return ret;
    
}

bool Detector::close()
{
    iGaze->stopControl();
    iGaze->restoreContext(startup_context_id);
   
    clientGaze->close();
    imagePort.close();
    outPort.close();
    targetPort.close();
    faceExpPort.close();
    if(withSaliency) {
        saliencyPort.close();
    }
    return true;
}

bool Detector::interrupt()
{
    imagePort.interrupt();
    return true;
}

void Detector::setGazeControl(bool t_disableGazeControl){
    disableGazeControl = t_disableGazeControl;
}



