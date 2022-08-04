// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2016  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Fabio Vannucci
  * email: fabio.vannucci@iit.it
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
 * @file skinProcessorThread.cpp
 * @brief Implementation of the skin thread (see skinProcessorThread.h).
 */

#include <iCub/skinProcessorRateThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace iCub::skinDynLib;
using namespace std;
using namespace cv;

///// dictionary

const int32_t VOCAB_TORSO_UPRIGHT               = yarp::os::createVocab32('t','u','r');
const int32_t VOCAB_TORSO_UPLEFT                = yarp::os::createVocab32('t','u','l');
const int32_t VOCAB_TORSO_DOWNRIGHT             = yarp::os::createVocab32('t','d','r');
const int32_t VOCAB_TORSO_DOWNLEFT              = yarp::os::createVocab32('t','d','l');
const int32_t VOCAB_RIGHTARM_UPRIGHT            = yarp::os::createVocab32('r','u','r');
const int32_t VOCAB_RIGHTARM_UPLEFT             = yarp::os::createVocab32('r','u','l');
const int32_t VOCAB_RIGHTARM_FORERIGHT          = yarp::os::createVocab32('r','f','r');
const int32_t VOCAB_RIGHTARM_FORELEFT           = yarp::os::createVocab32('r','f','l');
const int32_t VOCAB_RIGHTARM_HAND               = yarp::os::createVocab32('r','h');
const int32_t VOCAB_LEFTARM_UPRIGHT             = yarp::os::createVocab32('l','u','r');
const int32_t VOCAB_LEFTARM_UPLEFT              = yarp::os::createVocab32('l','u','l');
const int32_t VOCAB_LEFTARM_FORERIGHT           = yarp::os::createVocab32('l','f','r');
const int32_t VOCAB_LEFTARM_FORELEFT            = yarp::os::createVocab32('l','f','l');
const int32_t VOCAB_LEFTARM_HAND                = yarp::os::createVocab32('l','h');

//////

#define THRATE 33 //ms

skinProcessorRateThread::skinProcessorRateThread():RateThread(THRATE) {
    robot = "icub";
}

skinProcessorRateThread::skinProcessorRateThread(string _robot , string _configFile):RateThread(THRATE){
    robot = _robot;
    configFile = _configFile;
}

skinProcessorRateThread::~skinProcessorRateThread() {
    // do nothing
}

bool skinProcessorRateThread::threadInit() {
    //init variables
    bodyPart = 0;
    touchedPart = 0;
    avgPressure = 0.0;
    taxellList.resize(0);
    geometricCenter.resize(0);
    touchInfo.resize(14);
    for(int i=0; i<14;i++){
        touchInfo[i] = 0;
    }
    
    image = Mat::zeros( 600, 600, CV_8UC3 );
    outputImage      = new ImageOf<PixelRgb>;
    outputImage->resize(600,600);

    // opening the port for direct input
    if (!inputPort.open(getName("/skinTouch:i").c_str())) {                                 
        yError("unable to open port to receive input");
        return false;  // unable to open; let RFModule know so that it won't run
    }

    // opening the port for direct output
    if (!outputImagePort.open(getName("/skinTouch/image:o").c_str())) {
        yError("unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }   
    
    if (!outputDataPort.open(getName("/skinTouch/data:o").c_str())) {
        yError("unable to open port to send unmasked events ");
        return false;  // unable to open; let RFModule know so that it won't run
    }     

    yInfo("Initialization of the processing thread correctly ended");
    yWarning("left, right, torso %d %d %d", useLeftArm, useRightArm, useTorso );

    return true;
}

void skinProcessorRateThread::setName(string str) {
    this->name=str;
}

void skinProcessorRateThread::setBodyParts(int _left, int _right, int _torso) {
    useLeftArm = _left;
    useRightArm = _right;
    useTorso = _torso;
}


std::string skinProcessorRateThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void skinProcessorRateThread::run() {
        if (inputPort.getInputCount()) {
            inputValue = inputPort.read(false);                                                     //non blocking read
            if (inputValue != NULL) {
                
                for(int l=0; l<14;l++){
                    touchInfo[l] = touchInfo[l]-5;
                    if(touchInfo[l] < 0) touchInfo[l] = 0;
                }


                int inputSize = inputValue->size();
                for(int i=0; i<inputSize; i++){                                                     // inputValue is a list of contacts. A contact is a list layout described at
                    contactList = inputValue->get(i).asList();                                      // http://wiki.icub.org/wiki/Tactile_sensors_(aka_Skin)
                    bodyPart = contactList->get(0).asList()->get(3).asInt16();                        // retrieve touched body part

                    int taxellCount = contactList->get(6).asList()->size();
                    taxellList.resize(taxellCount);                                                 // retrieve list of contact taxells
                    for(int j=0; j<taxellCount; j++){                                             
                        taxellList[j] = contactList->get(6).asList()->get(j).asInt16();
                    }
        
                    geometricCenter.resize(3);
                    for(int j=0; j<3; j++){                 
                        geometricCenter[j] = contactList->get(4).asList()->get(j).asFloat32();        // retrieve the coordinates of contact point
                    }

                    avgPressure = contactList->get(7).asFloat32();                                    // retrieve average pressure of contact
                    
                    Bottle& b = outputDataPort.prepare();                                            //preparing data output bottle
                    b.clear();

                    switch(bodyPart){                                                                               //retrieve touched sub-part
                        case (7):                                                                                   // torso
                            if(geometricCenter[2] >= 0.0 && geometricCenter[1] >= -0.08){ 
                                touchedPart = 1;                                                                    // torso upright        1
                                b.addVocab32(VOCAB_TORSO_UPRIGHT);
                            }
                            else if(geometricCenter[2] < 0.0 && geometricCenter[1] >= -0.08){ 
                                touchedPart = 2;                                                                    // torso upleft         2
                                b.addVocab32(VOCAB_TORSO_UPLEFT);
                            }
                            else if(geometricCenter[2] >= 0.0 && geometricCenter[1] < -0.08){
                                touchedPart = 3;                                                                    // torso downright      3
                                b.addVocab32(VOCAB_TORSO_DOWNRIGHT);
                            }
                            else if(geometricCenter[2] < 0.0 && geometricCenter[1] < -0.08){
                                touchedPart = 4;                                                                    // torso downleft       4
                                b.addVocab32(VOCAB_TORSO_DOWNLEFT);
                            }
                            break; 
                        case (6):                                                                                   // rightarm upper
                            if(geometricCenter[2] >= 0.0){
                                touchedPart = 5;                                                                    // rightarm upright     5
                                b.addVocab32(VOCAB_RIGHTARM_UPRIGHT);
                            }                                          
                            else if(geometricCenter[2] < 0.0){ 
                                touchedPart = 6;                                                                    // rightarm upleft      6
                                b.addVocab32(VOCAB_RIGHTARM_UPLEFT);
                            }
                            break;
                        case (5):                                                                                   // rightarm forearm
                            if(geometricCenter[0] <= 0.0){ 
                                touchedPart = 7;                                                                    // rightarm downright   7
                                b.addVocab32(VOCAB_RIGHTARM_FORERIGHT);
                            }
                            else if(geometricCenter[0] > 0.0){
                                touchedPart = 8;                                                                    // rightarm downleft    8
                                b.addVocab32(VOCAB_RIGHTARM_FORELEFT);
                            }
                            break;
                        case (4):
                            touchedPart = 9;                                                                        // right hand           9
                            b.addVocab32(VOCAB_RIGHTARM_HAND);
                            break;
                        case (3):                                                                                   // leftarm upper
                            if(geometricCenter[2] >= 0.0){ 
                                touchedPart = 10;                                                                   // leftarm upright     10
                                b.addVocab32(VOCAB_LEFTARM_UPRIGHT);
                            }
                            else if(geometricCenter[2] < 0.0){
                                 touchedPart = 11;                                                                  // leftarm upleft      11
                                 b.addVocab32(VOCAB_LEFTARM_UPLEFT);
                            }
                            break;
                        case (2):                                                                                   // leftarm forearm
                            if(geometricCenter[0] <= 0.0){ 
                                touchedPart = 12;                                                                   // leftarm downright   12
                                b.addVocab32(VOCAB_LEFTARM_FORERIGHT);
                            }
                            else if(geometricCenter[0] > 0.0){ 
                                touchedPart = 13;                                                                   // leftarm downleft    13
                                b.addVocab32(VOCAB_LEFTARM_FORELEFT);
                            }
                            break;
                        case (1):
                            touchedPart = 14;                                                                       // left hand           14
                            b.addVocab32(VOCAB_LEFTARM_HAND);
                            break;  
                    }
                    
                    if(taxellCount > 3){
                        touchInfo[touchedPart-1] = touchInfo[touchedPart-1] + 10 + avgPressure/10;
                        if(touchInfo[touchedPart-1] >= 255){
                            touchInfo[touchedPart-1] = 255;
                        }
                        b.addInt16(taxellCount);                                                      //add taxell number to output
                        b.addFloat32(avgPressure);                                                   //addaverage pressure to output
    
                        if(outputDataPort.getOutputCount()) {
                            if(useLeftArm && touchedPart >=10){ 
                                outputDataPort.write();
                            }else if(useRightArm && touchedPart <10 && touchedPart >=5){ 
                                outputDataPort.write();
                            }else if(useTorso && touchedPart <5){ 
                                outputDataPort.write();
                            }
                            
                        }                
                    }
                    
                    drawContact();

                    if(outputImagePort.getOutputCount()) {
                        outputImagePort.write();
                    }                                                  
                }                 
            }            
        }       
    //}  
}

int skinProcessorRateThread::processing(){
    return true;
}


void skinProcessorRateThread::drawContact() {
    
    //drawing the white rectangles container
    rectangle(  image, Point(200, 150), Point(300, 250), Scalar(255, 255, 255), 2, 8   );             // torso upright      1
    rectangle(  image, Point(300, 150), Point(400, 250), Scalar(255, 255, 255), 2, 8   );             // torso upleft       2
    rectangle(  image, Point(200, 250), Point(300, 350), Scalar(255, 255, 255), 2, 8   );             // torso downright    3
    rectangle(  image, Point(300, 250), Point(400, 350), Scalar(255, 255, 255), 2, 8   );             // torso downleft     4

    rectangle(  image, Point(100, 150), Point(125, 250), Scalar(255, 255, 255), 2, 8   );             // rightarm upright   5
    rectangle(  image, Point(125, 150), Point(150, 250), Scalar(255, 255, 255), 2, 8   );             // rightarm upleft    6
    rectangle(  image, Point(100, 300), Point(125, 400), Scalar(255, 255, 255), 2, 8   );             // rightarm downright 7
    rectangle(  image, Point(125, 300), Point(150, 400), Scalar(255, 255, 255), 2, 8   );             // rightarm downleft  8
    rectangle(  image, Point(100, 450), Point(150, 500), Scalar(255, 255, 255), 2, 8   );             // right hand         9

    rectangle(  image, Point(450, 150), Point(475, 250), Scalar(255, 255, 255), 2, 8   );             // leftarm upright    10
    rectangle(  image, Point(475, 150), Point(500, 250), Scalar(255, 255, 255), 2, 8   );             // leftarm upleft     11
    rectangle(  image, Point(450, 300), Point(475, 400), Scalar(255, 255, 255), 2, 8   );             // leftarm downright  12
    rectangle(  image, Point(475, 300), Point(500, 400), Scalar(255, 255, 255), 2, 8   );             // leftarm downleft   13
    rectangle(  image, Point(450, 450), Point(500, 500), Scalar(255, 255, 255), 2, 8   );             // left hand          14

    //drawing the intensity of touch
    if(useTorso){ 
        rectangle(  image, Point(200, 150), Point(300, 250), Scalar(0, 0, touchInfo[0]), -1, 8   );             // torso upright      1
        rectangle(  image, Point(300, 150), Point(400, 250), Scalar(0, 0, touchInfo[1]), -1, 8   );             // torso upleft       2
        rectangle(  image, Point(200, 250), Point(300, 350), Scalar(0, 0, touchInfo[2]), -1, 8   );             // torso downright    3
        rectangle(  image, Point(300, 250), Point(400, 350), Scalar(0, 0, touchInfo[3]), -1, 8   );             // torso downleft     4
    }    
    if(useRightArm){ 
        rectangle(  image, Point(100, 150), Point(125, 250), Scalar(0, 0, touchInfo[4]), -1, 8   );             // rightarm upright   5
        rectangle(  image, Point(125, 150), Point(150, 250), Scalar(0, 0, touchInfo[5]), -1, 8   );             // rightarm upleft    6
        rectangle(  image, Point(100, 300), Point(125, 400), Scalar(0, 0, touchInfo[6]), -1, 8   );             // rightarm downright 7
        rectangle(  image, Point(125, 300), Point(150, 400), Scalar(0, 0, touchInfo[7]), -1, 8   );             // rightarm downleft  8
        rectangle(  image, Point(100, 450), Point(150, 500), Scalar(0, 0, touchInfo[8]), -1, 8   );             // right hand         9
    }
    if(useLeftArm){ 
        rectangle(  image, Point(450, 150), Point(475, 250), Scalar(0, 0, touchInfo[9]), -1, 8   );             // leftarm upright    10
        rectangle(  image, Point(475, 150), Point(500, 250), Scalar(0, 0, touchInfo[10]), -1, 8   );             // leftarm upleft     11
        rectangle(  image, Point(450, 300), Point(475, 400), Scalar(0, 0, touchInfo[11]), -1, 8   );             // leftarm downright  12
        rectangle(  image, Point(475, 300), Point(500, 400), Scalar(0, 0, touchInfo[12]), -1, 8   );             // leftarm downleft   13
        rectangle(  image, Point(450, 450), Point(500, 500), Scalar(0, 0, touchInfo[13]), -1, 8   );             // left hand          14
    }
    touchedPart = 0;
    //imshow( "Skin", image );
    //waitKey( 1 );

    IplImage Ipl = (IplImage) image; 
    outputImage = &outputImagePort.prepare();        //preparing the  image for output
    outputImage->resize(600, 600);
    outputImage->wrapIplImage(&Ipl);
}

void skinProcessorRateThread::threadRelease() {
    // nothing    
}

void skinProcessorRateThread::onStop() {
    
    yInfo("closing the ports");
    inputPort.interrupt();
    outputImagePort.interrupt();
    outputDataPort.interrupt();
    inputPort.close();
    outputImagePort.close();
    outputDataPort.close();
}

