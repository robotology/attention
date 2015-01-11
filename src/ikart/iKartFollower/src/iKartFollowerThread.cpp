// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-


/*
  * Copyright (C)2014  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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
 * @file iKartFollowerThread.cpp
 * @brief Implementation of the eventDriven thread (see iKartFollowerThread.h).
 */

#include <iCub/iKartFollowerThread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define infoCycle 2

iKartFollowerThread::iKartFollowerThread():inputCbPort() {
    robot = "icub";        
}

iKartFollowerThread::iKartFollowerThread(string _robot, string _configFile):inputCbPort(){
    robot = _robot;
    configFile = _configFile;
}

iKartFollowerThread::~iKartFollowerThread() {
    // do nothing
}

bool iKartFollowerThread::threadInit() {
    //initialization of the variables
    forward  = 1;
    heading  = 0;
    linSpeed = 0;
    angSpeed = 0;
    counter  = 0;
    targetInfoPointer = 0;

    val = new double[100];

    /* open ports */ 
    //inputCbPort.hasNewImage = false;
    //inputCbPort.useCallback();          // to enable the port listening to events via callback

    if (!inputCbPort.open(getName("/data:i").c_str())) {
        cout <<": unable to open port for reading events  "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!outputPort.open(getName("/control:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    printf("\n");

    // opening file
    //string targetFilePath("./targetFile.txt");

    if (targetFile == NULL){
        yInfo("targetFile not found! Waiting target info from connection");
        //return false;
    }
    else {
        readTargetInfo(targetFile);
    }

    return true;
    

}

void iKartFollowerThread::readTargetInfo(FILE* targetFile) {
    float x;
    int numRead = 0;
    countVal = 0;
    while ((numRead != -1) && (countVal < 100)){
        for (int i = 0; i < 4; i++) {
            numRead = fscanf(targetFile, "%f", &x);
            if(numRead != -1) {
                printf("%f \n",numRead,x);
                val[countVal] = (double) x;
            }
            else {
                //printf("empty file! Please fill it with target info \n");
            }
            countVal++;
        }
        printf("\n");
    }
    rewind(targetFile);
}

void iKartFollowerThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string iKartFollowerThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void iKartFollowerThread::setInputPortName(string InpPort) {
    this->inputPortName = InpPort;
}

void iKartFollowerThread::run() {    
    while (isStopping() != true) {
        
        if(outputPort.getOutputCount()) {
            acquireTarget();
            computeControl();
            commandControl();
    
        }        
    }               
}

void iKartFollowerThread::acquireTarget() {
    //printf("acquiring target \n");

    if (counter % 4 == 0){
        printf("targetInfoPointer %d\n", targetInfoPointer);

        for( int i = 0; i < 4 ; i++) {
            printf("%d: %f \n", i, val[targetInfoPointer * 4 + i]);
        }
          
        Time::delay(5.0);

        targetInfoPointer++;
        //int countValratio4 = countVal>>2;
        if(targetInfoPointer == infoCycle){
            //printf("zeroing targetInfoPointer \n");
            targetInfoPointer = 0;
        }        
    }
    
    //positionX      = 0.1;       // m
    //positionY      = 0.1;       // m
    //velocityTarget = 0.1;       // m/s
    //omegaTarget    = 0.1;       // degrees/s
}

void iKartFollowerThread::computeControl() {
    counter++;
    //heading = counter % 360;
    /*
    if(counter % 1000 == 0){
        forward = forward * -1;
    }
    if(forward > 0){
        linSpeed = 0.1;
    }
    else{
        linSpeed = -0.1;
    }
    */
    jackKnifeControl(positionX, positionY, velocityTarget, omegaTarget, linSpeed, angSpeed);
    yInfo("linSpeed %f angSpeed %f", linSpeed, angSpeed);
}

void iKartFollowerThread::jackKnifeControl(double position1, double position2, double velocityTarget, double omegaTarget, double& velocityIKART, double& omegaIKART ){

    // convert omegaTarget from degrees to rad
    double omegaTarget_rad = omegaTarget * 3.1415 / 180;
    
    double k1 = 0.1, k2 = 0.2, k3 = 0.3;
    double epsilon = 0.001;
    double d2 = 0.05;
    double vp1 = velocityIKART * cos(omegaTarget_rad);
    double vp2 = velocityIKART * sin(omegaTarget_rad);

    double vp1power4 = vp1 * vp1 * vp1 * vp1;

    omegaIKART    = k3 * velocityIKART * velocityIKART * (vp2 * vp1 / (vp1power4 + epsilon)) - k2 * velocityIKART * velocityIKART * (position2 * vp1 / (vp1  * vp1 + epsilon));
    velocityIKART = vp1 - k1 * position1 + d2 * omegaIKART;

    //omegaIKART    = 0.1;
    //velocityIKART = 0.2;
}

void iKartFollowerThread::commandControl() {
    Bottle& b = outputPort.prepare();
    b.clear();
    b.addInt(2);
    b.addDouble(heading);  //the commanded linear direction of the iKart, expressed in degrees.
    b.addDouble(linSpeed); //the commanded linear speed, expressed in m/s.
    b.addDouble(angSpeed); //the commanded angular speed, expressed in deg/s.
    outputPort.writeStrict();
}

void iKartFollowerThread::threadRelease() {
    //delete[] val;
    if(targetFile!=NULL){
        fclose(targetFile);
    }
}

void iKartFollowerThread::onStop() {
    inputCallbackPort.interrupt();
    outputPort.interrupt();

    inputCallbackPort.close();
    outputPort.close();
}

