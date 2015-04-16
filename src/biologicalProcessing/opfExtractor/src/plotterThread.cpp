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
 * @file plotterThread.cpp
 * @brief Implementation of the thread that represent image (see header plotterThread.h)
 */

#include <iCub/plotterThread.h>
#include <cstring>
#include <cassert>
#include <cstdlib>
#include <cv.h>
#include <highgui.h>
#include <cxcore.h>


using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 66   //15 fps

plotterThread::plotterThread() : RateThread(THRATE) {
    synchronised = false;
    count  = 0;
    width  = 320;   //default dimension width 
    height = 240;   //default dimension height
}

plotterThread::~plotterThread() {

}

bool plotterThread::threadInit() {
    printf("\n starting the thread.... \n");
    // opening ports 
    outputPort.open      (getName("/colorResult:o").c_str());
    outputPortu.open      (getName("/uResult:o").c_str());
    outputPortv.open      (getName("/vResult:o").c_str());
    outputPortm.open      (getName("/mResult:o").c_str());

    
    // initialising images
    outputImage      = new ImageOf<PixelRgb>;
    outputImage->resize(width,height);

    uImage      = new ImageOf<PixelMono>;
    uImage->resize(width,height);

    vImage      = new ImageOf<PixelMono>;
    vImage->resize(width,height);

    mImage      = new ImageOf<PixelMono>;
    mImage->resize(width,height);

    printf("initialization in plotter thread correctly ended \n");
    return true;
}

void plotterThread::interrupt() {
    outputPort.interrupt();   
    outputPortu.interrupt();
    outputPortv.interrupt();
    outputPortm.interrupt();
}



void plotterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string plotterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void plotterThread::resize(int widthp, int heightp) {
}

void plotterThread::copyImage(ImageOf<PixelMono>* image) {
	sem.wait();
	//yDebug("copy the image of the module");
	outputImage->copy(*image);
	sem.post();
}

void plotterThread::copyImage(ImageOf<PixelRgb>* image) {
	sem.wait();
	//yDebug("copy the image of the module");
	outputImage->copy(*image);
	sem.post();
}


void plotterThread::copyU(cv::Mat U) {
    sem.wait();
    ImageOf<PixelMono>* imageu = new ImageOf<PixelMono>();    //here we are just putting the pointer image at the beginning of the memory , but we have to allocate memory with new
    imageu->resize(width, height);
    convertMat2ImageOf(U, imageu);      //oppure fare le 3 sopra,in  modo che sia riutilizzabile anche nel caso uImage e U siano di tipo diverso
    uImage->copy(*imageu);
    sem.post();
    delete imageu;
}

void plotterThread::copyV(cv::Mat V) {        
    sem.wait();
    ImageOf<PixelMono>* imagev = new ImageOf<PixelMono>();    //here we are just putting the pointer image at the beginning of the memory , but we have to allocate memory with new
    imagev->resize(width, height);
    convertMat2ImageOf(V, imagev);      //oppure fare le 3 sopra,in  modo che sia riutilizzabile anche nel caso uImage e U siano di tipo diverso
    vImage->copy(*imagev);
    sem.post();
    delete imagev;
}


void plotterThread::copyM(cv::Mat Mask) {        
    sem.wait();
    ImageOf<PixelMono>* imagem = new ImageOf<PixelMono>();    //here we are just putting the pointer image at the beginning of the memory , but we have to allocate memory with new
    imagem->resize(width, height);
    convertMat2ImageOf(Mask, imagem);      //oppure fare le 3 sopra,in  modo che sia riutilizzabile anche nel caso uImage e U siano di tipo diverso
    mImage->copy(*imagem);
    sem.post();
    delete imagem;
}


void plotterThread::convertMat2ImageOf(cv::Mat a, ImageOf<PixelMono>* image) {
    IplImage tempIpla = (IplImage) a;
    //printf("%08x \n", image);
    image-> wrapIplImage(&tempIpla);
}


/*
void plotterThread::copyLeft(ImageOf<PixelRgb>* image) {
    int padding= imageLeft->getPadding();
    //printf("retinalSize in plotterThread %d %d %d \n",padding, width, height);
    unsigned char* pimage = image->getRawImage();
    unsigned char* pleft  = imageLeft->getRawImage();
    if(imageLeft != 0) {
        for(int r = 0;r < height; r++) {
            for(int c = 0; c < width; c++) {                
                if(r%2 == 0) {
                    *pleft++ = 0;
                    *pleft++ = 0;
                    *pleft++ = 0;
                }
                else{
                    *pleft++ = 255;
                    *pleft++ = 255;
                    *pleft++ = 255;
                }
            }
            pleft  += padding;
            pimage += padding;
        }
    }
}
*/

bool plotterThread::test(){
    bool ret = true;
    ret &= Network::exists(getName("/image:i").c_str());
    return ret;
}

void plotterThread::run() {
    //count++;
    if(outputPort.getOutputCount()) {
        //yDebug("plotting the image");
        ImageOf<PixelRgb>& imagePrepare  = outputPort.prepare();
        imagePrepare.resize(width, height);
        imagePrepare.zero();

        sem.wait();
        imagePrepare.copy(*outputImage);
        //copyImage(outputImage, imagePrepare);
        sem.post();
    
        synchronised = true;
    
        outputPort.write();
    }

    if (outputPortu.getOutputCount()) {
        //yDebug("plotting the image");
        ImageOf<PixelMono>& imagePrepareu  = outputPortu.prepare();
        imagePrepareu.resize(width, height);

        sem.wait();
        imagePrepareu.copy(*uImage);
        //copyImage(outputImage, imagePrepare);
        sem.post();
    
        synchronised = true;
    
        outputPortu.write();
    }

    if (outputPortv.getOutputCount()) {
        //yDebug("plotting the image");
        ImageOf<PixelMono>& imagePreparev  = outputPortv.prepare();
        imagePreparev.resize(width, height);

        sem.wait();
        imagePreparev.copy(*vImage);
        //copyImage(outputImage, imagePrepare);
        sem.post();
    
        synchronised = true;
    
        outputPortv.write();
    }

    if (outputPortm.getOutputCount()) {
        //yDebug("plotting the image");
        ImageOf<PixelMono>& imagePreparem  = outputPortm.prepare();
        imagePreparem.resize(width, height);

        sem.wait();
        imagePreparem.copy(*mImage);
        //copyImage(outputImage, imagePrepare);
        sem.post();
    
        synchronised = true;
    
        outputPortm.write();
    }
}


void plotterThread::threadRelease() {
    printf("plotterThread: portClosing \n");  
    outputPort.close();
    outputPortu.close();
    outputPortv.close();
    outputPortm.close();

    printf("freeing memory \n");

    // free allocated memory here please
    delete outputImage;

    printf("success in release the plotter thread \n");
}


//----- end-of-file --- ( next line intentionally left blank ) ------------------
