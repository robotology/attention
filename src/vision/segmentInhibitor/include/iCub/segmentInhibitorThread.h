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

#ifndef ATTENTION_SEGMENTINHIBITORTHREAD_H
#define ATTENTION_SEGMENTINHIBITORTHREAD_H


#include <string>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Network.h>
#include <yarp/os/all.h>
#include <yarp/cv/Cv.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::cv;
using namespace std;
using namespace cv;

typedef yarp::sig::ImageOf<yarp::sig::PixelRgb> yImgPixelRgb;
typedef yarp::sig::ImageOf<yarp::sig::PixelMono> yImgPixelMono;

typedef yarp::os::BufferedPort<yImgPixelRgb> yImgPixelRgbBuffer;
typedef yarp::os::BufferedPort<yImgPixelMono> yImgPixelMonoBuffer;

    
class segmentInhibitorThread : public PeriodicThread {

public:
    segmentInhibitorThread(string moduleName = "segmentInhibitor");
    ~segmentInhibitorThread();
    bool configure(yarp::os::ResourceFinder &rf);
    bool threadInit();
    void run();
    void threadRelease();
    string getName(const char* p) const;


private:




    /* ===========================================================================
	 *  Names
	 * =========================================================================== */
    string moduleName;


    string cartCombinedImageInPortName;
    string segmentedImageOutPortName;
    string inhibitionImageOutPortName;
    string cartWithInhibitionImageOutPortName;

    
    /* ===========================================================================
	 *  Ports
	 * =========================================================================== */

    yImgPixelRgbBuffer cartCombinedImageInPort;
    yImgPixelMonoBuffer segmentedImageOutPort;
    yImgPixelMonoBuffer inhibitionImageOutPort;
    yImgPixelRgbBuffer cartWithInhibitionImageOutPort;


    /* ===========================================================================
    *  Data
    * =========================================================================== */

    yImgPixelRgb *cartCombinedImage;
    yImgPixelMono *segmentedImage;
    yImgPixelMono *inhibitionImage;
    yImgPixelRgb *cartWithInhibitionImage;

    Mat cartInMat;
    Mat cartInputGray;
    Mat thrsholdedImg;
    Mat thrsholdedRGBImg;

    

    void publishImagesOnPorts();

};



#endif //SEGMENTINHIBITOR_SEGMENTINHIBITORTHREAD_H 

