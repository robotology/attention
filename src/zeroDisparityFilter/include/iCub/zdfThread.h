/*
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff
 * email:   vadim.tikhanoff@iit.it
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
*
* @ingroup icub_logpolarAttention
* \defgroup icub_zeroDisparityFilter zeroDisparityFilter
*
* This Thread performs zero disparity filtering. Given the object at zero disparity segments the silhouette of the object
*/

#ifndef ZERODISPARITYFILTER_ZDFTHREAD_H
#define ZERODISPARITYFILTER_ZDFTHREAD_H


#include <yarp/os/Thread.h>
#include <yarp/sig/Image.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include "iCub/multiclass.h"
#include <iCub/centerSurround.h>


class ZDFThread : public yarp::os::Thread {
public:

    struct MultiClass::Parameters *params;


    /**
    * constructor
    */
    ZDFThread(MultiClass::Parameters *parameters);

    /**
     * destructor
     */
    ~ZDFThread() override;

    bool threadInit() override;

    void setName(std::string module);

    void run() override;

    void onStop() override;



private:

    typedef unsigned  char element;


    //string containing module name
    std::string moduleName;

    /*port name strings*/
    std::string inputNameLeft;           //string containing input port name left
    std::string inputNameRight;          //string containing input port name right
    std::string outputNameProb;          //string containing the probability output port name
    std::string outputNameSeg;           //string containing the segmentation output port name
    std::string outputNameDogLeft;           //string containing the difference of gaussian output port name
    std::string outputNameDogRight;           //string containing the difference of gaussian output port name
    std::string outputNameTemp;          //string containing the segmented template output port name
    std::string outputNameTemp2;          //string containing the segmented template output port name
    std::string outputNameGeometry;

    // Images for processing
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageInLeft;      //input port cartesian image left
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageInRight;     //input port cartesian image right
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutProb;     //output port probability image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutSeg;      //output port segmented image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutDogL;      //output port difference of gaussian image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutDogR;      //output port difference of gaussian image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutSaliency;     //output port saliency image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > imageOutTemplateRGB;     //output port template image

    yarp::os::BufferedPort<yarp::os::Bottle> outputGeometry;    // output 2D position and size of the object
    yarp::os::BufferedPort<yarp::os::Bottle> outputCOG;    // output 2D position and size of the object

    //Difference of Gaussian
    cv::Mat DOG_left, DOG_right;

    // Pyramid of gaussian
    IplImage *left_pyramid_DOG, *right_pyramid_DOG;
    CenterSurround *centerSurround;

    //Fovea variables
    cv::Rect2d fovea_rect, foveaRecWithOffset;
    cv::Mat left_fovea, right_fovea, YUV_left, YUV_right;



    // segmentation variables
    cv::Mat prob_mat, zd_prob_mat, seg_image, seg_DOG;
    int koffsetx, koffsety;
    Coord c;
    int *buffer1, *buffer2;
    MultiClass *multiClass;
    unsigned char** p_prob;
    defSize fovea_size;
    cv::Mat templateRGB;


    cv::Mat getDOG(const cv::Mat& in);


    //void get_rank(Coord c,Ipp8u *im, int w, int*list);
    void get_rank(Coord c, unsigned char *im, int w, int *list);

    double cmp_rank(int *l1, int *l2);

    void get_ndt(Coord coord, unsigned char *im, int w, int *list); //void get_ndt(Coord c,Ipp8u *im, int w, int*list);

    double cmp_ndt(int *l1, int *l2);

    void processDisparityMap(const unsigned char* img_one, const unsigned char* img_two);

    void parametersInit();

    void medianfilter(element* signal, element* result, int N);

    void _medianfilter(const element* signal, element* result, int N);

    void getRoundingBoxSegmented(int *top, int *bottom, int *left, int *right, cv::Mat *segmentedImage);

    void cannyBlobDetection(cv::Mat &input, cv::Mat &output);



};

#endif //ZERODISPARITYFILTER_ZDFTHREAD_H
