// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2011  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Rea Francesco, Shashank Pathak
  * email:francesco.rea@iit.it, shashank.pathak@iit.it
  * Permission is granted to copy, distribute, and/or modify this program
  * under the terms of the GNU General Public License, version 2 or any
  * later version published by the Free Software Foundation.
  *
  * A copy of the license can be found at
  *http://www.robotcub.org/icub/license/gpl.txt
  *
  * This program is distributed in the hope that it will be useful, but
  * WITHOUT ANY WARRANTY; without even the implied warranty of
  * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
  * Public License for more details
*/
/**
 * @file centerSurround.cpp
 * @brief This is re-implementation of center-surround(originally by Andrew Dankers & Vadim Tikhanoff) without IPP .
 */

#include <math.h>
#include <iostream>
#include <stdlib.h>
#include "iCub/centerSurround.h"

#ifdef WITH_CUDA
#include <iCub/cudaVision/cudaVision.h>
#endif

#define KERNSIZE 3    //kernsize (odd, >= 3)
#define PI 3.1415

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;

CenterSurround::CenterSurround(int width,int height, double sigma_)
{
    ngauss = 0;

    srcsizeWidth = width;
    srcsizeHeight = height;
    ngauss = ngs;
    sigma = sigma_;


    for (int ng=0;ng<ngauss;ng++){
        psizeWidth[ng]   = (int)ceil( ( (double)srcsizeWidth )/double(1<< ng) );//pow(2.0f, ng));
        psizeHeight[ng]  = (int)ceil( ( ((double)srcsizeHeight)/ ( (double)srcsizeWidth) ) * psizeWidth[ng] );
        pyramid[ng]         = new cv::Mat(psizeHeight[ng],psizeWidth[ng],CV_32FC1);
        pyramid_gauss[ng]   = new cv::Mat(psizeHeight[ng],psizeWidth[ng],CV_32FC1);
        gauss[ng]           = new cv::Mat(srcsizeHeight,srcsizeWidth,CV_32FC1);
    }


    im_in_32f = new cv::Mat(srcsizeHeight, srcsizeWidth, CV_32FC1);
    tmp_im_32f = new cv::Mat(srcsizeHeight, srcsizeWidth, CV_32FC1);
    cs_tot_32f = new cv::Mat(srcsizeHeight, srcsizeWidth, CV_32FC1);
    cs_tot_8u = new cv::Mat(srcsizeHeight, srcsizeWidth, CV_32FC1);

/*    // initialize LANCZOS window for filtering in spatial domain

    float ONE_BY_N_1 = 1/(N_LANCZOS -1);
    for(int i=0; i<N_LANCZOS; ++i){

        float _x = PI*(2*i*ONE_BY_N_1 -1);
        LANCZOS_VECTOR[i]= sin(_x)/_x;

    }

    LanczosHorConvolution = new convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelFloat> ,short > (N_LANCZOS,LANCZOS_VECTOR,0,.5,0);
    LanczosVerConvolution = new convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelFloat> ,short > (N_LANCZOS,LANCZOS_VECTOR,1,.5,0);

*/


}

CenterSurround::~CenterSurround() {

    delete im_in_32f;
    delete tmp_im_32f;
    delete cs_tot_32f;
    delete cs_tot_8u;

    im_in_32f = nullptr;
    tmp_im_32f = nullptr;
    cs_tot_32f = nullptr;
    cs_tot_8u = nullptr;


    
    for (int ng=0;ng<ngauss;ng++) {
        delete pyramid[ng];
        delete pyramid_gauss[ng];
        delete pyramid[ng];

        pyramid[ng] = nullptr;
        pyramid_gauss[ng] = nullptr;
        gauss[ng] = nullptr;
    }

    //delete LanczosHorConvolution;
    //delete LanczosVerConvolution;
}

void CenterSurround::proc_im_8u(cv::Mat* input_8u, cv::Mat* output8u)
{
    //convert im precision to 32f and process as normal:
    input_8u->convertTo(*im_in_32f,CV_32F,0.003922,0);
    proc_im_32f(im_in_32f,output8u);
}

void CenterSurround::proc_im_32f(cv::Mat* im_32f, cv::Mat* output8u)
{
    //make image & gauss pyramids:
    make_pyramid(im_32f);

    //reset tot cs_tot_tmp:
    cs_tot_32f->setTo(cv::Scalar(0));
	
    //subtractions (ABSDIFF) to make DOG pyramid:
    //add to create the final response
    // TODO : try not linear operation rather than addition
  	//1st neighbours:  
    for (int nd=0;nd<ngauss-1;nd++){
        cv::absdiff(*gauss[nd],*gauss[nd+1],*tmp_im_32f);
        cv::add(*tmp_im_32f,*cs_tot_32f,*cs_tot_32f);
        
    }

  	//2nd neighbours:
  	for (int ndd=0;ndd<ngauss-2;ndd++){
    	cv::absdiff(*gauss[ndd],*gauss[ndd+2],*tmp_im_32f);
        cv::add(*tmp_im_32f,*cs_tot_32f,*cs_tot_32f);
  	}

    
    
  	//This scaling can be avoided
  	//double minPixelVal, maxPixelVal;
    /*minPixelVal = 1000; // arbitrary
    maxPixelVal = -1000;
  	float* ptrcs_tot_32f = (float*)cs_tot_32f->imageData; 
    for(int i=0; i<cs_tot_32f->height;i++){
        for(int j=0; j<cs_tot_32f->width; ++j){
            float now = *ptrcs_tot_32f++;
            minPixelVal = minPixelVal> now? now:minPixelVal;
            maxPixelVal = maxPixelVal<now? now:maxPixelVal;
        }
    }*/

  	//if (maxPixelVal == minPixelVal)
    //maxPixelVal=255.0f;minPixelVal=0.0f;
    cs_tot_32f->convertTo(*output8u,CV_8UC1,255,0);

}

void CenterSurround::make_pyramid(cv::Mat* im_32f) {
#ifdef WITH_CUDA   


#else
    //copy im to pyramid[0]:
    *pyramid[0] = im_32f->clone();


    //filter first pyramid:
    cv::GaussianBlur(*pyramid[0], *pyramid_gauss[0], cv::Size(KERNSIZE, KERNSIZE), sigma);

    //copy filter output (within padding) to gauss:
    *pyramid[0] = pyramid_gauss[0]->clone();

    //others:
    sd = 0.5;
    su = 2.0;
    int interpolation = cv::INTER_CUBIC;// IPPI_INTER_LANCZOS is not available in openCV, CV_INTER_AREA is rough

    for (int sg=1;sg<ngauss;sg++){
        //Downsize previous pyramid image by half:
        cv::resize(*pyramid[sg - 1], *pyramid[sg], pyramid[sg]->size(), 0, 0, interpolation);

        cv::GaussianBlur(*pyramid[sg], *pyramid_gauss[sg], cv::Size(KERNSIZE, KERNSIZE), sigma);

        //Upsize and store to gauss:
        //su = pow( 2.0f, sg );
        su = double(1<< sg); // a bit faster....

        cv::resize(*pyramid_gauss[sg], *gauss[sg], gauss[sg]->size(), 0, 0, interpolation);
            
    }
#endif
}


