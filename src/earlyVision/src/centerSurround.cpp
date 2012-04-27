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
        pyramid[ng]         = cvCreateMat(psizeHeight[ng],psizeWidth[ng],CV_32FC1);
        pyramid_gauss[ng]   = cvCreateMat(psizeHeight[ng],psizeWidth[ng],CV_32FC1);
        gauss[ng]           = cvCreateMat(srcsizeHeight,srcsizeWidth,CV_32FC1);
        
    }

    
    im_in_32f  = cvCreateImage(cvSize(srcsizeWidth,srcsizeHeight),IPL_DEPTH_32F,1); 
    tmp_im_32f = cvCreateImage(cvSize(srcsizeWidth,srcsizeHeight),IPL_DEPTH_32F,1);
    cs_tot_32f = cvCreateImage(cvSize(srcsizeWidth,srcsizeHeight),IPL_DEPTH_32F,1);
    cs_tot_8u  = cvCreateImage(cvSize(srcsizeWidth,srcsizeHeight),IPL_DEPTH_8U,1);

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

    cvReleaseImage(&im_in_32f);
    cvReleaseImage(&tmp_im_32f);
    cvReleaseImage(&cs_tot_32f);
    cvReleaseImage(&cs_tot_8u);
    
    for (int ng=0;ng<ngauss;ng++) {
        cvReleaseMat(&pyramid[ng]);
        cvReleaseMat(&pyramid_gauss[ng]);
        cvReleaseMat(&gauss[ng]);
    }
    //delete LanczosHorConvolution;
    //delete LanczosVerConvolution;
}

void CenterSurround::proc_im_8u(IplImage* input_8u, IplImage* output8u)
{
    //convert im precision to 32f and process as normal:
    cvConvertScale(input_8u,im_in_32f,0.003922,0);  //  0.003922 = 1/255.0     
    proc_im_32f(im_in_32f,output8u);
}

void CenterSurround::proc_im_32f(IplImage* im_32f, IplImage* output8u)
{
    //make image & gauss pyramids:
    make_pyramid(im_32f);

    //reset tot cs_tot_tmp:
    cvSet(cs_tot_32f,cvScalar(0));
	
    //subtractions (ABSDIFF) to make DOG pyramid:
    //add to create the final response
    // TODO : try not linear operation rather than addition
  	//1st neighbours:  
    for (int nd=0;nd<ngauss-1;nd++){
        cvAbsDiff(gauss[nd],gauss[nd+1],tmp_im_32f);        
        cvAdd(tmp_im_32f,cs_tot_32f,cs_tot_32f);
        
    }

  	//2nd neighbours:
  	for (int ndd=0;ndd<ngauss-2;ndd++){
    	cvAbsDiff(gauss[ndd],gauss[ndd+2],tmp_im_32f);
        cvAdd(tmp_im_32f,cs_tot_32f,cs_tot_32f);
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
  	                
    cvConvertScale(cs_tot_32f,output8u,255,0);
     
}

void CenterSurround::make_pyramid( IplImage* im_32f) {
    //copy im to pyramid[0]:
    cvCopy(im_32f,pyramid[0],NULL);

    //filter first pyramid:
    cvSmooth(pyramid[0],pyramid_gauss[0],CV_GAUSSIAN,KERNSIZE,KERNSIZE,sigma);
    
    //copy filter output (within padding) to gauss:
    cvCopy(pyramid_gauss[0],gauss[0]);
  
    //others:
    sd = 0.5;
    su = 2.0;
    int interpolation = CV_INTER_CUBIC;// IPPI_INTER_LANCZOS is not available in openCV, CV_INTER_AREA is rough
    for (int sg=1;sg<ngauss;sg++){
        //Downsize previous pyramid image by half:
        
        
        cvResize(pyramid[sg-1],pyramid[sg],interpolation);
        
        //filter:
        cvSmooth(pyramid[sg],pyramid_gauss[sg],CV_GAUSSIAN,KERNSIZE,KERNSIZE,sigma);
        
    
        //Upsize and store to gauss:
        //su = pow( 2.0f, sg );
        su = double(1<< sg); // a bit faster....
        
        
        cvResize(pyramid_gauss[sg],gauss[sg],interpolation);
            
    }
}


