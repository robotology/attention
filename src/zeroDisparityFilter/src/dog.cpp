// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Andrew Dankers, maintainer Vadim Tikhanoff
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
#include "iCub/dog.h"
#include <stdio.h>

//set up DoG Kernel stuff:
const int kern_sz  = 7;
const int kern_anc = 3;

//const Ipp32f kern1[] ={18.0,33.0,49.0,55.0,49.0,33.0,18.0};
//const Ipp32f kern2[] ={ 5.0,23.0,59.0,82.0,59.0,23.0,5.0 };
const float kern1[] = {18.0,33.0,49.0,55.0,49.0,33.0,18.0};
const float kern2[] = { 5.0,23.0,59.0,82.0,59.0,23.0,5.0 };


DoG::DoG(IppiSize srcsize_)
{   
    width = 0, height = 0, psb_o = 0, psb_pad = 0, psb_pad_8u = 0;
    srcsize = srcsize_;
    width   = srcsize.width;
    height  = srcsize.height;

    //in_pad_8u = ippiMalloc_8u_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad_8u);
    in_pad_8u_image = cvCreateImage(cvSize(width, height),IPL_DEPTH_8U, 1);

    //in_pad    = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    in_pad_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height+PAD_BORD*2), IPL_DEPTH_32F,1);

    //tmp1      = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    tmp1_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F,1);

    //tmp2      = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    tmp2_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F,1); 

    //tmp3      = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    tmp3_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F,1); 

    //dog       = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F,1); 

    //invert image
    invert_image = cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F,1);

    //dog_on    = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_on_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F,1);
    
    //dog_off   = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_off_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F,1);

    //dog_onoff = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_onoff_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F,1);

    //out_dog_on    = ippiMalloc_8u_C1(width,height,&psb_o);
    out_dog_on_image =  cvCreateImage(cvSize(width, height), IPL_DEPTH_8U,1);

    //out_dog_off   = ippiMalloc_8u_C1(width,height,&psb_o);
    out_dog_off_image =  cvCreateImage(cvSize(width, height), IPL_DEPTH_8U,1);

    //out_dog_onoff = ippiMalloc_8u_C1(width,height,&psb_o);
    out_dog_onoff_image =  cvCreateImage(cvSize(width, height), IPL_DEPTH_8U,1);

    psize.width  = width  + PAD_BORD * 2;
    psize.height = height + PAD_BORD * 2;

    kern1_mat = cvCreateMat(7, 1, CV_32FC1);
    for (int x = 0; x < 7; x++){
			cvmSet(kern1_mat, x, 0, kern1[x]);
    }
    kern2_mat = cvCreateMat(1, 7, CV_32FC1);
    for (int x = 0; x < 7; x++){
			cvmSet(kern2_mat, 0, x, kern1[x]);
    }
    
    cvSet( invert_image, cvScalar(-1,-1,-1), NULL);
    
}


DoG::~DoG()
{
    cvReleaseImage(&in_pad_8u_image);     // ippFree(in_pad_8u);
    cvReleaseImage(&in_pad_image);        // ippFree(in_pad);
    cvReleaseImage(&tmp1_image);          // ippFree(tmp1);
    cvReleaseImage(&tmp2_image);          // ippFree(tmp2);
    cvReleaseImage(&tmp3_image);          // ippFree(tmp3);
    cvReleaseImage(&dog_image);           // ippFree(dog);
    cvReleaseImage(&dog_on_image);        // ippFree(dog_on);
    cvReleaseImage(&dog_off_image);       // ippFree(dog_off);
    cvReleaseImage(&dog_onoff_image);     // ippFree(dog_onoff);
    cvReleaseImage(&out_dog_on_image);    // ippFree(out_dog_on);
    cvReleaseImage(&out_dog_off_image);   // ippFree(out_dog_off);
    cvReleaseImage(&out_dog_onoff_image); // ippFree(out_dog_onoff);    
}

void DoG::conv_32f_to_8u( float *im_i, int p4_,char *im_o, int p1_, IppiSize srcsize_) {

    float min = 0.0; //Ipp32f
    float max = 0.0; //Ipp32f
    //ippiMinMax_32f_C1R( im_i, p4_,srcsize_, &min, &max);
    //if (max == min){max=255.0; min=0.0;}
    //ippiScale_32f8u_C1R(im_i, p4_, im_o, p1_, srcsize_, min, max );    
} 

void DoG::conv_32f_to_8u( IplImage *im_i, int p4_, IplImage *im_o, int p1_, IppiSize srcsize_) {
    float min = 0.0; //Ipp32f
    float max = 0.0; //Ipp32f
    cv::Mat mat_i = cvCloneImage(im_i);
    cv::Mat mat_o = cvCloneImage(im_o);
    mat_i.convertTo(mat_o, CV_8U, max, min);
}


void DoG::conv_8u_to_32f( IplImage *im_i, int p4_, IplImage *im_o, int p1_, IppiSize srcsize_) {
    float min = 0.0; //Ipp32f
    float max = 0.0; //Ipp32f
    cv::Mat mat_i = cvCloneImage(im_i);
    cv::Mat mat_o = cvCloneImage(im_o);
    mat_i.convertTo(mat_o, CV_32F);
}

void DoG::conv_8u_to_32f( cv::Mat *mat_i, int p4_, cv::Mat *mat_o, int p1_, IppiSize srcsize_) {
    float min = 0.0; //Ipp32f
    float max = 0.0; //Ipp32f
    //cv::Mat mat_i = cvCloneImage(im_i);
    //cv::Mat mat_o = cvCloneImage(im_o);
    mat_i->convertTo(*mat_o, CV_32F);
}

void DoG::proc(unsigned char *in_, int psb_in_)
{
    //pad:
    //ippiCopyReplicateBorder_8u_C1R(in_,psb_in_,srcsize,in_pad_8u,psb_pad_8u,psize,PAD_BORD,PAD_BORD);

    //convert to 32f: 
    //ippiConvert_8u32f_C1R(in_pad_8u,psb_pad_8u,in_pad,psb_pad,psize);

    //DOG filtering:
    //ippiFilterColumn_32f_C1R(&in_pad[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern1,kern_sz,kern_anc);
    //ippiFilterRow_32f_C1R(&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp2[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern1,kern_sz,kern_anc);
    //ippiFilterColumn_32f_C1R(&in_pad[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern2,kern_sz,kern_anc);
    //ippiFilterRow_32f_C1R(&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp3[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern2,kern_sz,kern_anc);

    //ippiSub_32f_C1R(tmp2,psb_pad,tmp3,psb_pad,dog,psb_pad,psize);

    //on-centre:
    //keep only results above zero:
    //ippiThreshold_LT_32f_C1R(dog,psb_pad,dog_on,psb_pad,psize,0.0);
    //off-centre:  
    //negate: 
    //ippiMulC_32f_C1IR(-1.0,dog,psb_pad,psize);
    //and keep only results above zero:
    //ippiThreshold_LT_32f_C1R(dog,psb_pad,dog_off,psb_pad,psize,0.0);
    //on+off:
    //ippiAdd_32f_C1R(dog_on,psb_pad,dog_off,psb_pad,dog_onoff,psb_pad,psize);

    //convert to 8u and remove pad:
    //conv_32f_to_8u(&dog_on[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,out_dog_on,psb_o,srcsize);
    //conv_32f_to_8u(&dog_off[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,out_dog_off,psb_o,srcsize);
    //conv_32f_to_8u(&dog_onoff[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,out_dog_onoff,psb_o,srcsize);

}

void DoG::proc(IplImage *in_, int psb_in_)
{
    //padding
    printf("cloning the image for border making \n");
    
    cv::Mat in_mat = cvCloneImage(in_);
    cv::Mat in_pad_8u_mat;
    //ippiCopyReplicateBorder_8u_C1R(in_,psb_in_,srcsize,in_pad_8u,psb_pad_8u,psize,PAD_BORD,PAD_BORD);
    cv::Scalar value = cv::Scalar( 0, 0, 0 );
    copyMakeBorder(in_mat, in_pad_8u_mat, 5, 5, 5, 5, cv::BORDER_REPLICATE, value); 

    //convert to 32f: 
    //ippiConvert_8u32f_C1R(in_pad_8u,psb_pad_8u,in_pad,psb_pad,psize);
    cv::Mat in_pad_mat;
    IppiSize is; is.width = 0; is.height = 0;
    conv_8u_to_32f(&in_pad_8u_mat, 0, &in_pad_mat, 0, is);

    //----------------  DOG filtering -----------------------------------------
    printf("DOG filtering \n");

    cv::Point anchor(0, 0);
    //cv::Mat kern1_mat;
    
    //ippiFilterColumn_32f_C1R(&in_pad[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern1,kern_sz,kern_anc);
    filter2D(in_pad_mat, tmp1_mat, CV_32F, (cv::Mat) kern1_mat, anchor); 
    
    //ippiFilterRow_32f_C1R(&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp2[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern1,kern_sz,kern_anc);
    filter2D(tmp1_mat, tmp2_mat, CV_32F, (cv::Mat) kern1_mat, anchor); 
    

    //ippiFilterColumn_32f_C1R(&in_pad[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern2,kern_sz,kern_anc);
    filter2D(in_pad_mat, tmp1_mat, CV_32F, (cv::Mat) kern2_mat, anchor); 
    

    //ippiFilterRow_32f_C1R(&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp3[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern2,kern_sz,kern_anc);
    filter2D(tmp1_mat, tmp3_mat, CV_32F, (cv::Mat) kern2_mat, anchor); 

    printf("filtering successfully ended \n");

    //ippiSub_32f_C1R(tmp2,psb_pad,tmp3,psb_pad,dog,psb_pad,psize);
    //dog_mat   = tmp2_mat - tmp3_mat;
    //cvSub(&tmp2_mat, &tmp3_mat, dog_image);
    //dog_image = cvCloneImage(&(IplImage)dog_mat);
    //cvCopy(dog_image, &dog_mat.operator IplImage(),NULL); 
    //dog_image = &dog_mat.operator IplImage();

    //---------------  on-centre -----------------------------------------------
    //keep only results above zero:
    //ippiThreshold_LT_32f_C1R(dog,psb_pad,dog_on,psb_pad,psize,0.0);
    cvThreshold(dog_image , dog_on_image, 0.0, 1.0, CV_THRESH_BINARY);

    //---------------- off-centre ----------------------------------------------
    //negate: 
    //ippiMulC_32f_C1IR(-1.0,dog,psb_pad,psize);
    cvMul(dog_image, invert_image, dog_image, 1);

    //and keep only results above zero:
    //ippiThreshold_LT_32f_C1R(dog,psb_pad,dog_off,psb_pad,psize,0.0);
    cvThreshold(dog_image , dog_off_image, 0.0, 1.0, CV_THRESH_BINARY);

    //on+off:
    //ippiAdd_32f_C1R(dog_on,psb_pad,dog_off,psb_pad,dog_onoff,psb_pad,psize);
    cvAdd(dog_on_image, dog_off_image, dog_onoff_image);

    out_dog_onoff = dog_onoff_image->imageData;
    

    //convert to 8u and remove pad:
    printf("converting back to 8u and removing padding \n");
    //conv_32f_to_8u(&dog_on   [PAD_BORD * psb_pad / 4 + PAD_BORD], psb_pad, out_dog_on,    psb_o, srcsize);
    conv_32f_to_8u(dog_on_image, 0,out_dog_on_image, 0,srcsize);
    //conv_32f_to_8u(&dog_off  [PAD_BORD * psb_pad / 4 + PAD_BORD], psb_pad, out_dog_off,   psb_o, srcsize);
    conv_32f_to_8u(dog_off_image, 0, out_dog_off_image,0, srcsize);
    //conv_32f_to_8u(&dog_onoff[PAD_BORD * psb_pad / 4 + PAD_BORD], psb_pad, out_dog_onoff, psb_o, srcsize);
    conv_32f_to_8u(dog_onoff_image, 0, out_dog_onoff_image, 0, srcsize);
    printf("procedure concluded \n");

}

