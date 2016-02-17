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


DoG::DoG(defSize srcsize_)
{   
    width = 0, height = 0, psb_o = 0, psb_pad = 0, psb_pad_8u = 0;
    srcsize = srcsize_;
    width   = srcsize.width;
    height  = srcsize.height;

    //in_pad_8u = ippiMalloc_8u_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad_8u);
    in_pad_8u_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    //in_pad    = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    in_pad_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height+PAD_BORD*2), IPL_DEPTH_32F, 1);

    //tmp1      = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    tmp1_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F, 1);

    //tmp2      = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    tmp2_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F, 1); 

    //tmp3      = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    tmp3_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F, 1); 

    //dog       = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F, 1); 

	dog_image_8u = cvCreateImage(cvSize(width + PAD_BORD * 2, height + PAD_BORD * 2), IPL_DEPTH_8U, 1);

    //invert image
    invert_image = cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F, 1);

    //dog_on    = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_on_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F, 1);
    
    //dog_off   = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_off_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F, 1);

    //dog_onoff = ippiMalloc_32f_C1(width+PAD_BORD*2,height+PAD_BORD*2,&psb_pad);
    dog_onoff_image =  cvCreateImage(cvSize(width + PAD_BORD*2, height + PAD_BORD*2), IPL_DEPTH_32F, 1);

    //out_dog_on    = ippiMalloc_8u_C1(width,height,&psb_o);
    out_dog_on_image =  cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    //out_dog_off   = ippiMalloc_8u_C1(width,height,&psb_o);
    out_dog_off_image =  cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

    //out_dog_onoff = ippiMalloc_8u_C1(width,height,&psb_o);
    out_dog_onoff_image =  cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);

	dog_aux_32f_small = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);

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
    
    cvSet(invert_image, cvScalar(-1,-1,-1), NULL);
    
}


DoG::~DoG()
{
    cvReleaseImage(&in_pad_8u_image);     // ippFree(in_pad_8u);
    cvReleaseImage(&in_pad_image);        // ippFree(in_pad);
    cvReleaseImage(&tmp1_image);          // ippFree(tmp1);
    cvReleaseImage(&tmp2_image);          // ippFree(tmp2);
    cvReleaseImage(&tmp3_image);          // ippFree(tmp3);
    cvReleaseImage(&dog_image);           // ippFree(dog);
	cvReleaseImage(&dog_image_8u);
    cvReleaseImage(&dog_on_image);        // ippFree(dog_on);
    cvReleaseImage(&dog_off_image);       // ippFree(dog_off);
    cvReleaseImage(&dog_onoff_image);     // ippFree(dog_onoff);
    cvReleaseImage(&out_dog_on_image);    // ippFree(out_dog_on);
    cvReleaseImage(&out_dog_off_image);   // ippFree(out_dog_off);
    cvReleaseImage(&out_dog_onoff_image); // ippFree(out_dog_onoff);    
	cvReleaseImage(&dog_aux_32f_small);
}

void DoG::conv_32f_to_8u( float *im_i, int p4_,char *im_o, int p1_, defSize srcsize_) {

    float min = 0.0; //Ipp32f
    float max = 0.0; //Ipp32f

    //ippiMinMax_32f_C1R( im_i, p4_,srcsize_, &min, &max);
    //if (max == min){max=255.0; min=0.0;}
    //ippiScale_32f8u_C1R(im_i, p4_, im_o, p1_, srcsize_, min, max );    
} 

void DoG::conv_32f_to_8u(const IplImage *im_i, int p4_, IplImage *im_o, int p1_, defSize srcsize_) {
    //float min = 1.0; //Ipp32f
    //float max = 0.0; //Ipp32f
	printf("conv32fto8uIpl IN ADDRESS %08X \n", im_i);

	double minVal, maxVal;
	
	//change to default numbers and modified by amaroyo on 11/02/2016
	cv::Mat mat_i = cv::cvarrToMat(im_i); 
	minMaxLoc(mat_i, &minVal, &maxVal); //find minimum and maximum intensities
	cv::Mat mat_o;
	mat_i.convertTo(mat_o, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
	printf("min Val of conversion23fto8u : %f, max val of conversion23fto8u: %f \n", minVal, maxVal);

	
	/*printf("test data of the _mat \n");
	for (int i = 1; i < 11; i++) {
		for (int j = 1; j < 11; j++) {
			//p++;
			printf(" data inside: %d \n", mat_o.at<char>(i, j));
		}
	}
	*/

	/*
	cv::imshow("MatrixIN", mat_i);
	cv::waitKey(1);
	
	cv::imshow("MatrixOUT", mat_o);
	cv::waitKey(1);
	*/
	
	

	*im_o = (IplImage)mat_o;
	printf("conv32fto8uIpl OUT ADDRESS %08X \n", im_o);
	
}


void DoG::conv_8u_to_32f(const IplImage *im_i, int p4_, IplImage *im_o, int p1_, defSize srcsize_) {
    float min = 0.0; //Ipp32f
    float max = 0.0; //Ipp32f
	printf("conv8uto32fIpl\n");
	//change to default numbers and modified by amaroyo on 11/02/2016
	cv::Mat mat_i = cv::cvarrToMat(im_i);
	cv::Mat mat_o;
	mat_i.convertTo(mat_o, CV_32F, 1.0 / 255.0);;
	*im_o = (IplImage)mat_o;
}

void DoG::conv_8u_to_32f(const cv::Mat *mat_i, int p4_, cv::Mat *mat_o, int p1_, defSize srcsize_) {
	printf("conv 8u to 32f matrix \n");
    //float min = 0.0; //Ipp32f
    //float max = 0.0; //Ipp32f
    //cv::Mat mat_i = cvCloneImage(im_i);
    //cv::Mat mat_o = cvCloneImage(im_o);
	mat_i->convertTo(*mat_o, CV_32F, 1.0 / 255.0);

	double minVal, maxVal;
	minMaxLoc(*mat_o, &minVal, &maxVal); //find minimum and maximum intensities
	printf("min Val : %f, max val: %f \n", minVal, maxVal);


}

void DoG::conv_32f_to_8u(const cv::Mat *mat_i, int p4_, cv::Mat *mat_o, int p1_, defSize srcsize_) {
	printf("conv 32f to 8u matrix \n");
	float min = 0.0; //Ipp32f
	float max = 0.0; //Ipp32f
	//cv::Mat mat_i = cvCloneImage(im_i);
	//cv::Mat mat_o = cvCloneImage(im_o);

	double minVal, maxVal;
	minMaxLoc(*mat_i, &minVal, &maxVal); //find minimum and maximum intensities
	mat_i->convertTo(*mat_o, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

	double minValO, maxValO;
	minMaxLoc(*mat_o, &minValO, &maxValO); //find minimum and maximum intensities
	printf("min Val of output : %f, max val of output: %f \n", minValO, maxValO);

}

void DoG::proc(unsigned char *in_, int psb_in_)
{
	//TODO uncomment this?
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
    printf("cloning the image for border making, size %d by %d \n", in_->width, in_->height);
    
	//IplImage in_ is 8U
    cv::Mat in_mat = cv::cvarrToMat(in_);



    cv::Mat in_pad_8u_mat;
    //ippiCopyReplicateBorder_8u_C1R(in_,psb_in_,srcsize,in_pad_8u,psb_pad_8u,psize,PAD_BORD,PAD_BORD);
    cv::Scalar value = cv::Scalar( 0, 0, 0 );
	copyMakeBorder(in_mat, in_pad_8u_mat, PAD_BORD, PAD_BORD, PAD_BORD, PAD_BORD, cv::BORDER_REPLICATE, value);

	

    //convert to 32f: 
    //ippiConvert_8u32f_C1R(in_pad_8u,psb_pad_8u,in_pad,psb_pad,psize);
    cv::Mat in_pad_mat;
    defSize is; is.width = 0; is.height = 0;
    conv_8u_to_32f(&in_pad_8u_mat, 0, &in_pad_mat, 0, is);
	printf("converted the image to 32f success \n");

	
	//----------------  DOG filtering -----------------------------------------
    printf("DOG filtering \n");

    cv::Point anchor(0, 0);
    //cv::Mat kern1_mat; removed because already defined
    
    //ippiFilterColumn_32f_C1R(&in_pad[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern1,kern_sz,kern_anc);
    filter2D(in_pad_mat, tmp1_mat, CV_32F, (cv::Mat) kern1_mat, anchor); 
    
    //ippiFilterRow_32f_C1R(&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp2[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern1,kern_sz,kern_anc);
    filter2D(tmp1_mat, tmp2_mat, CV_32F, (cv::Mat) kern1_mat, anchor); 
    

    //ippiFilterColumn_32f_C1R(&in_pad[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern2,kern_sz,kern_anc);
    filter2D(in_pad_mat, tmp1_mat, CV_32F, (cv::Mat) kern2_mat, anchor); 
    

    //ippiFilterRow_32f_C1R(&tmp1[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,&tmp3[PAD_BORD*psb_pad/4+PAD_BORD],psb_pad,srcsize,kern2,kern_sz,kern_anc);
    filter2D(tmp1_mat, tmp3_mat, CV_32F, (cv::Mat) kern2_mat, anchor); 

    printf("filtering successfully ended \n");
	
	printf("DoG: Substraction \n");


    //ippiSub_32f_C1R(tmp2,psb_pad,tmp3,psb_pad,dog,psb_pad,psize);
    dog_mat   = tmp2_mat - tmp3_mat;
    //cvSub(&tmp2_mat, &tmp3_mat, dog_image);

	double dog_mat_min, dog_mat_max;
	minMaxLoc(dog_mat, &dog_mat_min, &dog_mat_max); //find minimum and maximum intensities
	printf("min Val of DoG : %f, max val of Dog: %f \n", dog_mat_min, dog_mat_max);

	//we do not remove the borders, we need them for later #amaroyo 11/02/2016
	*dog_image = (IplImage)dog_mat;


	/****************************
	 Debugging and testing

	 //removing borders
	 cv::Rect imgROI(5, 5, in_->width, in_->height);
	 cv::Mat croppedImage = dog_mat(imgROI);
	 cv::Mat copy;
	 croppedImage.copyTo(copy);

	 *dog_image = (IplImage)copy;

	 //convert 32F copy to 8u mat 
	cv::Mat copy_8u;
	conv_32f_to_8u(&copy, 0, &copy_8u, 0, is);

	*dog_image_8u = (IplImage)copy_8u;
	cv::imshow("Matrix", copy_8u);
	cv::waitKey(1);


	printf("test data of the _mat \n");
	for (int i = 1; i < 5; i++) {
		for (int j = 1; j < 5; j++) {
			//p++;
			printf(" data inside: %d \n", copy_8u.at<char>(i, j));
		}
	}

	*/


    //cvCopy(dog_image, &dog_mat.operator IplImage(),NULL); 
    //dog_image = &dog_mat.operator IplImage(); --unnecesary, done before
	printf("DoG: Substraction success \n");

    //---------------  on-centre -----------------------------------------------
    //keep only results above zero:
    //ippiThreshold_LT_32f_C1R(dog,psb_pad,dog_on,psb_pad,psize,0.0);
	printf("sizes: %d == %d and %d == %d \n", dog_image->height, dog_on_image->height, dog_image->width, dog_on_image->width);
	printf("depths: %d == %d \n", dog_image->depth, dog_on_image->depth);
	printf("channels: %d == %d \n", dog_image->nChannels, dog_on_image->nChannels);
	cvThreshold(dog_image, dog_on_image, 0.0, NULL, CV_THRESH_TOZERO); // threshold to Zero; BEFORE: 1.0, CV_THRESH_BINARY, by #amaroyo on 15/02/2016


    //---------------- off-centre ----------------------------------------------
    //negate: 
    //ippiMulC_32f_C1IR(-1.0,dog,psb_pad,psize);
    cvMul(dog_image, invert_image, dog_image, 1);

    //and keep only results above zero:
    //ippiThreshold_LT_32f_C1R(dog,psb_pad,dog_off,psb_pad,psize,0.0);
	cvThreshold(dog_image, dog_off_image, 0.0, NULL, CV_THRESH_TOZERO);
	
    //on+off:
    //ippiAdd_32f_C1R(dog_on,psb_pad,dog_off,psb_pad,dog_onoff,psb_pad,psize);
    cvAdd(dog_on_image, dog_off_image, dog_onoff_image);
	
    //TODO this seems not useful #amaroyo 12/02/2016
	//out_dog_onoff = dog_onoff_image->imageData; 
    

    //convert to 8u and remove pad:
    printf("converting back to 8u and removing padding \n");

    //conv_32f_to_8u(&dog_on   [PAD_BORD * psb_pad / 4 + PAD_BORD], psb_pad, out_dog_on,    psb_o, srcsize);
	printf("IN ADDRESSES %08X  and %08X \n", dog_on_image, out_dog_on_image);
	conv_32f_to_8u(remove_borders(dog_on_image), 0, out_dog_on_image, 0, srcsize);
	printf("OUT ADDRESSES %08X  and %08X \n", dog_on_image, out_dog_on_image);


    //conv_32f_to_8u(&dog_off  [PAD_BORD * psb_pad / 4 + PAD_BORD], psb_pad, out_dog_off,   psb_o, srcsize);
	printf("IN ADDRESSES %08X  and %08X \n", dog_off_image, out_dog_off_image);
	conv_32f_to_8u(remove_borders(dog_off_image), 0, out_dog_off_image, 0, srcsize);
	printf("OUT ADDRESSES %08X  and %08X \n", dog_off_image, out_dog_off_image);


    //conv_32f_to_8u(&dog_onoff[PAD_BORD * psb_pad / 4 + PAD_BORD], psb_pad, out_dog_onoff, psb_o, srcsize);
	conv_32f_to_8u(remove_borders(dog_onoff_image), 0, out_dog_onoff_image, 0, srcsize);

    printf("procedure concluded \n");

}


IplImage* DoG::remove_borders(IplImage* input){

	printf("removing borders \n");
	cv::Rect imgROI(PAD_BORD, PAD_BORD, srcsize.width, srcsize.height);
	cv::Mat dog_mat = cv::cvarrToMat(input);
	cv::Mat croppedImage = dog_mat(imgROI);
	cv::Mat copy;
	croppedImage.copyTo(copy);
	*dog_aux_32f_small = (IplImage)copy;
	printf("removing borders concluded\n");
	return dog_aux_32f_small;
}


