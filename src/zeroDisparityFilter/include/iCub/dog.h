/* 
 * Copyright (C) 2014 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco on Andrew Dankers ` code mainteined by Vadim Tikhanoff
 * email:   francesco.rea@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __DOG_H
#define __DOG_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//#include <ipp.h>

typedef struct {
    int width;
    int height;
} defSize;

typedef struct {
    int x;
    int y;
    int width;
    int height;
} defRect;

#define PAD_BORD 8

/** 
  * A processing class that constructs on- and off-centre difference-of-Gaussian maps.
  */
class DoG{
	
public:

    /** Constructor.
     * @param imsize Input image width and height for memory allocation.
     */
    DoG(defSize imsize);

    /** Destructor.
     */
    ~DoG();

    /** Processing initiator.
     * @param im Pointer to input image.
     * @param psb_8u Step in bytes through the input image.
     */
    //void proc(Ipp8u* im, int psb_8u);
    void proc(unsigned char* im, int psb_8u);
    void proc(IplImage* im, int psb_8u);
    

    /** Access to the on-centre output.
     * @return Pointer to the on-centre output image.
     */
    //Ipp8u* get_dog_on(){return out_dog_on;}          //on-centre
    unsigned char* get_dog_on(){return out_dog_on;}    //on-centre
	IplImage* get_dog_on_ipl(){ 
		//printf("DoG ON ADDRESS %08X  \n", out_dog_on_image);
		//cv::imshow("Matrix1", cv::cvarrToMat(out_dog_on_image));
		//cv::waitKey(0);
		return out_dog_on_image; }

    /** Access to the off-centre output.
     * @return Pointer to the off-centre output image.
     */
    //Ipp8u* get_dog_off(){return out_dog_off;}        //off-centre
    unsigned char* get_dog_off(){return out_dog_off;}  //off-centre
	IplImage* get_dog_off_ipl(){ 
		//printf("DoG OFF ADDRESS %08X  \n", out_dog_off_image);
		//cv::imshow("Matrix2", cv::cvarrToMat(out_dog_off_image));
		//cv::waitKey(0);
		return out_dog_off_image; }

    /** Access to the magnitude output.
     * @return Pointer to the on/off-centre output image.
     */
    //Ipp8u* get_dog_onoff(){return out_dog_onoff;}        //absolute difference
    unsigned char* get_dog_onoff(){return out_dog_onoff;}           //absolute difference
	IplImage* get_dog_onoff_ipl(){ 
		//printf("DoG ON OFF ADDRESS %08X  \n", out_dog_onoff_image);
		//cv::imshow("Matrix3", cv::cvarrToMat(out_dog_onoff_image));
		//cv::waitKey(0);
		return out_dog_onoff_image; }

    /** Memory width return function.
     * @return Step in bytes through the output image.
     */
    int get_psb(){return psb_o;}

	/** DoG image return function
	 * @return IplImage* 32F
	 */
	IplImage* get_dog_image(){ return dog_image; }

	/** DoG image return function
	* @return IplImage* 8U
	*/
	IplImage* get_dog_image_8u(){ return dog_image_8u; }

    /** 
     * Convert from 32f precision back to 8u
     */
    //void conv_32f_to_8u( Ipp32f* im_i, int p4_, Ipp8u*im_o, int p1_, IppiSize srcsize_);
    void conv_32f_to_8u( float* im_i, int p4_, char *im_o, int p1_, defSize srcsize_);

     /** 
     * Convert from 32f precision back to 8u
     */
    void conv_32f_to_8u(const IplImage* im_i, int p4_, IplImage *im_o, int p1_, defSize srcsize_);
	void conv_32f_to_8u(const cv::Mat *mat_i, int p4_, cv::Mat *mat_o, int p1_, defSize srcsize_);

    /**
     * Convert from 8u precision back to 32f
     */
    void conv_8u_to_32f(const IplImage *im_i, int p4_, IplImage *im_o, int p1_, defSize srcsize_);
    void conv_8u_to_32f(const cv::Mat *mat_i, int p4_, cv::Mat *mat_o, int p1_, defSize srcsize_);

	IplImage* remove_borders(const IplImage* input);

private:
    float *dog;          //Ipp32f
    float *dog_on;       //Ipp32f
    float *dog_off;      //Ipp32f
    float *dog_onoff;    //Ipp32f
    float *tmp1;         //Ipp32f
    float *tmp2;         //Ipp32f
    float *tmp3;         //Ipp32f
    float *in_pad;       //Ipp32f

    IplImage *dog_image;          //Ipp32f
    IplImage *dog_on_image;       //Ipp32f
    IplImage *dog_off_image;      //Ipp32f
    IplImage *dog_onoff_image;    //Ipp32f
    IplImage *tmp1_image;         //Ipp32f
    IplImage *tmp2_image;         //Ipp32f
    IplImage *tmp3_image;         //Ipp32f
    IplImage *in_pad_image;       //Ipp32f
    IplImage *invert_image;    
	IplImage *dog_aux_32f_small;

    unsigned char *in_pad_8u;     //Ipp8u
	unsigned char *out_dog_on;             //Ipp8u
	unsigned char *out_dog_off;            //Ipp8u
	unsigned char *out_dog_onoff;          //Ipp8u

    IplImage *in_pad_8u_image;          //Ipp8u
    IplImage *out_dog_on_image;         //Ipp8u
    IplImage *out_dog_off_image;        //Ipp8u
    IplImage *out_dog_onoff_image;      //Ipp8u
	IplImage *dog_image_8u;
    

    int width, height;
    int psb_o, psb_pad, psb_pad_8u;
    defSize srcsize, psize;

    CvMat* kern1_mat;
    CvMat* kern2_mat;

    cv::Mat tmp1_mat, tmp2_mat, tmp3_mat, dog_mat;
	
};
#endif
