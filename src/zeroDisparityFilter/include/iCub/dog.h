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

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>
//#include <ipp.h>

typedef struct {
    int width;
    int height;
} IppiSize;

typedef struct {
    int x;
    int y;
    int width;
    int height;
} IppiRect;

#define PAD_BORD 8

/** 
  * A processing class that constructs on- and off-centre difference-of-Gaussian maps.
  */
class DoG{
	
public:

    /** Constructor.
     * @param imsize Input image width and height for memory allocation.
     */
    DoG(IppiSize imsize);

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

    /** Access to the off-centre output.
     * @return Pointer to the off-centre output image.
     */
    //Ipp8u* get_dog_off(){return out_dog_off;}        //off-centre
    unsigned char* get_dog_off(){return out_dog_off;}  //off-centre

    /** Access to the magnitude output.
     * @return Pointer to the on/off-centre output image.
     */
    //Ipp8u* get_dog_onoff(){return out_dog_onoff;}        //absolute difference
    unsigned char* get_dog_onoff(){return out_dog_onoff;}  //absolute difference

    /** Memory width return function.
     * @return Step in bytes through the output image.
     */
    int get_psb(){return psb_o;}

    /** 
     * Convert from 32f precision back to 8u
     */
    //void conv_32f_to_8u( Ipp32f* im_i, int p4_, Ipp8u*im_o, int p1_, IppiSize srcsize_);
    void conv_32f_to_8u( float* im_i, int p4_, unsigned char *im_o, int p1_, IppiSize srcsize_);

     /** 
     * Convert from 32f precision back to 8u
     */
    void conv_32f_to_8u( IplImage* im_i, int p4_, IplImage *im_o, int p1_, IppiSize srcsize_);

    /**
     * Convert from 8u precision back to 32f
     */
    void conv_8u_to_32f( IplImage *im_i, int p4_, IplImage *im_o, int p1_, IppiSize srcsize_);

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

    unsigned char *in_pad_8u;          //Ipp8u
    unsigned char *out_dog_on;         //Ipp8u
    unsigned char *out_dog_off;        //Ipp8u
    unsigned char *out_dog_onoff;      //Ipp8u

    IplImage *in_pad_8u_image;          //Ipp8u
    IplImage *out_dog_on_image;         //Ipp8u
    IplImage *out_dog_off_image;        //Ipp8u
    IplImage *out_dog_onoff_image;      //Ipp8u
    

    int width,height;
    int psb_o,psb_pad,psb_pad_8u;
    IppiSize srcsize,psize;
	
};
#endif
