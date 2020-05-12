// -*- mode:C++; tab-width():4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco, Shashank Pathak
 * email:   francesco.rea@iit.it, shashank.pathak@iit.it
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
 * \file centsur.h
 * \brief An implementation modelling the centre-surround response, used for construction of spatial uniqueness maps.
 * Based on the difference-of-Gaussians pyramid approach of Itti. A single Gaussian pyramid is created. Neighbouring pyramid entries are subtracted (eg Pyramid level 0 - Pyramid level 1, 1-2, 2-3 ...etc), and so are 2nd neighbours (eg 1-3,2-4,3-5..etc), to obtain spatial uniqueness at various spatial scales. All resultant subtraction results (the difference of Gaussian maps) are summated to yield the centre-surround map output.
 *
 * \author Andrew Dankers
 * \date 2009
 * \note Release under GNU GPL v2.0
 **/


/**
 * @file centerSurround.h
 * @brief Center-surround(original by Andrew & Vadim) without IPP .
 */

#ifndef __CENTSUR_H__
#define __CENTSUR_H__


#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

#include <yarp/sig/all.h>
#include <yarp/os/all.h>

#include <iCub/convolve.h> 

#define ngs 3
#define N_LANCZOS 7     // 'a' for LANCZOS
#define _PI_CS 3.14159265358979
	
class CenterSurround { 
public:
    /**
    * constructor
    */
	CenterSurround(int _w, int _h,  double sigma = 1.0);
    /**
     * destructor
     */
	~CenterSurround();

    /**
     * convert image to 32f precision
     */
	void proc_im_8u(IplImage* im_8u, IplImage* output8u);

    /**
     * process 32f image creating gauss pyramids:
     */
	void proc_im_32f(IplImage* im_32f, IplImage* output8u);
    
    /**
     * returns gaussians
     */
	CvMat* get_gauss(int s){return gauss[s];}

    /**
     * returns pyramids
     */
	CvMat* get_pyramid(int s){return pyramid[s];}
	
	/**
     * returns Gaussian smoothened pyramids
     */
	CvMat* get_pyramid_gauss(int s){return pyramid_gauss[s];}

    /**
     * get center surround image in 32f precision
     */
	IplImage* get_centsur_32f(){return cs_tot_32f;} 

    /**
     * get center surround image in 8u precision
     */
	IplImage*  get_centsur_norm8u(){return cs_tot_8u;}



private:

    /**
     * creates pyramids
     */
	void make_pyramid(IplImage* im_in);

    
	// Store IplImages for each level of pyramid
	CvMat *pyramid[ngs],*pyramid_gauss[ngs],*gauss[ngs];
	IplImage *cs_tot_32f,*cs_tot_8u,*im_in_32f,*tmp_im_32f;
	

	int ngauss;

	int srcsizeWidth;
	int srcsizeHeight;

	// Image sizes at each level
	int psizeWidth[ngs];
	int psizeHeight[ngs];
	

	// factors for down and up sampling and sigma for Gaussian
	double sd,su,sigma;

	/*float LANCZOS_VECTOR[N_LANCZOS];
	convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelFloat> ,short >* LanczosHorConvolution;
	convolve<yarp::sig::ImageOf<yarp::sig::PixelMono>,uchar,yarp::sig::ImageOf<yarp::sig::PixelFloat> ,short >* LanczosVerConvolution;*/
    
};

#endif

//----- end-of-file --- ( next line intentionally left blank ) ------------------

