// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 RBCS Robotics Brain and Cognitive Science
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

#ifndef __MULTCL_H__
#define __MULTCL_H__

//#include <ipp.h>
#include <stdlib.h>
#include "iCub/coord.h"
#include "iCub/dog.h"

#include <cv.h>
#include <cvaux.h>
#include <highgui.h>

/** 
  * The neighbourhood structure.
  */
    const struct Coord NEIGHBORS[] = { Coord(1, 0), Coord(0, -1) };
#define NEIGHBOR_NUM (sizeof(NEIGHBORS) / sizeof(Coord))
      
/** 
  * A processing class that performs multi-class segmentation.
  */
class MultiClass{
  
 public:

  /**
   * A structure to accommodate input parameters.
   */
  struct Parameters
  {
      int    iter_max;	
      int    randomize_every_iteration;
     
      int    smoothness_penalty;
      int    smoothness_penalty_base;
      int    smoothness_3sigmaon2;
      int    data_penalty;
      int    bland_dog_thresh; 
      int    radial_penalty; 
      int    acquire_wait;
      int    min_area;
      int    max_area;
      int    max_spread;
      double cog_snap;
      double bland_prob;

  };
	
  /** Constructor.
   * @param imsize Input image width and height for memory allocation.
   * @param psb_in Step in bytes through input images.
   * @param numClasses Number of classes in output, and number of class probability maps provided.
   * @param params Input parameters.
   */
  MultiClass(IppiSize imsize,int psb_in,int numClasses,Parameters *params);
  
  /** 
   *Destructor.
   */
  ~MultiClass();
  
  /** Access to the classification output.
   * @return Pointer to the output classification image.
   */
  //Ipp8u* get_class(){return out;};
  char* get_class(){return out;};

  /** Access to the classification output.
   * @return Pointer to the output classification image.
   */  
  IplImage* get_class_ipl(){return outImage;};
  
  /** Memory width return function.
   * @return Step in bytes through the output image.
   */
  int get_psb(){return psb;};
  
  /** 
   * Processing initiator.
   * @param im_in Pointer to input image to be used for edge smoothness.
   * @param pMaps Reference to the array of pointers to the input class probability maps.
   */
  //void proc(Ipp8u* im_in, Ipp8u** pMaps);
  void proc(char* im_in, char** pMaps);
  
 private:
  int  likelihood(Coord c, int d);
  int  prior_intensity_smoothness(Coord p, Coord np, int d, int nd);
  void generate_permutation(int *buf, int n);
  int  compute_energy();
  void clear();
  void expand(int a);  /* computes the minimum a-expansion configuration */
  
  int nmaps;
  int len_nv;
  int psb_in,psb;
  IppiSize im_size;
  IplImage* outImage;               //outputImage of the class
  Coord im_sz;
  char *im, **prob;        //Ipp8u
  char *out;               //Ipp8u 
  Parameters *params;
  int E;
  void **ptr_im;
  
};

#endif 
      
