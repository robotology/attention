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

#ifndef __ICUB_ZDFMODULE_H__
#define __ICUB_ZDFMODULE_H__

#include <iostream>
#include <string>

#include <yarp/sig/Image.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <ipp.h>
#include <stdlib.h>
#include <string>
#include "iCub/coord.h"
#include "iCub/dog.h"
#include "iCub/multiclass.h"
#include "yarp/os/Time.h"
#include <yarp/sig/Vector.h>

//use NDT or RANK comparision
#define RANK0_NDT1 0 //0 (NDT FASTER and RANK a little too sensitive)
//NDT:
#define NDTX     2
#define NDTY     2
#define NDTSIZE  8 //4 or 8: 4
//RANK:
#define RANKY    2 //1 or 2
#define RANKX    2  //1 or 2
#define RANKSIZE 25  //9 or 25: 9

#define NDTEQ    1 //0

#define COMMAND_VOCAB_IS VOCAB2('i','s')
#define COMMAND_VOCAB_HELP VOCAB4('h','e','l','p')
#define COMMAND_VOCAB_SET VOCAB3('s','e','t')
#define COMMAND_VOCAB_GET VOCAB3('g','e','t')
#define COMMAND_VOCAB_K1 VOCAB2('k','1') //data penalty
#define COMMAND_VOCAB_K2 VOCAB2('k','2') //smoothness penalty base
#define COMMAND_VOCAB_K3 VOCAB2('k','3') //smoothness penalty
#define COMMAND_VOCAB_K4 VOCAB2('k','4') //radial penalty
#define COMMAND_VOCAB_K5 VOCAB2('k','5') //smoothness 3sigmaon2
#define COMMAND_VOCAB_K6 VOCAB2('k','6') //bland dog thresh
#define COMMAND_VOCAB_K7 VOCAB2('k','7') //bland prob
#define COMMAND_VOCAB_K8 VOCAB2('k','8') //max spread
  
class ZDFThread : public yarp::os::Thread
{
private:

    /*port name strings*/
    std::string inputNameLeft;           //string containing input port name left
	std::string inputNameRight;          //string containing input port name right
    std::string outputNameProb;          //string containing the probability output port name
	std::string outputNameSeg;           //string containing the segmentation output port name
	std::string outputNameDog;           //string containing the difference of gaussian output port name
    std::string outputNameTemp;          //string containing the segmented template output port name
    std::string outputNameCog;
    std::string inputCheckArbiter;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *img_out_prob;   //probability image
	yarp::sig::ImageOf<yarp::sig::PixelMono> *img_out_seg;    //segmentation image
	yarp::sig::ImageOf<yarp::sig::PixelMono> *img_out_dog;    //difference of gaussian image
    yarp::sig::ImageOf<yarp::sig::PixelBgr> *img_out_temp;    //template image

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >  imageInLeft;      //input port cartesian image left
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >  imageInRight;     //input port cartesian image right
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >  imageOutProb;    //output port probability image
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >  imageOutSeg;     //output port segmented image
	yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> >  imageOutDog;     //output port difference of gaussian image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> >  imageOutTemp;     //output port template image

    yarp::os::BufferedPort < yarp::sig::Vector >    cogPort;
    yarp::os::Port inputCheckStatus;
    
	bool allocated; // flag to check if the variables have been already allocated
    bool withArbiter;
    bool startProcessing;
	int psb_in, t_lock_lr, t_lock_ud;
	//Sizes:
	IppiSize srcsize, msize, tsize, tisize, trsize; //ippi variables containing all different sizes
	//Vars:
	int sx,sy;
  	Ipp32f max_v, max_t;
  	int mid_x, mid_y, mid_x_m, mid_y_m, area;
  	double cog_x, cog_y, cog_x_send, cog_y_send, spread, cmp_res;
  	int koffsetx, koffsety;
    int ndt1[NDTSIZE];
  	int ndt2[NDTSIZE];
  	int rank1[RANKSIZE];
  	int rank2[RANKSIZE];

	Coord c;

	int psb_m,psb_t,psb_rest,psb_resv, psb_trgb, psbtemp, psbCopy;
	int nclasses, dpix_y;

	Ipp32f *res_t; 
  	Ipp8u *out, *seg_im, *seg_dog, *fov_l, *fov_r, *zd_prob_8u, *o_prob_8u, *tempImg, *copyImg;
	Ipp8u **p_prob;
  	//templates:
  	Ipp8u *temp_l, *temp_r;
  	//input:
  	Ipp8u *rec_im_ly;
  	Ipp8u *rec_im_ry;

    Ipp8u *yuva_orig_l, *yuva_orig_r;   // yuv+a image    
    Ipp8u** pyuva_l, **pyuva_r;      // yuv+a image used to extract y, u and v plane
    Ipp8u *tmp;
    Ipp8u *first_plane_l, *first_plane_r;      // plane either y or h
    Ipp8u *second_plane_l, *second_plane_r;      // plane either u or v
    Ipp8u *third_plane_l, *third_plane_r;      // plane either v or s

    int psb4, f_psb, s_psb, t_psb;
	//Difference of Gaussian:
	DoG * dl;
  	DoG * dr;
	
	int tl_x, tl_y;
  	int tr_x, tr_y;
  	int waiting;
  	bool update, acquire;
  	int rad_pen,max_rad_pen;
  	double r, rmax, r_deg, l_deg, t_deg, posx, posy, posz, z_;
	
	int width, height;
	double scale;

	//test
	Ipp8u *l_orig, *r_orig;
	int psb;
	IppiRect inroi;
	IppiSize insize;
    IppiSize tempSize;
    int BufferSize;
    
    //string containing module name
    std::string moduleName;

public:

    /**
    * constructor
    */
    ZDFThread( MultiClass::Parameters *parameters, std::string workWith);
    /**
     * destructor
     */
    ~ZDFThread();

    /**
     * allocate all memory and variables
     */    
    //Multiclass
    struct MultiClass::Parameters *params;
	MultiClass * m;
    void allocate(yarp::sig::ImageOf<yarp::sig::PixelBgr> *img);
    void deallocate();
    bool threadInit();     
    void threadRelease();
    void run();
    void onStop();
	
	void get_rank(Coord c,Ipp8u *im, int w, int*list);
	double cmp_rank(int*l1, int*l2);
	void get_ndt(Coord c,Ipp8u *im, int w, int*list);
	double cmp_ndt(int*l1, int*l2);
	void getAreaCoGSpread(Ipp8u*im, int p,IppiSize s, int*parea,double*pdx,double*pdy,double*pspread);
    void setName(std::string module);
};

class zdfMod:public yarp::os::RFModule
{
    /* module parameters */
    std::string moduleName; 
    std::string handlerName;
    std::string workWith;
	
    yarp::os::Port handlerPort;      //a port to handle messages 
    
    struct MultiClass::Parameters parameters; // multi class parameters passed to the thread
    /* pointer to a new thread to be created and started in configure() and stopped in close() */
    ZDFThread *zdfThread;

public:

    bool configure(yarp::os::ResourceFinder &rf); // configure all the module parameters and return true if successful
    bool interruptModule();                       // interrupt, e.g., the ports 
    bool close();                                 // close and shut down the module
    bool respond(const yarp::os::Bottle& command, yarp::os::Bottle& reply);
    double getPeriod(); 
    bool updateModule();
};

#endif
//empty line to make gcc happy
