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

#ifndef ZERODISPARITYFILTER_ZERODISPARITYTHREAD_H
#define ZERODISPARITYFILTER_ZERODISPARITYTHREAD_H


#include "iCub/multiclass.h"
#include "iCub/centerSurround.h"


class ZDFThread : public yarp::os::Thread {
private:

    /*port name strings*/
    std::string inputNameLeft;           //string containing input port name left
    std::string inputNameRight;          //string containing input port name right
    std::string outputNameProb;          //string containing the probability output port name
    std::string outputNameSeg;           //string containing the segmentation output port name
    std::string outputNameDogLeft;           //string containing the difference of gaussian output port name
    std::string outputNameDogRight;           //string containing the difference of gaussian output port name
    std::string outputNameTemp;          //string containing the segmented template output port name
    std::string outputNameTemp2;          //string containing the segmented template output port name
    std::string outputNameCog;
    std::string inputCheckArbiter;
    std::string outputNameGeometry;

    yarp::sig::ImageOf<yarp::sig::PixelMono> *img_out_prob;   //probability image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *img_out_seg;    //segmentation image
    yarp::sig::ImageOf<yarp::sig::PixelMono> *img_out_dog;    //difference of gaussian image
    yarp::sig::ImageOf<yarp::sig::PixelBgr> *img_out_temp;    //template image
    yarp::sig::ImageOf<yarp::sig::PixelBgr> *img_out_temp2;    //template image

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > imageInLeft;      //input port cartesian image left
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > imageInRight;     //input port cartesian image right
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutProb;     //output port probability image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutSeg;      //output port segmented image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutDog;      //output port difference of gaussian image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono> > imageOutDogR;      //output port difference of gaussian image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > imageOutTemp;     //output port template image
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelBgr> > imageOutTemp2;     //output port template image


    yarp::os::BufferedPort<yarp::os::Bottle> outputGeometry;    // output 2D position and size of the object

    yarp::os::BufferedPort<yarp::sig::Vector> cogPort;
    yarp::os::Port inputCheckStatus;

    bool allocated; // flag to check if the variables have been already allocated
    bool withArbiter;
    bool startProcessing;
    int psb_in, t_lock_lr, t_lock_ud;
    //Sizes:
    defSize srcsize, foveaSize, tsize, tisize, trsize; //ippi variables containing all different sizes
    //Vars:
    int sx, sy;
    float max_v, max_t;              //Ipp32f
    int mid_x, mid_y, mid_x_m, mid_y_m, area;
    double cog_x, cog_y, cog_x_send, cog_y_send, spread, cmp_res;
    int koffsetx, koffsety;
    int *ndt1;
    int *ndt2;
    int *rank1;
    int *rank2;

    Coord c;

    int psb_m, psb_t, psb_rest, psb_resv, psb_trgb, psbtemp, psbCopy;
    int nclasses, dpix_y;

    float *res_t;                                    //Ipp32f
    IplImage *res_t_ipl;
    //Ipp8u *out, *seg_im, *seg_dog, *fov_l, *fov_r, *zd_prob_8u, *o_prob_8u, *tempImg, *copyImg;
    unsigned char * out;
    unsigned char * seg_im, *seg_dog;
    IplImage *out_ipl, *seg_im_ipl, *seg_im_ipl_filtered, *seg_dog_ipl, *fov_l_ipl;
    unsigned char *fov_r, *fov_l;
    unsigned char *tempImg, *copyImg;
    unsigned char *zd_prob_8u, *o_prob_8u;
    IplImage *fov_r_ipl, *zd_prob_8u_ipl, *o_prob_8u_ipl, *tempImg_ipl, *copyImg_ipl;
    IplImage *filtered_r_ipl, *filtered_l_ipl;
    unsigned char **p_prob;                          //Ipp8u **p_prob;

    //templates:
    unsigned char *temp_l, *temp_r;              //Ipp8u *temp_l, *temp_r;
    IplImage *template_left_ipl, *temp_r_ipl;

    //input:x
    unsigned char *rec_im_ly;                             //Ipp8u
    IplImage *rec_im_ly_ipl;
    unsigned char *rec_im_ry;                             //Ipp8u
    IplImage *rec_im_ry_ipl;
    IplImage *maskMsize;
    unsigned char *yuva_orig_l, *yuva_orig_r;         // yuv+a image Ipp8u
    IplImage *yuva_orig_l_ipl, *yuva_orig_r_ipl;     //
    unsigned char **pyuva_l, **pyuva_r;                   // yuv+a image used to extract y, u and v plane Ipp8u
    IplImage *pyuva_l_ipl, *pyuva_r_ipl;
    unsigned char *tmp;                                   //Ipp8u
    IplImage *tmp_ipl;
    unsigned char *first_plane_l, *first_plane_r;       // plane either y or h Ipp8u
    IplImage *first_plane_l_ipl, *first_plane_r_ipl;   //
    unsigned char *second_plane_l, *second_plane_r;         // plane either u or v Ipp8u
    IplImage *second_plane_l_ipl, *second_plane_r_ipl; //
    unsigned char *third_plane_l, *third_plane_r;       // plane either v or s Ipp8u
    IplImage *third_plane_l_ipl, *third_plane_r_ipl;   //


    int psb4, f_psb, s_psb, t_psb;
    //Difference of Gaussian:
    DoG *dl;
    DoG *dr;


    int tl_x, tl_y;
    int tr_x, tr_y;
    int waiting;
    bool update, acquire;
    int rad_pen, max_rad_pen;
    double r, rmax, r_deg, l_deg, t_deg, posx, posy, posz, z_;

    int width, height;
    double scale;

    //test
    unsigned char *l_orig, *r_orig;                   // Ipp8u
    IplImage *left_originalImage_ipl, *right_originalImage_ipl;
    int psb;
    defRect inroi;
    defSize insize;
    defSize tempSize;
    int BufferSize;

    //string containing module name
    std::string moduleName;

    //Get boudning box of the segmented object
    void getRoundingBoxSegmented(int* top, int* bottom, int *left, int *right, IplImage *segmentedImage);



public:

    /**
    * constructor
    */
    ZDFThread(MultiClass::Parameters *parameters, std::string workWith);

    /**
     * destructor
     */
    ~ZDFThread();

    /**
     * allocate all memory and variables
     */
    //Multiclass
    struct MultiClass::Parameters *params;
    MultiClass *multiClass;

    void allocate(yarp::sig::ImageOf<yarp::sig::PixelBgr> *img);

    void deallocate();

    bool threadInit();

    void threadRelease();

    void run();

    void onStop();

    //void get_rank(Coord c,Ipp8u *im, int w, int*list);
    void get_rank(Coord c, unsigned char *im, int w, int *list);

    double cmp_rank(int *l1, int *l2);

    void get_ndt(Coord c, unsigned char *im, int w, int *list); //void get_ndt(Coord c,Ipp8u *im, int w, int*list);
    double cmp_ndt(int *l1, int *l2);

    void getAreaCoGSpread(unsigned char *im, int p, defSize s, int *parea, double *pdx, double *pdy, double *pspread);

    //void getAreaCoGSpread(Ipp8u*im, int p,IppiSize s, int*parea,double*pdx,double*pdy,double*pspread);
    void setName(std::string module);

    void preprocessImageHSV(IplImage *srcImage, IplImage *destImage);
    void preprocessImageYUV(IplImage *srcImage, IplImage *destImage);
    void preprocessImageGray(IplImage *srcImage, IplImage *destImage);
    void filterInputImage(IplImage* input, IplImage* dst);

    void matchTemplate(IplImage* templateImage, IplImage* inputImage,  IplImage* dst);

    void processDisparityMap(IplImage *leftDOG, IplImage *rightDOG);

};
#endif //ZERODISPARITYFILTER_ZERODISPARITYTHREAD_H

