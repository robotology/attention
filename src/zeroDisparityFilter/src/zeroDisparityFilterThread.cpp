// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Vadim Tikhanoff Andrew Dankers
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

#include "iCub/zeroDisparityFilterThread.h"
#include <cassert>
#include <iCub/dog.h>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "CannotResolve"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


// Display an Ipl* image with openCV
void showIplImage(const IplImage *image, const string a) {
    cv::imshow(a, cv::cvarrToMat(image));
    cv::waitKey(50);
}



ZDFThread::ZDFThread(MultiClass::Parameters *parameters, string workWith) {
    if (workWith == "arbiter" || workWith == "ARBITER") {
        withArbiter = true;
        cout << "will now wait for the vergence module to finish motion" << endl;
    } else {
        withArbiter = false;
        cout << "running normally without attention system" << endl;
    }

    params = parameters;
    img_out_prob = nullptr;
    img_out_seg = nullptr;
    img_out_dog = nullptr;
    img_out_temp = nullptr;
    res_t = nullptr;
    out = nullptr;
    seg_im = nullptr;
    seg_dog = nullptr;
    fov_l = nullptr;
    fov_r = nullptr;
    zd_prob_8u = nullptr;
    o_prob_8u = nullptr;
    tempImg = nullptr;
    copyImg = nullptr;
    p_prob = nullptr;
    //templates:
    temp_l = nullptr, temp_r = nullptr;
    //input:
    rec_im_ly = nullptr;
    rec_im_ry = nullptr;
    yuva_orig_l = nullptr;
    yuva_orig_r = nullptr;
    tmp = nullptr;
    first_plane_l = nullptr;
    second_plane_l = nullptr;
    third_plane_l = nullptr;
    first_plane_r = nullptr;
    second_plane_r = nullptr;
    third_plane_r = nullptr;
    pyuva_l = nullptr;
    pyuva_r = nullptr;
    //Difference of Gaussian:
    dl = nullptr;
    dr = nullptr;
    multiClass = nullptr;
    l_orig = nullptr;
    r_orig = nullptr;

    allocated = false;
    startProcessing = false;

    //HACK: init images, view deallocate for more info. #amaroyo on 04/02/2016
    copyImg_ipl = nullptr;
    fov_r_ipl = nullptr;
    zd_prob_8u_ipl = nullptr;
    o_prob_8u_ipl = nullptr;
    tempImg_ipl = nullptr;
    left_originalImage_ipl = nullptr;
    right_originalImage_ipl = nullptr;
    yuva_orig_l_ipl = nullptr;
    yuva_orig_r_ipl = nullptr;
    tmp_ipl = nullptr;
    first_plane_l_ipl = nullptr;
    second_plane_l_ipl = nullptr;
    third_plane_l_ipl = nullptr;
    first_plane_r_ipl = nullptr;
    second_plane_r_ipl = nullptr;
    third_plane_r_ipl = nullptr;
    rec_im_ly_ipl = nullptr;
    rec_im_ry_ipl = nullptr;
    res_t_ipl = nullptr;
    out_ipl = nullptr;
    seg_im_ipl = nullptr;
    seg_dog_ipl = nullptr;
    fov_l_ipl = nullptr;
    template_left_ipl = nullptr;
    temp_r_ipl = nullptr;


    ndt1 = (int*)malloc(sizeof(int) * params->ndtSize);
    ndt2 = (int*)malloc(sizeof(int) * params->ndtSize);
    rank1 = (int*)malloc(sizeof(int) * params->rankSize);
    rank2 = (int*)malloc(sizeof(int) * params->rankSize);


    yInfo("max_iteration: %d", parameters->iter_max);
    yInfo("randomize_iteration: %d", parameters->randomize_every_iteration);
    yInfo("smoothness_penalty_base: %d", parameters->smoothness_penalty_base);
    yInfo("smoothness_penalty: %d", parameters->smoothness_penalty);
    yInfo("data_penalty: %d", parameters->data_penalty);
    yInfo("smoothness_3sigmaon2: %d", parameters->smoothness_3sigmaon2);
    yInfo("bland_dog_thresh: %d", parameters->bland_dog_thresh);
    yInfo("radial_penalty: %d", parameters->radial_penalty);
    yInfo("acquire_wait: %d", parameters->acquire_wait);
    yInfo("min_area: %d", parameters->min_area);
    yInfo("max_area: %d", parameters->max_area);
    yInfo("max_spread: %d", parameters->max_spread);
    yInfo("cog_snap: %f", parameters->cog_snap);
    yInfo("bland_prob: %f", parameters->bland_prob);


    yInfo("End of the Thread Costructor");
}

ZDFThread::~ZDFThread() {

    //HACK: better call deallocate - it's improved. #amaroyo 04/02/2016
    deallocate();


}

void ZDFThread::setName(string module) {
    this->moduleName = module;
}

bool ZDFThread::threadInit() {
    /* initialize variables and create data-structures if needed */
    yDebug("Start of the Thread Initialization");
    //create all ports
    inputNameLeft = "/" + moduleName + "/imageLeft:i";
    imageInLeft.open(inputNameLeft.c_str());

    inputNameRight = "/" + moduleName + "/imageRight:i";
    imageInRight.open(inputNameRight.c_str());

    inputCheckArbiter = "/" + moduleName + "/status:i";
    inputCheckStatus.open(inputCheckArbiter.c_str());

    outputNameProb = "/" + moduleName + "/imageProb:o";
    imageOutProb.open(outputNameProb.c_str());

    outputNameSeg = "/" + moduleName + "/imageSeg:o";
    imageOutSeg.open(outputNameSeg.c_str());

    outputNameDogLeft = "/" + moduleName + "/imageDog:o";
    imageOutDog.open(outputNameDogLeft.c_str());

    outputNameDogRight = "/" + moduleName + "/imageDogR:o";
    imageOutDogR.open(outputNameDogRight.c_str());

    outputNameTemp = "/" + moduleName + "/imageTemp:o";
    imageOutTemp.open(outputNameTemp.c_str());

    outputNameTemp2 = "/" + moduleName + "/imageTemp2:o";
    imageOutTemp2.open(outputNameTemp2.c_str());

    outputNameGeometry = "/" + moduleName + "/geometry:o";
    outputGeometry.open(outputNameGeometry.c_str());



    outputNameCog = "/" + moduleName + "/cog:o";
    cogPort.open(outputNameCog.c_str());


    yDebug("End of the Thread Initialization");
    return true;
}

void ZDFThread::run() {

    while (!isStopping()) { // the thread continues to run until isStopping() returns true

        yDebug("Cycle Start");
        ImageOf<PixelBgr> *img_in_left = imageInLeft.read(true);
        ImageOf<PixelBgr> *img_in_right = imageInRight.read(true);


        if (img_in_left != nullptr && img_in_right != nullptr) {

            Bottle check;
            check.clear();

            if (withArbiter) {
                inputCheckStatus.read(check, false);
                startProcessing = check.get(0).asString() == "vergence_accomplished";
            } else {
                startProcessing = true;
            }

            if (startProcessing) {
                yDebug("START PROCESSING \n");

                if (!allocated || img_in_left->width() != insize.width || img_in_left->height() != insize.height) {
                    deallocate();
                    allocate(img_in_left);
                }

                yDebug("image size %d %d", srcsize.width, srcsize.height);

                //Rescaling to 320x240 if needed
                if (scale != 1.0) {
                    yInfo("scale is not 1 : Resizing");
                    img_in_left->resize(srcsize.width, srcsize.height);
                    img_in_right->resize(srcsize.width, srcsize.height);

                }


                //Make a copy of the left input Image

                cvCopy((IplImage *) img_in_left->getIplImage(), copyImg_ipl, nullptr);

                copyImg = (unsigned char *) copyImg_ipl->imageData;

                // Copy left and Right image from input Yarp-port
                cvCopy((IplImage *) img_in_left->getIplImage(), left_originalImage_ipl, nullptr);
                cvCopy((IplImage *) img_in_right->getIplImage(), right_originalImage_ipl, nullptr);

            }


            //Preprocess the input image
            preprocessImageHSV(left_originalImage_ipl, rec_im_ly_ipl);
            preprocessImageHSV(right_originalImage_ipl, rec_im_ry_ipl);

            if (acquire) {
                yDebug("Acquiring : set new Region of Interest \n");
                cv::Rect regionInterestRec((srcsize.width - tsize.width) / 2,
                                           (srcsize.height - tsize.height) / 2,
                                           tsize.width, tsize.height);


                cvSetImageROI(rec_im_ly_ipl, regionInterestRec);
                cvCopy(rec_im_ly_ipl, template_left_ipl, nullptr);
                cvResetImageROI(rec_im_ly_ipl);


                cvSetImageROI(rec_im_ry_ipl, regionInterestRec);
                cvCopy(rec_im_ry_ipl, temp_r_ipl, nullptr);
                cvResetImageROI(rec_im_ry_ipl);

            }

            //******************************************************************
            //Create left fovea and find left template in left image
            yDebug("creating left fovea and left template matching \n");
            cv::Rect foveaRec(mid_x, mid_y, foveaSize.width, foveaSize.height);


            cvSetImageROI(rec_im_ly_ipl, foveaRec);
            cvCopy(rec_im_ly_ipl, fov_l_ipl, nullptr);
            cvResetImageROI(rec_im_ly_ipl);

            //******************************************************************
            //Create right fovea and find right template in right image:
            yDebug("creating right fovea and right template matching \n");

            cvSetImageROI(rec_im_ry_ipl, foveaRec);
            cvCopy(rec_im_ry_ipl, fov_r_ipl, nullptr);
            cvResetImageROI(rec_im_ry_ipl);

//            matchTemplate(fov_l_ipl, rec_im_ry_ipl, fov_r_ipl);


            //*****************************************************************
            //Start diffence of gaussian on foveated images
            yDebug("difference of gaussian on foveated images \n");

            //dl->procOpenCv(fov_l_ipl, params->sigma1, params->sigma2);
            //dr->procOpenCv(fov_r_ipl, params->sigma1, params->sigma2);

            CenterSurround *centerSurround = new CenterSurround(fov_l_ipl->width, fov_l_ipl->height, params->sigma1/100);

            IplImage *leftDOG = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
            IplImage *rightDOG = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);


            centerSurround->proc_im_8u(fov_l_ipl, leftDOG);
            centerSurround->proc_im_8u(fov_r_ipl, rightDOG);

            delete centerSurround;


            processDisparityMap(leftDOG, rightDOG);

            //*******************************************************************
            //Do MRF optimization:
            yDebug("performing Markov Random Field optimization \n");
            fov_r = (unsigned char *) fov_r_ipl->imageData;

            p_prob[0] = o_prob_8u;
            p_prob[1] = zd_prob_8u;

            multiClass->proc(fov_r, p_prob); //provide edge map and probability map
            //cache for distribution:

            yDebug("getting the class out \n");
            IplImage *m_class_ipl = multiClass->get_class_ipl();
            out = (unsigned char *) m_class_ipl->imageData;

            //evaluate result:
            getAreaCoGSpread(out, psb_m, foveaSize, &area, &cog_x, &cog_y, &spread);
            yDebug("evaluating result.... \n");
            yDebug("area: %d, cog_x: %f, cog_y: %f, spread: %f", area, cog_x, cog_y, spread);

            cog_x_send = cog_x;
            cog_y_send = cog_y;

            seg_im = (unsigned char *) seg_im_ipl->imageData;
            seg_dog = (unsigned char *) seg_dog_ipl->imageData;


            //we have mask and image  (out)   [0/255]
            //construct masked image  (fov_l) [0..255]

            for (int j = 0; j < foveaSize.height; j++) {
                for (int i = 0; i < foveaSize.width; i++) {
                    if (out[i + j * psb_m] == 0) {
                        seg_im[j * psb_m + i] = 0;
                        seg_dog[j * psb_m + i] = 0;
                    } else {

                        // Gray matching
                        seg_im[j * psb_m + i] = fov_r[j * psb_m + i];
                        // Set the segmentation match in white
                        //seg_im[j * psb_m + i] = 255;
                    }
                }
            }


            //********************************************************************
            //If nice segmentation:
//if(area >= params->min_area && area <= params->max_area && spread <= params->max_spread)           
            yDebug("checking for nice segmentation \n");
            if (false) {
                //don't update templates to image centre any more as we have a nice target
                acquire = false;
                //update templates towards segmentation CoG:
                yDebug("area:%d spread:%f cogx:%f cogy:%f - UPDATING TEMPLATE\n", area, spread, cog_x, cog_y);
                //Bring cog of target towards centre of fovea, SNAP GAZE TO OBJECT:
                cog_x *= params->cog_snap;
                cog_y *= params->cog_snap;


                //We've updated, so reset waiting:

                int floorFov_l =
                        (mid_x_m + ((int) floor(cog_x + 0.5))) + (mid_y_m + ((int) floor(cog_y + 0.5))) * psb_m;
                int floorFov_r =
                        (mid_x_m + ((int) floor(cog_x + 0.5))) + (mid_y_m + ((int) floor(cog_y + 0.5))) * psb_m;

                temp_l = &fov_l[floorFov_l];
                temp_r = &fov_r[floorFov_r];

                waiting = 0;
                //report that we-ve updated templates:
                update = true;
            }
                //Otherwise, just keep previous templates:
            else {
                yDebug("area:%d spread:%f cogx:%f cogy:%f\n", area, spread, cog_x, cog_y);
                waiting++;
                //report that we didn't update template:
                //-----------------------------------------------extract just template

                if (imageOutTemp.getOutputCount() > 0 || imageOutTemp2.getOutputCount() > 0) {
                    cout << " sending template " << endl;
                    int top = -1;
                    int left = -1;
                    int right = -1;
                    int bottom = -1;


                    for (int j = 0; j < foveaSize.height * psb_m; j++) {
                        if ((int) seg_im[j] > 0) {
                            top = j / psb_m;
                            break;
                        }
                    }

                    for (int j = foveaSize.height * psb_m; j > 0; j--) {

                        if ((int) seg_im[j] > 0) {
                            bottom = j / psb_m;
                            break;
                        }
                    }
                    bool Tmpout = false;


                    for (int i = 0; i < foveaSize.width; i++) {
                        for (int j = 0; j < foveaSize.height; j++) {
                            if ((int) seg_im[i + j * psb_m] > 0) {
                                Tmpout = true;
                                break;
                            }
                        }
                        if (Tmpout)break;
                    }
                    Tmpout = false;
                    for (int i = foveaSize.width; i > 0; i--) {
                        for (int j = 0; j < foveaSize.height; j++) {
                            if ((int) seg_im[i + j * psb_m] > 0) {
                                right = i;
                                Tmpout = true;
                                break;
                            }
                        }
                        if (Tmpout)break;
                    }


                    tempSize.width = right - left + 1;
                    tempSize.height = bottom - top + 1;

                    tempImg_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
                    cvSet(tempImg_ipl, cvScalar(0,0,0));
                    tempImg = (unsigned char *) tempImg_ipl->imageData;
                    psbtemp = tempImg_ipl->widthStep;

                    IplImage *original_seg_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
                    cvCopy(left_originalImage_ipl, original_seg_ipl, nullptr);


                    int topBis, bottomBis, leftBis, rightBis;
                    getRoundingBoxSegmented(&topBis, &bottomBis, &leftBis, &rightBis, seg_im_ipl);

                    int yOneSeg = mid_y + topBis;
                    int yTwoSeg = mid_y + bottomBis;
                    int xOneSeg = mid_x + leftBis;
                    int xTwoSeg = mid_x + (rightBis);

                    cv::Point pt1(xOneSeg, yOneSeg);
                    cv::Point pt2(xTwoSeg, yTwoSeg);


                    cv::Rect rRect(pt1, pt2);
                    cvRectangle(original_seg_ipl, pt1, pt2, cvScalar(255, 0, 0), 2);

                    /*
                    cvSetImageROI(left_originalImage_ipl, rRect);
                    IplImage * temp2 =  cvCreateImage(cvSize(rRect.width, rRect.height), IPL_DEPTH_8U,3);
                    cvCopy(left_originalImage_ipl, temp2);
                    cvResetImageROI(left_originalImage_ipl);
                    */

                    int u = 0;
                    int v = 0;
                    for (int j = top; j < bottom + 1; j++) {
                        for (int i = left; i < right + 1; i++) {

                            if ((int) seg_im[i + j * psb_m] > 0) {
                                int x = srcsize.width / 2 - foveaSize.width / 2 + i;
                                int y = srcsize.height / 2 - foveaSize.height / 2 + j;
                                tempImg[x * 3 + y * psbtemp] = copyImg[x * 3 + y * psbCopy];
                                tempImg[x * 3 + y * psbtemp + 1] = copyImg[x * 3 + y * psbCopy + 1];
                                tempImg[x * 3 + y * psbtemp + 2] = copyImg[x * 3 + y * psbCopy + 2];

                            }
                            u++;
                        }
                        u = 0;
                        v++;
                    }


                    if( imageOutTemp.getOutputCount() > 0 ) {

                        // Display temp_img to YarpPort
                        yarp::sig::ImageOf<yarp::sig::PixelBgr> *processingRgbImage = &imageOutTemp.prepare();
                        processingRgbImage->resize(tempImg_ipl->width, tempImg_ipl->height);
                        processingRgbImage->wrapIplImage(tempImg_ipl);
                        imageOutTemp.write();

                    }

                    if( imageOutTemp2.getOutputCount() > 0) {
                        yarp::sig::ImageOf<yarp::sig::PixelBgr> *processingRgbImageBis  = &imageOutTemp2.prepare();
                        processingRgbImageBis->resize(original_seg_ipl->width, original_seg_ipl->height);
                        processingRgbImageBis->wrapIplImage(original_seg_ipl); //temp_r_ipl
                        imageOutTemp2.write();

                        //cvCopy(left_originalImage_ipl, img_out_temp->getIplImage(), nullptr);
                        yDebug("copied image success \n");


                        // HACK: added just to make the deallocation safer. #amaroyo 04/02/2016
                        img_out_temp = nullptr;
                    }

                }


                //*******************************************************************
                //finished extracting
                //retreive only the segmented object in order to send as template
                update = false;
            }

            if (waiting >= 25) { //acquire_wait
                yDebug("Acquiring new target until nice seg (waiting:%d >= acquire_wait:%d)\n", waiting,
                       params->acquire_wait);//acquire_wait
                acquire = true;
            }

            yDebug("preparing the vector target for the COG \n");

            /*Property option;
            option.put("device","gazecontrollerclient");
            option.put("remote","/iKinGazeCtrl");
            option.put("local","/client/gaze");
            yarp::dev::PolyDriver clientGazeCtrl(option);
            yarp::dev::IGazeControl *igaze=nullptr;

            if (clientGazeCtrl.isValid()) {
                clientGazeCtrl.view(igaze);
            }


            int camSel=0;   // select the image plane: 0 (left), 1 (right)
            Vector px(2);   // specify the pixel where to look
            px[0] = cog_x_send;
            px[1] = cog_y_send;
            double z = 10.0;   // distance [m] of the object from the image plane (extended to infinity): yes, you probably need to guess, but it works pretty robustly
            igaze->lookAtMonoPixel(camSel,px,z);    // look!

             */

            //send it all when connections are established
            if (imageOutProb.getOutputCount() > 0) {
                yDebug("Outputing imageProb\n");


                yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage = &imageOutProb.prepare();
                processingMonoImage->resize(o_prob_8u_ipl->width, o_prob_8u_ipl->height);
                processingMonoImage->wrapIplImage(o_prob_8u_ipl); //fov_l_ipl
                imageOutProb.write();

            }
            if (imageOutSeg.getOutputCount() > 0) {
                yDebug("Outputing imageSeg\n");

                yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage = &imageOutSeg.prepare();
                processingMonoImage->resize(seg_im_ipl->width, seg_im_ipl->height);
                processingMonoImage->wrapIplImage(seg_im_ipl);
                imageOutSeg.write();

            }
            if (imageOutDog.getOutputCount() > 0) {
                yDebug("Outputing imageDog\n");

                yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage;


                processingMonoImage = &imageOutDog.prepare();
                processingMonoImage->resize(leftDOG->width, leftDOG->height);
                processingMonoImage->wrapIplImage(leftDOG);
                imageOutDog.write();


                processingMonoImage = &imageOutDogR.prepare();
                processingMonoImage->resize(rightDOG->width, rightDOG->height);
                processingMonoImage->wrapIplImage(rightDOG);
                imageOutDogR.write();

            }

        }
    }
}


void ZDFThread::threadRelease() {
    yDebug("Nothing happens_zeroDisparityFilterMod.cpp_threadRelease");
}

void ZDFThread::onStop() {
    cout << "closing ports.." << endl;
    imageInLeft.interrupt();
    imageInRight.interrupt();
    imageOutProb.interrupt();
    imageOutSeg.interrupt();
    imageOutDog.interrupt();
    imageOutTemp.interrupt();
    inputCheckStatus.interrupt();

    imageInLeft.close();
    imageInRight.close();
    imageOutProb.close();
    imageOutSeg.close();
    imageOutDog.close();
    imageOutTemp.close();
    inputCheckStatus.close();
    cout << "finished cleaning.." << endl;
}

void ZDFThread::deallocate() {

    yDebug("deallocate process started ..........\n");

    dl = nullptr;
    dr = nullptr;
    multiClass = nullptr;
    delete dl;
    delete dr;
    delete multiClass;

    /* HACK: As the very first step is to deallocate and then allocate, some
       of the variables are not initialized, thus, thr program crashes
       when it tries to release non existing images. Setting them to nullptr
       in the constructor and checking them here solves the issue.
       #amaroyo 04/02/2016

    */

    if (tempImg_ipl != nullptr) cvReleaseImage(&tempImg_ipl);
    if (copyImg_ipl != nullptr) cvReleaseImage(&copyImg_ipl);
    if (left_originalImage_ipl != nullptr) cvReleaseImage(&left_originalImage_ipl);
    if (right_originalImage_ipl != nullptr) cvReleaseImage(&right_originalImage_ipl);
    if (yuva_orig_l_ipl != nullptr) cvReleaseImage(&yuva_orig_l_ipl);
    if (yuva_orig_r_ipl != nullptr) cvReleaseImage(&yuva_orig_r_ipl);
    if (tmp_ipl != nullptr) cvReleaseImage(&tmp_ipl);
    if (first_plane_l_ipl != nullptr) cvReleaseImage(&first_plane_l_ipl);
    if (second_plane_l_ipl != nullptr) cvReleaseImage(&second_plane_l_ipl);
    if (third_plane_l_ipl != nullptr) cvReleaseImage(&third_plane_l_ipl);
    if (first_plane_r_ipl != nullptr) cvReleaseImage(&first_plane_r_ipl);
    if (second_plane_r_ipl != nullptr) cvReleaseImage(&second_plane_r_ipl);
    if (third_plane_r_ipl != nullptr) cvReleaseImage(&third_plane_r_ipl);
    free(pyuva_l);
    free(pyuva_r);
    if (rec_im_ly_ipl != nullptr) cvReleaseImage(&rec_im_ly_ipl);
    if (rec_im_ry_ipl != nullptr) cvReleaseImage(&rec_im_ry_ipl);
    if (res_t_ipl != nullptr) cvReleaseImage(&res_t_ipl);
    if (out_ipl != nullptr) cvReleaseImage(&out_ipl);
    if (seg_im_ipl != nullptr) cvReleaseImage(&seg_im_ipl);
    if (seg_dog_ipl != nullptr) cvReleaseImage(&seg_dog_ipl);
    if (fov_l_ipl != nullptr) cvReleaseImage(&fov_l_ipl);
    if (fov_r_ipl != nullptr) cvReleaseImage(&fov_r_ipl);
    if (zd_prob_8u_ipl != nullptr) cvReleaseImage(&zd_prob_8u_ipl);
    if (o_prob_8u_ipl != nullptr) cvReleaseImage(&o_prob_8u_ipl);
    free(p_prob);
    if (template_left_ipl != nullptr) cvReleaseImage(&template_left_ipl);
    if (temp_r_ipl != nullptr) cvReleaseImage(&temp_r_ipl);
    delete img_out_prob;
    delete img_out_seg;
    delete img_out_dog;
    allocated = false;

    yDebug("deallocating sequence ended successfully  \n");
}

void ZDFThread::allocate(ImageOf<PixelBgr> *img) {

    //print("allocating process started............ \n");

    assert (!allocated);


    cout << "Received left input image dimensions: " << img->width() << "x" << img->height() << endl;
    cout << "Received right input image dimensions: " << img->width() << "x" << img->height() << endl;

    width = 320;
    height = 240;
    scale = 1.0;
    scale = ((double) width) / img->width();

    insize.width = img->width();
    insize.height = img->height();

    //print("Scaling to image dimensions: (%d,%d). Scale factor %f\n", width, height, scale);

    BufferSize = 0;
    inroi.x = 0;
    inroi.y = 0;
    inroi.width = img->width();
    inroi.height = img->height();

    srcsize.width = img->width();
    srcsize.height = img->height();

    foveaSize.width = params->fovea_width; //should be taken from ini file // was 128
    foveaSize.height = params->fovea_height; //should be taken from ini file // was 128

    tsize.width = params->fovea_width / 4;  //should be taken from ini file
    tsize.height = params->fovea_height / 4;  //should be taken from ini file

    //t_lock_lr = 32;     //should be taken from ini file
    //t_lock_ud = 32;     //should be taken from ini file

    //tisize.width = tsize.width + 2 * t_lock_lr;
    //tisize.height = tsize.height + 2 * t_lock_ud;

    trsize.width = (srcsize.width - tsize.width) + 1; //289;
    trsize.height = (srcsize.height - tsize.height) + 1; //209;

    mid_x = (srcsize.width - foveaSize.width) / 2;
    mid_y = (srcsize.height - foveaSize.height) / 2;
    mid_x_m = (foveaSize.width - tsize.width) / 2;
    mid_y_m = (foveaSize.height - tsize.height) / 2;

    cog_x = 0.0;
    cog_y = 0.0;
    cog_x_send = 0.0;
    cog_y_send = 0.0;

    if (params->rankOrNDT == 0) {
        koffsetx = params->rankX;
        koffsety = params->rankY;
    } else {
        koffsetx = params->ndtX;
        koffsety = params->ndtY;
    }

    nclasses = 2; // set the number of the classes in the classification
    dpix_y = 0;


    //copyImg     = ippiMalloc_8u_C3( srcsize.width, srcsize.height, &psbCopy);
    copyImg_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    psbCopy = copyImg_ipl->widthStep;

    filtered_r_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    filtered_l_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);


    //l_orig      = ippiMalloc_8u_C4( srcsize.width, srcsize.height, &psb4);
    //r_orig      = ippiMalloc_8u_C4( srcsize.width, srcsize.height, &psb4);
    left_originalImage_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    right_originalImage_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    psb4 = left_originalImage_ipl->widthStep;

    //rec_im_ly   = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb_in);
    //rec_im_ry   = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb_in);
    rec_im_ly_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    rec_im_ry_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    psb_in = rec_im_ly_ipl->widthStep;

    //res_t       = ippiMalloc_32f_C1(trsize.width,trsize.height,&psb_rest);
    res_t_ipl = cvCreateImage(cvSize(trsize.width, trsize.height), IPL_DEPTH_32F, 1);
    psb_rest = res_t_ipl->widthStep;

    //out          = ippiMalloc_8u_C1(foveaSize.width,foveaSize.height, &psb_m);
    //seg_im       = ippiMalloc_8u_C1(foveaSize.width,foveaSize.height, &psb_m);
    //seg_dog      = ippiMalloc_8u_C1(foveaSize.width,foveaSize.height, &psb_m);
    //fov_l        = ippiMalloc_8u_C1(foveaSize.width,foveaSize.height, &psb_m);
    //fov_r        = ippiMalloc_8u_C1(foveaSize.width,foveaSize.height, &psb_m);
    //zd_prob_8u   = ippiMalloc_8u_C1(foveaSize.width,foveaSize.height, &psb_m);
    // o_prob_8u   = ippiMalloc_8u_C1(foveaSize.width,foveaSize.height, &psb_m);
    out_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    seg_im_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    seg_dog_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    fov_l_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    fov_r_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    zd_prob_8u_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    o_prob_8u_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    psb_m = o_prob_8u_ipl->widthStep;

    //p_prob    = (Ipp8u**) malloc(sizeof(Ipp8u*)*nclasses);
    p_prob = (unsigned char **) malloc(sizeof(unsigned char *) * nclasses);

    //yuva_orig_l = ippiMalloc_8u_C1( srcsize.width *4, srcsize.height, &psb4);
    //yuva_orig_r = ippiMalloc_8u_C1( srcsize.width *4, srcsize.height, &psb4);
    yuva_orig_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    yuva_orig_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);


    //tmp               = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &psb );
    tmp_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    psb = tmp_ipl->widthStep;

    //first_plane_l     = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &f_psb);
    first_plane_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    f_psb = first_plane_l_ipl->widthStep;

    //second_plane_l    = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &s_psb);
    second_plane_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    s_psb = second_plane_l_ipl->widthStep;

    //third_plane_l     = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &t_psb);
    third_plane_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    t_psb = third_plane_l_ipl->widthStep;

    //first_plane_r     = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &f_psb);
    //second_plane_r    = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &s_psb);
    //third_plane_r     = ippiMalloc_8u_C1( srcsize.width, srcsize.height, &t_psb);
    first_plane_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    second_plane_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    third_plane_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);

    maskMsize = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);

    pyuva_l = (unsigned char **) malloc(4 * sizeof(unsigned char *));
    pyuva_r = (unsigned char **) malloc(4 * sizeof(unsigned char *));
    pyuva_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    pyuva_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);

    //ippiSet_8u_C1R( 0, zd_prob_8u, psb_m, foveaSize );
    cvSet(zd_prob_8u_ipl, cv::Scalar(0, 0, 0));
    //ippiSet_8u_C1R( 0, o_prob_8u, psb_m, foveaSize );
    cvSet(o_prob_8u_ipl, cv::Scalar(0, 0, 0));

    p_prob[0] = o_prob_8u;
    p_prob[1] = zd_prob_8u;

    //templates:
    //temp_l     = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);
    //temp_r     = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);
    template_left_ipl = cvCreateImage(cvSize(tsize.width, tsize.height), IPL_DEPTH_8U, 1);
    temp_r_ipl = cvCreateImage(cvSize(tsize.width, tsize.height), IPL_DEPTH_8U, 1);
    psb_t = template_left_ipl->widthStep;

//    dl = new DoG(foveaSize);
//    dr = new DoG(foveaSize);


    multiClass = new MultiClass(foveaSize, psb_m, nclasses, params);

    tl_x = 0;
    tl_y = 0;
    tr_x = 0;
    tr_y = 0;


    waiting = 0;


    rmax = sqrt((foveaSize.width / 2.0) * (foveaSize.width / 2.0)
                + (foveaSize.height / 2.0) * (foveaSize.height / 2.0));

    update = false;
    acquire = true;

    img_out_prob = new ImageOf<PixelMono>;
    img_out_prob->resize(foveaSize.width, foveaSize.height);

    img_out_seg = new ImageOf<PixelMono>;
    img_out_seg->resize(foveaSize.width, foveaSize.height);

    img_out_dog = new ImageOf<PixelMono>;
    img_out_dog->resize(foveaSize.width, foveaSize.height);

    cmp_res = 0.0;
    area = 0;
    allocated = true;

    tempImg_ipl = cvCreateImage(cvSize(seg_im_ipl->width, seg_im_ipl->height), IPL_DEPTH_8U, 3);
    tempImg = (unsigned char *) tempImg_ipl->imageData;
    psbtemp = tempImg_ipl->widthStep;
    cvSet(tempImg_ipl, cvScalar(0,0,0));




    //print("allocating process ended successfully \n \n \n");

}


void ZDFThread::get_ndt(Coord c, unsigned char *im, int w, int *list) {

    Coord n;

    int ndt_ind = 0;
    n = c + Coord(1, 0);

    if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= params->ndtEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[c.y * w + c.x] > params->ndtEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;

    ndt_ind++;
    n = c + Coord(0, 1);
    if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= params->ndtEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[c.y * w + c.x] > params->ndtEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;

    ndt_ind++;
    n = c + Coord(-1, 0);
    if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= params->ndtEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[c.y * w + c.x] > params->ndtEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;

    ndt_ind++;
    n = c + Coord(0, -1);
    if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= params->ndtEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[c.y * w + c.x] > params->ndtEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;


    if (params->ndtSize > 4) {

        //diagonals:
        ndt_ind++;
        n = c + Coord(1, 1);
        if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= params->ndtEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[c.y * w + c.x] > params->ndtEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;

        ndt_ind++;
        n = c + Coord(1, -1);
        if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= params->ndtEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[c.y * w + c.x] > params->ndtEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;

        ndt_ind++;
        n = c + Coord(-1, 1);
        if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= params->ndtEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[c.y * w + c.x] > params->ndtEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;

        ndt_ind++;
        n = c + Coord(-1, -1);
        if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= params->ndtEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[c.y * w + c.x] > params->ndtEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;
    }
}

double ZDFThread::cmp_ndt(int *ndt_l, int *ndt_r) {

    int s = 0;

    for (int count = 0; count < params->ndtSize; count++) {
        if (ndt_l[count] == ndt_r[count]) {
            s++;
        }
    }
    return ((double) s) / ((double) params->ndtSize);

}


void ZDFThread::get_rank(Coord c, unsigned char *im, int w, int *list) {
    Coord n;
    int i = 0;

    for (int x = -params->rankX; x <= params->rankX; x++) {
        for (int y = -params->rankY; y <= params->rankY; y++) {

            n = c + Coord(x, y);
            list[i] = im[n.y * w + n.x];
            i++;

        }
    }
}

double ZDFThread::cmp_rank(int *l1, int *l2) {
    int n1 = 0; //number of non-ties for x
    int n2 = 0; //number of non-ties for y
    int is = 0;

    int a1, a2, aa;

    double tau;//,svar,z,prob;

    for (int j = 0; j < params->rankSize; j++) {
        for (int k = j + 1; k < params->rankSize; k++) {
            a1 = l1[j] - l1[k];
            a2 = l2[j] - l2[k];
            aa = a1 * a2;
            if (aa) {
                ++n1;
                ++n2;

                aa > 0 ? ++is : --is;

            } else {
                if (a1) ++n1;
                if (a2) ++n2;
            }
        }
    }

    tau = (is) / (sqrt((float) n1) * sqrt((float) n2));
    // svar = (4.0 * n + 10.0) / (9.0 * n * (n - 1.0));
    // z = tau / sqrt(svar);
    // prob = erfcc(abs(z) / 1.4142136);

    if (tau < 0.0) { tau = 0.0; }

    return tau;
}


void
ZDFThread::getAreaCoGSpread(unsigned char *im_, int psb_, defSize sz_, int *parea, double *pdx, double *pdy,
                            double *spread) {

    double naccum = 0.0, xaccum = 0.0, yaccum = 0.0;
    *spread = 0.0;

    cout << "GET AREA " << sz_.height << " " << sz_.width << endl;

    for (int j = 0; j < sz_.height; j++) {
        for (int i = 0; i < sz_.width; i++) {
            if (im_[j * psb_ + i] != 0) {
                //cout << "GET AREA DOUBLE LOOP" <<endl;
                xaccum += (double) i;
                yaccum += (double) j;
                naccum += 1.0;
            }
        }
    }

    *parea = (int) naccum;
    cout << "naccum " << naccum << endl;
    if (naccum > 0.0) {
        *pdx = -(sz_.width / 2.0 - xaccum / naccum - 1.0);
        *pdy = -(sz_.height / 2.0 - yaccum / naccum - 1.0);

        //get spread:
        for (int j = 0; j < sz_.height; j++) {
            for (int i = 0; i < sz_.width; i++) {
                if (im_[j * psb_ + i] != 0) {
                    *spread += sqrt(
                            fabs((i - sz_.width / 2.0 - 1.0) - (*pdx)) * fabs((i - sz_.width / 2.0 - 1.0) - (*pdx))
                            +
                            fabs((j - sz_.height / 2.0 - 1.0) - (*pdy)) * fabs((j - sz_.height / 2.0 - 1.0) - (*pdy)));
                }
            }
        }

        *spread /= naccum;
    } else {
        *pdx = 0.0;
        *pdy = 0.0;
        *spread = 0.0;
    }
}


// Functions Image processing

/**
 * @param srcImage
 * Preprocess srcImage by changing to HSV colorSpace and return the V plane into destImage
 * @param destImage
 */
void ZDFThread::preprocessImageHSV(IplImage *srcImage, IplImage *destImage) {
    IplImage *hsvImage = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    IplImage *first_plane_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    IplImage *second_plane_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    IplImage *third_plane_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);


    cvCvtColor(srcImage, hsvImage, CV_RGB2HSV_FULL);
    cvSplit(hsvImage, first_plane_ipl, second_plane_ipl, third_plane_ipl, nullptr);
    cvCopy(third_plane_ipl, destImage, nullptr);

    cvReleaseImage(&hsvImage);
    cvReleaseImage(&first_plane_ipl);
    cvReleaseImage(&second_plane_ipl);
    cvReleaseImage(&third_plane_ipl);


}
/**
 * Preprocess srcImage by changing to YUV colorSpace and return the Y plane into destImage
 * @param srcImage
 * @param destImage
 */
void ZDFThread::preprocessImageYUV(IplImage *srcImage, IplImage *destImage) {
    cvCvtColor(srcImage, yuva_orig_l_ipl, CV_RGB2YUV);
    cvSplit(yuva_orig_l_ipl, first_plane_l_ipl, second_plane_l_ipl, third_plane_l_ipl, nullptr);
    cvCopy(third_plane_l_ipl, destImage, nullptr);


}
/**
 * Preprocess srcImage by changing to gray colorSpace  into destImage
 * @param srcImage
 * @param destImage
 */
void ZDFThread::preprocessImageGray(IplImage *srcImage, IplImage *destImage) {
    cvCvtColor(srcImage, destImage, CV_RGB2GRAY);
}


/**
 * Linear filter
 * @param input
 * @param dest
 */
void ZDFThread::filterInputImage(IplImage *input, IplImage *dest) {
    int i;
    const int szInImg = input->imageSize;
    unsigned char *pFilteredInpImg = (unsigned char *) dest->imageData;
    unsigned char *pCurr = (unsigned char *) input->imageData;
    float lambda = .05f;
    const float ul = 1.0f - lambda;
    for (i = 0; i < szInImg; i++) { // assuming same size
        *pFilteredInpImg = (unsigned char) (lambda * *pCurr++ + ul * *pFilteredInpImg++ + .5f);

    }
}

/**
 * Find in the inputImage the best match of templateImage and save it to dstImage
 * @param templateImage
 * @param inputImage
 * @param dstImage
 */
void ZDFThread::matchTemplate(IplImage *templateImage, IplImage *inputImage, IplImage *dstImage) {

    /// Create the result matrix
    cv::Mat img_display = cv::cvarrToMat(inputImage);
    cv::Mat result;
    int result_cols = inputImage->width - templateImage->width + 1;
    int result_rows = inputImage->height - templateImage->height + 1;

    result.create(result_rows, result_cols, CV_32FC1);

    /// Do the Matching and Normalize
    cv::matchTemplate(cv::cvarrToMat(inputImage), cv::cvarrToMat(templateImage), result, CV_TM_SQDIFF_NORMED);
    normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    /// Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::Point matchLoc;

    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
    matchLoc = minLoc;

    cvSetImageROI(inputImage, cvRect(matchLoc.x, matchLoc.y, fov_l_ipl->width, fov_l_ipl->height));
    cvCopy(inputImage, dstImage, nullptr);
    cvResetImageROI(inputImage);

}


/**
 * From the segmented image calcul the best rounding box by finding top, bottom, left, right point
 * @param top
 * @param bottom
 * @param left
 * @param right
 * @param segmentedImage
 */
void ZDFThread::getRoundingBoxSegmented(int *top, int *bottom, int *left, int *right, IplImage *segmentedImage) {
    const int width = segmentedImage->width;
    const int height = segmentedImage->height;
    const unsigned char *ptr_segmentedImage = (unsigned char *) segmentedImage->imageData;

    *top = height;
    *right = -1;
    *left = width;
    *bottom = -1;

    int index = 0;
    for (int i = 0; i < width; ++i)
        for (int j = 0; j < height; ++j) {
            index = i + j * width;
            if (ptr_segmentedImage[index] > 0) {
                if (i < *left) {
                    *left = i;
                }

                if (j < *top) {
                    *top = j;
                }

                if (j > *bottom) {
                    *bottom = j;
                }

                if (i > *right) {
                    *right = i;
                }

            }
        }
}

void ZDFThread::processDisparityMap(IplImage *leftDOG, IplImage *rightDOG){
//*****************************************************************
    //SPATIAL ZD probability map from fov_l and fov_r:
    yDebug("computing the spatial ZD probability map from fov_l and fov_r \n");

    //perform RANK or NDT kernel comparison:
    o_prob_8u = (unsigned char *) o_prob_8u_ipl->imageData;
    zd_prob_8u = (unsigned char *) zd_prob_8u_ipl->imageData;

    int data_penalty = params->data_penalty;
    int bland_dog_thresh = params->bland_dog_thresh;

    unsigned char *p_dogonoff_l = (unsigned char *) leftDOG->imageData;
    unsigned char *p_dogonoff_r = (unsigned char *) rightDOG->imageData;

    for (int j = koffsety; j < foveaSize.height - koffsety; j++) {
        c.y = j;
        for (int i = koffsetx; i < foveaSize.width - koffsetx; i++) {
            c.x = i;
            int index = i + j * leftDOG->width;


            //if either l or r textured at this retinal location:
            if (p_dogonoff_r[index] > bland_dog_thresh && p_dogonoff_l[index] > bland_dog_thresh ) {

                if (params->rankOrNDT == 0) {
                    //use RANK:
                    get_rank(c, (unsigned char *) fov_l_ipl->imageData, fov_l_ipl->widthStep,
                             rank1);   //yDebug("got RANK from left\n");
                    get_rank(c, (unsigned char *) fov_r_ipl->imageData, fov_r_ipl->widthStep,
                             rank2);   //yDebug("got RANK from right\n");
                    cmp_res = cmp_rank(rank1, rank2);
                    //yDebug("compared RANKS \n");
                } else {
                    //use NDT:
                    get_ndt(c, (unsigned char *) fov_l_ipl->imageData, fov_l_ipl->widthStep, ndt1);
                    get_ndt(c, (unsigned char *) fov_r_ipl->imageData, fov_l_ipl->widthStep, ndt2);
                    cmp_res = cmp_ndt(ndt1, ndt2);
                }
                zd_prob_8u[index] = (unsigned char) (cmp_res * 255.0);
            } else {
                //untextured in both l & r, so set to bland prob (ZD):
                zd_prob_8u[index] = (unsigned char) (params->bland_prob * 255.0);//bland_prob
            }

            //RADIAL PENALTY:
            //The further from the origin, less likely it's ZD, so reduce zd_prob radially:
            //current radius:

            r = sqrt((c.x - foveaSize.width / 2.0) * (c.x - foveaSize.width / 2.0) +
                     (c.y - foveaSize.height / 2.0) * (c.y - foveaSize.height / 2.0));

            rad_pen = (int) ((r / rmax) * params->radial_penalty);
            max_rad_pen = zd_prob_8u[index];

            if (max_rad_pen < rad_pen) {
                rad_pen = max_rad_pen;
            }

            //apply radial penalty
            zd_prob_8u[index] -= rad_pen;

            //manufacture NZD prob (other):
            o_prob_8u[index] = (unsigned char) (255 - zd_prob_8u[index]);

        }
    }
}


#pragma clang diagnostic pop
