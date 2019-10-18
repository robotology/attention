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
#include <iostream>
#include <cassert>
#include <iCub/dog.h>
#include <iCub/lbp.h>

#pragma clang diagnostic push
#pragma ide diagnostic ignored "CannotResolve"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;


static IplImage *leftDOG, *rightDOG;

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
    out = nullptr;
    seg_im = nullptr;
    seg_dog = nullptr;
    fov_l = nullptr;
    fov_r = nullptr;
    zd_prob_8u = nullptr;
    o_prob_8u = nullptr;

    p_prob = nullptr;
    //templates:

    pyuva_l = nullptr;
    pyuva_r = nullptr;
    //Difference of Gaussian:
    dl = nullptr;
    dr = nullptr;
    multiClass = nullptr;

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


    ndt1 = (int *) malloc(sizeof(int) * params->ndtSize);
    ndt2 = (int *) malloc(sizeof(int) * params->ndtSize);
    rank1 = (int *) malloc(sizeof(int) * params->rankSize);
    rank2 = (int *) malloc(sizeof(int) * params->rankSize);


    fixationPointX = 0;
    fixationPointY = 0;

    withArbiter = false;

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

    inputNameFixationPoint = "/" + moduleName + "/fixationPoint:i";
    inputFixationPoint.open(inputNameFixationPoint.c_str());

    outputNameProb = "/" + moduleName + "/imageProb:o";
    imageOutProb.open(outputNameProb.c_str());

    outputNameSeg = "/" + moduleName + "/imageSeg:o";
    imageOutSeg.open(outputNameSeg.c_str());

    outputNameDogLeft = "/" + moduleName + "/imageDog:o";
    imageOutDog.open(outputNameDogLeft.c_str());

    outputNameDogRight = "/" + moduleName + "/imageDogR:o";
    imageOutDogR.open(outputNameDogRight.c_str());

    outputNameTemp = "/" + moduleName + "/saliency:o";
    imageOutSaliency.open(outputNameTemp.c_str());

    outputNameTemp2 = "/" + moduleName + "/templateRGB:o";
    imageOutTemplateRGB.open(outputNameTemp2.c_str());

    outputNameGeometry = "/" + moduleName + "/ROIcoordinate:o";
    outputGeometry.open(outputNameGeometry.c_str());

    outputNameCog = "/" + moduleName + "/cog:o";
    cogPort.open(outputNameCog.c_str());


    yDebug("End of the Thread Initialization");
    return true;
}


bool ZDFThread::getFixationPoint(Bottle *receiveFixationBottle) {
    if(receiveFixationBottle != nullptr){

        fixationPointX = receiveFixationBottle->get(1).asInt();
        fixationPointY = receiveFixationBottle->get(2).asInt();
    }


    yDebug("Fixation point %d %d", fixationPointX, fixationPointY);

    return fixationPointX != 0 || fixationPointY != 0;

}


void ZDFThread::run() {

    while (!isStopping()) { // the thread continues to run until isStopping() returns true

        yDebug("Cycle Start");
        ImageOf<PixelBgr> *img_in_left = imageInLeft.read(true);
        ImageOf<PixelBgr> *img_in_right = imageInRight.read(true);


        if (img_in_left != nullptr && img_in_right != nullptr) {

            Bottle check;
            check.clear();

            if (inputFixationPoint.getInputCount()) {
                Bottle *fixationPointBottle = inputFixationPoint.read(false);
                getFixationPoint(fixationPointBottle);

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

                unsigned char *copyImg = (unsigned char *) copyImg_ipl->imageData;

                // Copy left and Right image from input Yarp-port
                cvCopy((IplImage *) img_in_left->getIplImage(), left_originalImage_ipl, nullptr);
                cvCopy((IplImage *) img_in_right->getIplImage(), right_originalImage_ipl, nullptr);

            }


            //Preprocess the input image
            preprocessImageGray(left_originalImage_ipl, rec_im_ly_ipl_YUV);
            preprocessImageGray(right_originalImage_ipl, rec_im_ry_ipl_YUV);

            preprocessImageGray(left_originalImage_ipl, rec_im_ly_ipl);
            preprocessImageGray(right_originalImage_ipl, rec_im_ry_ipl);





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


            cvCopy(left_originalImage_ipl, filtered_l_ipl, nullptr);

            cvSetImageROI(filtered_l_ipl, foveaRec);

            cvSetImageROI(rec_im_ly_ipl, foveaRec);
            cvSetImageROI(rec_im_ly_ipl_YUV, foveaRec);
            cvCopy(rec_im_ly_ipl, fov_l_ipl, nullptr);
            cvResetImageROI(rec_im_ly_ipl);

            //******************************************************************
            //Create right fovea and find right template in right image:
            yDebug("creating right fovea and right template matching \n");
            cv::Rect foveaRecWithOffset(mid_x,  mid_y - params->offsetVertical, foveaSize.width, foveaSize.height);
            cvCopy(right_originalImage_ipl, filtered_r_ipl, nullptr);

            cvSetImageROI(filtered_r_ipl, foveaRecWithOffset);
            cvSetImageROI(rec_im_ry_ipl, foveaRecWithOffset);

            cvSetImageROI(rec_im_ry_ipl_YUV, foveaRecWithOffset);
            cvCopy(rec_im_ry_ipl, fov_r_ipl, nullptr);
            cvResetImageROI(rec_im_ry_ipl);


            //*****************************************************************
            //Start diffence of gaussian on foveated images
            yDebug("difference of gaussian on foveated images \n");


            dl->procOpenCv(rec_im_ly_ipl_YUV, params->sigma1, params->sigma2);
            dr->procOpenCv(rec_im_ry_ipl_YUV, params->sigma1, params->sigma2);

//            cv::Mat lbpContoursLeft = getLBPMat(fov_l_ipl);
//            cv::Mat lbpContoursRight = getLBPMat(fov_r_ipl);


            auto centerSurround = new CenterSurround(fov_l_ipl->width, fov_l_ipl->height, 8);

            leftDOG = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
            rightDOG = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);

            centerSurround->proc_im_8u(fov_l_ipl, leftDOG);
            centerSurround->proc_im_8u(fov_r_ipl, rightDOG);



            delete centerSurround;


            processDisparityMap(dl->get_dog_onoff_ipl(), dr->get_dog_onoff_ipl());

            cvResetImageROI(rec_im_ry_ipl_YUV);
            cvResetImageROI(rec_im_ly_ipl_YUV);
            cvResetImageROI(filtered_l_ipl);
            cvResetImageROI(filtered_r_ipl);




            //*******************************************************************
            //Do MRF optimization:
            yDebug("performing Markov Random Field optimization \n");
            fov_r = (unsigned char *) fov_l_ipl->imageData;

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
            cvSetImageROI(copyImg_ipl, foveaRec);
            cvCopy(copyImg_ipl, tempImg_ipl);
            cvResetImageROI(copyImg_ipl);


            //we have mask and image  (out)   [0/255]
            //construct masked image  (fov_l) [0..255]

            double minVal, maxVal;
            minMaxLoc(cv::cvarrToMat(m_class_ipl), &minVal, &maxVal); //find minimum and maximum intensities


            auto classSeg = static_cast<const int>(minVal);
            cv::Mat temp_mat = cv::cvarrToMat(tempImg_ipl);
            cv::Mat template_originalMat = cv::cvarrToMat(template_original_size);

            for (int j = 0; j < foveaSize.height; j++) {
                for (int i = 0; i < foveaSize.width; i++) {
                    if (out[i + j * psb_m] == classSeg) {
                        seg_im[j * psb_m + i] = 0;
                        seg_dog[j * psb_m + i] = 0;
                        temp_mat.at<cv::Vec3b>(j,i)[0] = 0;
                        temp_mat.at<cv::Vec3b>(j,i)[1] = 0;
                        temp_mat.at<cv::Vec3b>(j,i)[2] = 0;

                    } else {

                        // Gray matching
                        seg_im[j * psb_m + i] = fov_r[j * psb_m + i];
                        // Set the segmentation match in white
                        seg_dog[j * psb_m + i] = 255;

                    }
                }
            }

            temp_mat.copyTo(template_originalMat(cv::Rect(mid_x, mid_y, temp_mat.cols, temp_mat.rows)));

            filterInputImage(seg_im_ipl, seg_im_ipl_filtered);
            seg_im = (unsigned char *) seg_im_ipl_filtered->imageData;

//            seg_im_ipl_filtered = seg_im_ipl;
            //********************************************************************
            //If nice segmentation:
            if (area >= params->min_area && area <= params->max_area && spread <= params->max_spread && false) {
                yDebug("checking for nice segmentation \n");
                //don't update templates to image centre any more as we have a nice target
                acquire = false;
                //update templates towards segmentation CoG:
                yDebug("area:%d spread:%f cogx:%f cogy:%f - UPDATING TEMPLATE\n", area, spread, cog_x, cog_y);
                //Bring cog of target towards centre of fovea, SNAP GAZE TO OBJECT:
//                cog_x *= params->cog_snap;
//                cog_y *= params->cog_snap;


                //We've updated, so reset waiting:

                int floorFov_l =
                        (mid_x_m + ((int) floor(cog_x + 0.5))) + (mid_y_m + ((int) floor(cog_y + 0.5))) * psb_m;
                int floorFov_r =
                        (mid_x_m + ((int) floor(cog_x + 0.5))) + (mid_y_m + ((int) floor(cog_y + 0.5))) * psb_m;



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





                if (imageOutSaliency.getOutputCount()) {

                    // Display temp_img to YarpPort
                    yarp::sig::ImageOf<yarp::sig::PixelMono> *processingRgbImage = &imageOutSaliency.prepare();
                    processingRgbImage->resize(seg_dog_ipl->width, seg_dog_ipl->height);
                    processingRgbImage->wrapIplImage(seg_dog_ipl);
                    imageOutSaliency.write();

                }

                if(area >= params->min_area && area <= params->max_area && spread <= params->max_spread && imageOutTemplateRGB.getOutputCount())
                {

                    yarp::sig::ImageOf<yarp::sig::PixelRgb> *processingRgbImageBis = &imageOutTemplateRGB.prepare();
                    processingRgbImageBis->resize(template_original_size->width, template_original_size->height);
                    processingRgbImageBis->wrapIplImage(template_original_size); //temp_r_ipl
                    imageOutTemplateRGB.write();

                    if (outputGeometry.getOutputCount()) {
                        Bottle &geometry = outputGeometry.prepare();
                        geometry.clear();
                        geometry.addDouble(rRect.tl().x);
                        geometry.addDouble(rRect.tl().y);
                        geometry.addDouble(rRect.br().x);
                        geometry.addDouble(rRect.br().y);
                        outputGeometry.write();

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
            double z = 0.10;   // distance [m] of the object from the image plane (extended to infinity): yes, you probably need to guess, but it works pretty robustly
            igaze->lookAtMonoPixel(camSel,px,z);    // look!

             */

            //send it all when connections are established
            if (imageOutProb.getOutputCount() > 0) {
                yDebug("Outputing imageProb\n");


                yarp::sig::ImageOf<yarp::sig::PixelMono> &probMonoImage = imageOutProb.prepare();
                probMonoImage.resize(o_prob_8u_ipl->width, o_prob_8u_ipl->height);

                cvCopy(o_prob_8u_ipl, (IplImage *) probMonoImage.getIplImage());
                imageOutProb.write();

            }


            if (imageOutSeg.getOutputCount() > 0) {
                yDebug("Outputing imageSeg\n");

                yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage = &imageOutSeg.prepare();
                processingMonoImage->resize(seg_im_ipl->width, seg_im_ipl->height);
                processingMonoImage->wrapIplImage(seg_im_ipl_filtered);
                imageOutSeg.write();

            }
            if (imageOutDog.getOutputCount() > 0) {
                yDebug("Outputing imageDog\n");

                yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage;


                processingMonoImage = &imageOutDog.prepare();
                processingMonoImage->resize(leftDOG->width, leftDOG->height);
                processingMonoImage->wrapIplImage(dl->get_dog_onoff_ipl());
                imageOutDog.write();


                processingMonoImage = &imageOutDogR.prepare();
                processingMonoImage->resize(rightDOG->width, rightDOG->height);
                processingMonoImage->wrapIplImage(dr->get_dog_onoff_ipl());
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
    imageOutSaliency.interrupt();
    inputFixationPoint.interrupt();

    imageInLeft.close();
    imageInRight.close();
    imageOutProb.close();
    imageOutSeg.close();
    imageOutDog.close();
    imageOutSaliency.close();
    inputFixationPoint.close();
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
    if (rec_im_ly_ipl != nullptr) cvReleaseImage(&rec_im_ly_ipl_YUV);
    if (rec_im_ry_ipl != nullptr) cvReleaseImage(&rec_im_ry_ipl_YUV);
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


    copyImg_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);

    filtered_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    filtered_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);



    left_originalImage_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    right_originalImage_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);


    rec_im_ly_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    rec_im_ry_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);

    rec_im_ly_ipl_YUV = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    rec_im_ry_ipl_YUV = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);

    res_t_ipl = cvCreateImage(cvSize(trsize.width, trsize.height), IPL_DEPTH_32F, 1);

    out_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    seg_im_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    seg_dog_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    seg_im_ipl_filtered = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    fov_l_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    fov_r_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    zd_prob_8u_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    o_prob_8u_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 1);
    psb_m = o_prob_8u_ipl->widthStep;


    p_prob = (unsigned char **) malloc(sizeof(unsigned char *) * nclasses);


    yuva_orig_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    yuva_orig_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);


    tmp_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);

    first_plane_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);

    second_plane_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);

    third_plane_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);

    first_plane_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    second_plane_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    third_plane_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);


    pyuva_l = (unsigned char **) malloc(4 * sizeof(unsigned char *));
    pyuva_r = (unsigned char **) malloc(4 * sizeof(unsigned char *));

    cvSet(zd_prob_8u_ipl, cv::Scalar(0, 0, 0));
    cvSet(o_prob_8u_ipl, cv::Scalar(0, 0, 0));

    p_prob[0] = o_prob_8u;
    p_prob[1] = zd_prob_8u;


    template_left_ipl = cvCreateImage(cvSize(tsize.width, tsize.height), IPL_DEPTH_8U, 1);
    temp_r_ipl = cvCreateImage(cvSize(tsize.width, tsize.height), IPL_DEPTH_8U, 1);

    dl = new DoG(foveaSize);
    dr = new DoG(foveaSize);


    multiClass = new MultiClass(foveaSize, psb_m, nclasses, params);



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

    tempImg_ipl = cvCreateImage(cvSize(foveaSize.width, foveaSize.height), IPL_DEPTH_8U, 3);
    cvSet(tempImg_ipl, cvScalar(0, 0, 0));

    template_original_size = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    cvSet(template_original_size, cvScalar(0, 0, 0));




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
    IplImage *hsvImage = cvCreateImage(cvSize(srcImage->width, srcImage->height), IPL_DEPTH_8U, 3);
    IplImage *first_plane_ipl = cvCreateImage(cvSize(srcImage->width, srcImage->height), IPL_DEPTH_8U, 1);
    IplImage *second_plane_ipl = cvCreateImage(cvSize(srcImage->width, srcImage->height), IPL_DEPTH_8U, 1);
    IplImage *third_plane_ipl = cvCreateImage(cvSize(srcImage->width, srcImage->height), IPL_DEPTH_8U, 1);


    cvCvtColor(srcImage, hsvImage, CV_RGB2HSV_FULL);
    cvSplit(hsvImage, first_plane_ipl, second_plane_ipl, third_plane_ipl, nullptr);
    cvCopy(first_plane_ipl, destImage, nullptr);

    cvReleaseImage(&hsvImage);
    cvReleaseImage(&first_plane_ipl);
    cvReleaseImage(&second_plane_ipl);
    cvReleaseImage(&third_plane_ipl);


}

cv::Mat ZDFThread::getLBPMat(IplImage *srcImage1){
    cv::Mat temp;
    cv::Mat imgMat = cv::cvarrToMat(srcImage1);
    cv::Mat lbp = cv::Mat::zeros( imgMat.size(), CV_8UC1 );

    // Variance-based LB  - lbp::OLBP
    lbp::VARLBP(imgMat, lbp, 5, 2);
    cv::Mat norm(lbp);

    normalize(lbp, norm, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    uint8_t* pixelPtr = norm.data;
    int cn = norm.channels();
    for(int i = 0; i < norm.rows; i++){
        for(int j = 0; j < norm.cols; j += cn){
            cv::Scalar_<uint8_t> bgrPixel;
            bgrPixel.val[0] = pixelPtr[i*norm.cols*cn + j*cn + 0];

            if (bgrPixel.val[0] < 255 && bgrPixel.val[0] > 15)
                pixelPtr[i*norm.cols*cn + j*cn + 0] = 255;
            else
                pixelPtr[i*norm.cols*cn + j*cn + 0] = 0;
        }
    }

    return norm;

}


/**
 * Preprocess srcImage by changing to YUV colorSpace and return the Y plane into destImage
 * @param srcImage
 * @param destImage
 */
void ZDFThread::preprocessImageYUV(IplImage *srcImage, IplImage *destImage) {
    cvCvtColor(srcImage, yuva_orig_l_ipl, CV_RGB2HSV);
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
    float lambda = .8f;
    const float ul = 1.0f - lambda;
    for (i = 0; i < szInImg; i++) { // assuming same size
        *pFilteredInpImg = (unsigned char) (lambda * *pCurr++ + ul * *pFilteredInpImg++ + .8f);

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


void ZDFThread::_medianfilter(const element *signal, element *result, int N) {
    //   Move window through all elements of the signal
    for (int i = 2; i < N - 2; ++i) {
        //   Pick up window elements
        element window[5];
        for (int j = 0; j < 5; ++j)
            window[j] = signal[i - 2 + j];

        //Order elements (only half of them)
        for (int j = 0; j < 3; ++j) {
            //   Find position of minimum element
            int min = j;
            for (int k = j + 1; k < 5; ++k)
                if (window[k] < window[min])
                    min = k;
            //   Put found minimum element in its place
            const element temp = window[j];
            window[j] = window[min];
            window[min] = temp;
        }
        //   Get result - the middle element
        result[i - 2] = window[2];
    }
}

void ZDFThread::medianfilter(element *signal, element *result, int N) {
    //   Check arguments
    if (!signal || N < 1)
        return;
    //   Treat special case N = 1
    if (N == 1) {
        if (result)
            result[0] = signal[0];
        return;
    }
    //   Allocate memory for signal extension
    element *extension = new element[N + 4];
    //   Check memory allocation
    if (!extension)
        return;
    //   Create signal extension
    memcpy(extension + 2, signal, N * sizeof(element));
    for (int i = 0; i < 2; ++i) {
        extension[i] = signal[1 - i];
        extension[N + 2 + i] = signal[N - 1 - i];
    }
    //   Call median filter implementation
    _medianfilter(extension, result ? result : signal, N + 4);
    //   Free memory
    delete[] extension;
}

void ZDFThread::processDisparityMapTexture(cv::Mat &leftMat, cv::Mat &rightMat) {
//*****************************************************************
    //SPATIAL ZD probability map from fov_l and fov_r:
    yDebug("computing the spatial ZD probability map from fov_l and fov_r \n");

    cvSet(o_prob_8u_ipl, cv::Scalar(0, 0, 0));
    cvSet(zd_prob_8u_ipl, cv::Scalar(0, 0, 0));



    //perform RANK or NDT kernel comparison:
    o_prob_8u = (unsigned char *) o_prob_8u_ipl->imageData;
    zd_prob_8u = (unsigned char *) zd_prob_8u_ipl->imageData;

    int data_penalty = params->data_penalty;
    int bland_dog_thresh = params->bland_dog_thresh;



    for (int j = koffsety; j < foveaSize.height - koffsety; j++) {
        c.y = j;
        for (int i = koffsetx; i < foveaSize.width - koffsetx; i++) {
            c.x = i;
            int index = i + j * leftDOG->width;



            //if either l or r textured at this retinal location:


            if ((leftMat.data[index] > 0 && rightMat.data[index] > 0) ||
                (leftDOG->imageData[index] > bland_dog_thresh && rightDOG->imageData[index] > bland_dog_thresh)  ) {

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

    medianfilter(o_prob_8u, o_prob_8u, 9);

}



void ZDFThread::processDisparityMap(IplImage* t_leftDOG, IplImage* t_rightDOG) {
//*****************************************************************
    //SPATIAL ZD probability map from fov_l and fov_r:
    yDebug("computing the spatial ZD probability map from fov_l and fov_r \n");

    cvSet(o_prob_8u_ipl, cv::Scalar(0, 0, 0));
    cvSet(zd_prob_8u_ipl, cv::Scalar(0, 0, 0));



    //perform RANK or NDT kernel comparison:
    o_prob_8u = (unsigned char *) o_prob_8u_ipl->imageData;
    zd_prob_8u = (unsigned char *) zd_prob_8u_ipl->imageData;

    int data_penalty = params->data_penalty;
    int bland_dog_thresh = params->bland_dog_thresh;



    for (int j = koffsety; j < foveaSize.height - koffsety; j++) {
        c.y = j;
        for (int i = koffsetx; i < foveaSize.width - koffsetx; i++) {
            c.x = i;
            int index = i + j * leftDOG->width;



            //if either l or r textured at this retinal location:


            if (( t_leftDOG->imageData[index] >  5 || t_rightDOG->imageData[index] > 5 ) &&
                (leftDOG->imageData[index] > bland_dog_thresh || rightDOG->imageData[index] > bland_dog_thresh)  ) {

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

    medianfilter(o_prob_8u, o_prob_8u, 3);

}


#pragma clang diagnostic pop
