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

#include "iCub/zeroDisparityFilterMod.h"
#include <cassert>

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


bool zeroDisparityFilterMod::configure(yarp::os::ResourceFinder &rf) {
    /* Process all parameters from both command-line and .ini file */

    /* get the module name which will form the stem of all module port names */
    moduleName = rf.check("name",
                          Value("zeroDisparityFilterMod"),
                          "module name (string)").asString();

    /*
    Default Ini file values: #amaroyo 03/03/2016

        max_iteration 1
        randomize_iteration 0
        smoothness_penalty_base 50
        smoothness_penalty 600
        data_penalty 106
        smoothness_3sigmaon2 13
        bland_dog_thresh 1
        radial_penalty 50
        acquire_wait 25
        min_area  5000
        max_area  10000
        max_spread 50
        cog_snap 0.3
        bland_prob 0.3

    */

    setName(moduleName.c_str());
    parameters.iter_max = rf.findGroup("PARAMS").check("max_iteration", Value(80), "what did the user select?").asInt();
    parameters.randomize_every_iteration = rf.findGroup("PARAMS").check("randomize_iteration", Value(0),
                                                                        "what did the user select?").asInt();
    parameters.smoothness_penalty_base = rf.findGroup("PARAMS").check("smoothness_penalty_base", Value(50),
                                                                      "what did the user select?").asInt();
    parameters.smoothness_penalty = rf.findGroup("PARAMS").check("smoothness_penalty", Value(600),
                                                                 "what did the user select?").asInt();
    parameters.data_penalty = rf.findGroup("PARAMS").check("data_penalty", Value(106),
                                                           "what did the user select?").asInt();
    parameters.smoothness_3sigmaon2 = rf.findGroup("PARAMS").check("smoothness_3sigmaon2", Value(13),
                                                                   "what did the user select?").asInt();
    parameters.bland_dog_thresh = rf.findGroup("PARAMS").check("bland_dog_thresh", Value(250),
                                                               "what did the user select?").asInt();
    parameters.radial_penalty = rf.findGroup("PARAMS").check("radial_penalty", Value(50),
                                                             "what did the user select?").asInt();
    parameters.acquire_wait = rf.findGroup("PARAMS").check("acquire_wait", Value(25),
                                                           "what did the user select?").asInt();
    parameters.min_area = rf.findGroup("PARAMS").check("min_area", Value(5000), "what did the user select?").asInt();
    parameters.max_area = rf.findGroup("PARAMS").check("max_area", Value(10000), "what did the user select?").asInt();
    parameters.max_spread = rf.findGroup("PARAMS").check("max_spread", Value(50), "what did the user select?").asInt();
    parameters.cog_snap = rf.findGroup("PARAMS").check("cog_snap", Value(0.3), "what did the user select?").asDouble();
    parameters.bland_prob = rf.findGroup("PARAMS").check("bland_prob", Value(0.3),
                                                         "what did the user select?").asDouble();

    parameters.fovea_width = rf.findGroup("PARAMS").check("fovea_size", Value(128),
                                                          "what did the user select?").asInt();
    parameters.fovea_height = rf.findGroup("PARAMS").check("fovea_size", Value(128),
                                                           "what did the user select?").asInt();




    /*
     * attach a port of the same name as the module (prefixed with a /) to the module
     * so that messages received from the port are redirected to the respond method
     */

    workWith = rf.check("with", Value("nothing"), "work with arbiter (string)").asString();


    handlerName = "/";
    handlerName += getName();         // use getName() rather than a literal 

    if (!handlerPort.open(handlerName.c_str())) {
        cout << getName() << ": Unable to open port " << handlerName << endl;
        return false;
    }

    attach(handlerPort);               // attach to port
    //attachTerminal();                // attach to terminal (maybe not such a good thing...)

    /* create the thread and pass pointers to the module parameters */
    zdfThread = new ZDFThread(&parameters, workWith);

    /*pass the name of the module in order to create ports*/
    zdfThread->setName(moduleName);

    /* now start the thread to do the work */
    zdfThread->start(); // this calls threadInit() and it if returns true, it then calls run()

    return true;
}

/* Called periodically every getPeriod() seconds */

bool zeroDisparityFilterMod::updateModule() {
    return true;
}

bool zeroDisparityFilterMod::interruptModule() {

    handlerPort.interrupt();
    return true;
}

bool zeroDisparityFilterMod::close() {
    handlerPort.close();
    zdfThread->stop();
    cout << "deleting thread " << endl;
    delete zdfThread;
    return true;
}

double zeroDisparityFilterMod::getPeriod() {
    return 0.1;
}


bool zeroDisparityFilterMod::respond(const Bottle &command, Bottle &reply) {

    //bool ok = false;
    //bool rec = false; // is the command recognized?

    // mutex.wait();
    switch (command.get(0).asVocab()) {
        case COMMAND_VOCAB_HELP: {
            //rec = true;
            string helpMessage = string(getName().c_str()) +
                                 " commands are: \n" +
                                 "help \n" +
                                 "quit \n";
            reply.clear();
            //ok = true;
            break;
        }

        case COMMAND_VOCAB_SET: {
            //rec = true;
            switch (command.get(1).asVocab()) {

                case COMMAND_VOCAB_K1: {
                    int w = command.get(2).asInt();
                    zdfThread->params->data_penalty = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K2: {
                    int w = command.get(2).asInt();
                    zdfThread->params->smoothness_penalty_base = w;
                    //ok = true;
                    break;

                }

                case COMMAND_VOCAB_K3: {
                    int w = command.get(2).asInt();
                    zdfThread->params->smoothness_penalty = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K4: {
                    int w = command.get(2).asInt();
                    zdfThread->params->radial_penalty = w;
                    //ok = true;
                    break;

                }

                case COMMAND_VOCAB_K5: {
                    int w = command.get(2).asInt();
                    zdfThread->params->smoothness_3sigmaon2 = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K6: {
                    int w = command.get(2).asInt();
                    zdfThread->params->bland_dog_thresh = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K7: {
                    int w = command.get(2).asInt();
                    //zdfThread->params->sigmaOne = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K8: {
                    int w = command.get(2).asInt();
                    //zdfThread->params->sigmaTwo = w;
                    //ok = true;
                    break;
                }
                default:
                    break;
            }
            break;
        }
        case COMMAND_VOCAB_GET: {
            //rec = true;

            reply.addVocab(COMMAND_VOCAB_IS);
            reply.add(command.get(1));

            switch (command.get(1).asVocab()) {

                case COMMAND_VOCAB_K1: {
                    double w = zdfThread->params->data_penalty;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K2: {
                    double w = zdfThread->params->smoothness_penalty_base;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K3: {
                    double w = zdfThread->params->smoothness_penalty;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K4: {
                    double w = zdfThread->params->radial_penalty;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K5: {
                    double w = zdfThread->params->smoothness_3sigmaon2;
                    reply.addDouble(w);
                    break;
                    //ok = true;
                }

                case COMMAND_VOCAB_K6: {
                    double w = zdfThread->params->bland_dog_thresh;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K7: {
                   // double w = zdfThread->params->sigmaOne;
                    //reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K8: {
//                    double w = zdfThread->params->sigmaTwo;
//                    reply.addDouble(w);
                    break;
                    //ok = true;
                }

                default: {
                    break;
                }
            }
            break;
        }
        default:
            break;
    }

    return true;
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
    img_out_prob = NULL;
    img_out_seg = NULL;
    img_out_dog = NULL;
    img_out_temp = NULL;
    res_t = NULL;
    out = NULL;
    seg_im = NULL;
    seg_dog = NULL;
    fov_l = NULL;
    fov_r = NULL;
    zd_prob_8u = NULL;
    o_prob_8u = NULL;
    tempImg = NULL;
    copyImg = NULL;
    p_prob = NULL;
    //templates:
    temp_l = NULL, temp_r = NULL;
    //input:
    rec_im_ly = NULL;
    rec_im_ry = NULL;
    yuva_orig_l = NULL;
    yuva_orig_r = NULL;
    tmp = NULL;
    first_plane_l = NULL;
    second_plane_l = NULL;
    third_plane_l = NULL;
    first_plane_r = NULL;
    second_plane_r = NULL;
    third_plane_r = NULL;
    pyuva_l = NULL;
    pyuva_r = NULL;
    //Difference of Gaussian:
    dl = NULL;
    dr = NULL;
    multiClass = NULL;
    l_orig = NULL;
    r_orig = NULL;

    allocated = false;
    startProcessing = false;

    //HACK: init images, view deallocate for more info. #amaroyo on 04/02/2016
    copyImg_ipl = NULL;
    fov_r_ipl = NULL;
    zd_prob_8u_ipl = NULL;
    o_prob_8u_ipl = NULL;
    tempImg_ipl = NULL;
    l_orig_ipl = NULL;
    r_orig_ipl = NULL;
    yuva_orig_l_ipl = NULL;
    yuva_orig_r_ipl = NULL;
    tmp_ipl = NULL;
    first_plane_l_ipl = NULL;
    second_plane_l_ipl = NULL;
    third_plane_l_ipl = NULL;
    first_plane_r_ipl = NULL;
    second_plane_r_ipl = NULL;
    third_plane_r_ipl = NULL;
    rec_im_ly_ipl = NULL;
    rec_im_ry_ipl = NULL;
    res_t_ipl = NULL;
    out_ipl = NULL;
    seg_im_ipl = NULL;
    seg_dog_ipl = NULL;
    fov_l_ipl = NULL;
    template_left_ipl = NULL;
    temp_r_ipl = NULL;


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


    /*
    delete dl;
    delete dr;
    delete m;
    cvReleaseImage(&copyImg_ipl);        //ippiFree(copyImg);
    cvReleaseImage(&l_orig_ipl);         //ippiFree(l_orig);
    cvReleaseImage(&r_orig_ipl);         //ippiFree(r_orig);
    cvReleaseImage(&yuva_orig_l_ipl);    //ippiFree(yuva_orig_l);
    cvReleaseImage(&yuva_orig_r_ipl);    //ippiFree(yuva_orig_r);
    cvReleaseImage(&tmp_ipl);            //ippiFree(tmp);
    cvReleaseImage(&first_plane_l_ipl);  //ippiFree(first_plane_l);
    cvReleaseImage(&second_plane_l_ipl); //ippiFree(second_plane_l);
    cvReleaseImage(&third_plane_l_ipl);  //ippiFree(third_plane_l);
    cvReleaseImage(&first_plane_r_ipl);  //ippiFree(first_plane_r);
    cvReleaseImage(&second_plane_r_ipl); //ippiFree(second_plane_r);
    cvReleaseImage(&third_plane_r_ipl);  //ippiFree(third_plane_r);
    free(pyuva_l);
    free(pyuva_r);
    cvReleaseImage(&rec_im_ly_ipl);  //ippiFree(rec_im_ly);
    cvReleaseImage(&rec_im_ry_ipl);  //ippiFree(rec_im_ry);
    cvReleaseImage(&res_t_ipl);      //ippiFree(res_t);
    cvReleaseImage(&out_ipl);        //ippiFree(out);
    cvReleaseImage(&seg_im_ipl);     //ippiFree(seg_im);
    cvReleaseImage(&seg_dog_ipl);    //ippiFree(seg_dog);
    cvReleaseImage(&fov_l_ipl);      //ippiFree(fov_l);
    cvReleaseImage(&fov_r_ipl);      //ippiFree(fov_r);
    cvReleaseImage(&zd_prob_8u_ipl); //ippiFree(zd_prob_8u);
    cvReleaseImage(&o_prob_8u_ipl); //ippiFree(o_prob_8u);
    free(p_prob);
    cvReleaseImage(&template_left_ipl);     //ippiFree(temp_l);
    cvReleaseImage(&temp_r_ipl);     //ippiFree(temp_r);
    delete img_out_prob;
    delete img_out_seg;
    delete img_out_dog;

    */

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

        if (img_in_left != NULL && img_in_right != NULL) {

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
                cvCopy((IplImage *) img_in_left->getIplImage(), copyImg_ipl, NULL);
                copyImg = (unsigned char *) copyImg_ipl->imageData;

                // Copy left and Right image from input Yarp-port
                cvCopy((IplImage *) img_in_left->getIplImage(), l_orig_ipl, NULL);
                cvCopy((IplImage *) img_in_right->getIplImage(), r_orig_ipl, NULL);

            }
            //Preprocess the input image
            //filterInputImage(l_orig_ipl, filtered_l_ipl);
            //filterInputImage(r_orig_ipl, filtered_r_ipl);
            preprocessImageHSV(l_orig_ipl, rec_im_ly_ipl);
            preprocessImageHSV(r_orig_ipl, rec_im_ry_ipl);

            if (acquire) {

                yDebug("Acquiring : set new Region of Interest \n");
                cvSetImageROI(rec_im_ly_ipl, cvRect((srcsize.width - tsize.width) / 2,
                                                    (srcsize.height - tsize.height) / 2,
                                                    tsize.width, tsize.height));
                cvCopy(rec_im_ly_ipl, template_left_ipl, NULL);
                cvResetImageROI(rec_im_ly_ipl);

                cvSetImageROI(rec_im_ry_ipl, cvRect((srcsize.width - tsize.width) / 2,
                                                    (srcsize.height - tsize.height) / 2,
                                                    tsize.width, tsize.height));
                cvCopy(rec_im_ry_ipl, temp_r_ipl, NULL);
                cvResetImageROI(rec_im_ry_ipl);

            }

            //******************************************************************
            //Create left fovea and find left template in left image
            yDebug("creating left fovea and left template matching \n");

            cvSetImageROI(rec_im_ly_ipl, cvRect(mid_x, mid_y, foveaSize.width, foveaSize.height));
            cvCopy(rec_im_ly_ipl, fov_l_ipl, NULL);
            cvResetImageROI(rec_im_ly_ipl);

            //******************************************************************
            //Create right fovea and find right template in right image:
            yDebug("creating right fovea and right template matching \n");

//            cvSetImageROI(rec_im_ry_ipl, cvRect(mid_x, mid_y, foveaSize.width, foveaSize.height));
//            cvCopy(rec_im_ry_ipl, fov_r_ipl, NULL);
//            cvResetImageROI(rec_im_ry_ipl);

              matchTemplate(fov_l_ipl, rec_im_ry_ipl, fov_r_ipl);


            //*****************************************************************
            //Start diffence of gaussian on foveated images
            yDebug("difference of gaussian on foveated images \n");

            dl->procOpenCv(fov_l_ipl, 17, 2);
            dr->procOpenCv(fov_r_ipl, 17, 2);


            //*****************************************************************
            //SPATIAL ZD probability map from fov_l and fov_r:
            yDebug("computing the spatial ZD probability map from fov_l and fov_r \n");

            //perform RANK or NDT kernel comparison:
            o_prob_8u = (unsigned char *) o_prob_8u_ipl->imageData;
            zd_prob_8u = (unsigned char *) zd_prob_8u_ipl->imageData;

//            yDebug("Koffsetx: %d, width: %d \n", koffsetx, foveaSize.width);
//            yDebug("Koffsety: %d, height: %d \n", koffsety, foveaSize.height);

            int data_penalty = params->data_penalty;
            int bland_dog_thresh = params->bland_dog_thresh;

            unsigned char *p_dogonoff_l = (unsigned char *) dl->get_dog_onoff_ipl()->imageData;
            unsigned char *p_dogonoff_r = (unsigned char *) dr->get_dog_onoff_ipl()->imageData;

            for (int j = koffsety; j < foveaSize.height - koffsety; j++) {

                c.y = j;
                for (int i = koffsetx; i < foveaSize.width - koffsetx; i++) {

                    c.x = i;
                    int index = i + j * dl->get_dog_onoff_ipl()->width;


                    //if either l or r textured at this retinal location:
                    if (p_dogonoff_l[index] > data_penalty ||
                        p_dogonoff_r[index] > bland_dog_thresh) {

                        if (RANK0_NDT1 == 0) {
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

                        //yDebug("preparing zerodisparity probability mono channel \n");
                        zd_prob_8u[index] = (unsigned char) (cmp_res * 255.0);
                    } else {
                        //yDebug("untextured in both l & r \n");
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
            IplImage *test = dr->get_dog_onoff_ipl();

            for (int j = 0; j < foveaSize.height; j++) {
                for (int i = 0; i < foveaSize.width; i++) {
                    if (out[i + j * psb_m] == 0) {
                        seg_im[j * psb_m + i] = 0;
                        seg_dog[j * psb_m + i] = 0;
                    } else {

                        // Gray matching
                        //seg_dog[j * psb_m + i] = test->imageData[j * test->widthStep + i];
                        seg_im[j * psb_m + i] = fov_r[j * psb_m + i];
                        // Set the segmentation match in white
                        //seg_im[j * psb_m + i] = 255;
                    }
                }
            }


            //********************************************************************
            //If nice segmentation:
            yDebug("checking for nice segmentation \n");
            if (area >= params->min_area && area <= params->max_area && spread <= params->max_spread) {
                //don't update templates to image centre any more as we have a nice target
                acquire = false;
                //update templates towards segmentation CoG:
                yDebug("area:%d spread:%f cogx:%f cogy:%f - UPDATING TEMPLATE\n", area, spread, cog_x, cog_y);
                //Bring cog of target towards centre of fovea, SNAP GAZE TO OBJECT:
                cog_x *= params->cog_snap;
                cog_y *= params->cog_snap;


                //floor(val + 0.5) instead of round
                //ippiCopy_8u_C1R(&fov_l[( mid_x_m + ( (int) floor ( cog_x + 0.5 ) ) ) + ( mid_y_m + ( ( int ) floor ( cog_y + 0.5) ) ) * psb_m], psb_m, temp_l, psb_t, tsize );
                //ippiCopy_8u_C1R(&fov_r[( mid_x_m + ( (int) floor ( cog_x + 0.5 ) ) ) + ( mid_y_m + ( ( int ) floor ( cog_y + 0.5) ) ) * psb_m], psb_m, temp_r, psb_t, tsize );

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

                if (imageOutTemp.getOutputCount() > 0) {
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
                                left = i;
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

                    int u = 0;
                    int v = 0;
                    tempSize.width = right - left + 1;
                    tempSize.height = bottom - top + 1;

                    //tempImg = ippiMalloc_8u_C3( tempSize.width, tempSize.height, &psbtemp);
                    tempImg_ipl = cvCreateImage(cvSize(tempSize.width, tempSize.height), IPL_DEPTH_8U, 3);
                    tempImg = (unsigned char *) tempImg_ipl->imageData;
                    psbtemp = tempImg_ipl->widthStep;

                    IplImage *original_seg_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U,
                                                               3);
                    cvCopy(l_orig_ipl, original_seg_ipl, NULL);

                    int yOneSeg = mid_y + top;
                    int yTwoSeg = mid_y + (bottom - top);
                    int xOneSeg = mid_x + left;
                    int xTwoSeg = mid_x + (right - left);

                    cv::Point pt1(xOneSeg, yOneSeg);
                    cv::Point pt2(xTwoSeg, yTwoSeg);

                    cvRectangle(original_seg_ipl, pt1, pt2, CvScalar(255, 0, 0), 2);


                    img_out_temp = new ImageOf<PixelBgr>;
                    img_out_temp->resize(tempSize.width, tempSize.height);

                    for (int j = top; j < bottom + 1; j++) {
                        for (int i = left; i < right + 1; i++) {
                            if ((int) seg_im[i + j * psb_m] > 0) {
                                int x = srcsize.width / 2 - foveaSize.width / 2 + i;
                                int y = srcsize.height / 2 - foveaSize.height / 2 + j;
                                tempImg[u * 3 + v * psbtemp] = copyImg[x * 3 + y * psbCopy];
                                tempImg[u * 3 + v * psbtemp + 1] = copyImg[x * 3 + y * psbCopy + 1];
                                tempImg[u * 3 + v * psbtemp + 2] = copyImg[x * 3 + y * psbCopy + 2];

                            } else {
                                tempImg[u * 3 + v * psbtemp] = 0;
                                tempImg[u * 3 + v * psbtemp + 1] = 0;
                                tempImg[u * 3 + v * psbtemp + 2] = 0;
                            }
                            u++;
                        }
                        u = 0;
                        v++;
                    }

                    //ippiCopy_8u_C3R( tempImg, psbtemp, img_out_temp->getRawImage(), img_out_temp->getRowSize() , tempSize);

                    // Display temp_img to YarpPort
                    yarp::sig::ImageOf<yarp::sig::PixelBgr> *processingRgbImage;
                    processingRgbImage = &imageOutTemp.prepare();
                    processingRgbImage->resize(tempImg_ipl->width, tempImg_ipl->height);
                    IplImage *aux = tempImg_ipl;
                    processingRgbImage->wrapIplImage(aux); //temp_r_ipl
                    imageOutTemp.write();



                    //cvCopy(l_orig_ipl, img_out_temp->getIplImage(), NULL);
                    yDebug("copied image success \n");


                    // HACK: added just to make the deallocation safer. #amaroyo 04/02/2016
                    img_out_temp = NULL;


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
            yarp::dev::IGazeControl *igaze=NULL;

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
                //ippiCopy_8u_C1R( zd_prob_8u, psb_m, img_out_prob->getRawImage(), img_out_prob->getRowSize(), foveaSize );
                //cvCopy(zd_prob_8u_ipl, img_out_prob->getIplImage(), NULL);
                //imageOutProb.prepare() = *img_out_prob;
                //imageOutProb.write();

                yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage;
                processingMonoImage = &imageOutProb.prepare();
                processingMonoImage->resize(foveaSize.width, foveaSize.height);
                IplImage *aux = o_prob_8u_ipl;
                processingMonoImage->wrapIplImage(aux); //fov_l_ipl
                imageOutProb.write();

            }
            if (imageOutSeg.getOutputCount() > 0) {
                yDebug("Outputing imageSeg\n");
                //ippiCopy_8u_C1R( seg_im, psb_m, img_out_seg->getRawImage(), img_out_seg->getRowSize(), foveaSize );
                //imageOutSeg.prepare() = *img_out_seg;
                //imageOutSeg.write();

                yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage;
                processingMonoImage = &imageOutSeg.prepare();
                processingMonoImage->resize(foveaSize.width, foveaSize.height);
                IplImage *aux = seg_im_ipl;
                processingMonoImage->wrapIplImage(aux);
                imageOutSeg.write();

            }
            if (imageOutDog.getOutputCount() > 0) {
                yDebug("Outputing imageDog\n");
                //ippiCopy_8u_C1R( seg_dog, psb_m, img_out_dog->getRawImage(), img_out_dog->getRowSize(), foveaSize );
                //imageOutDog.prepare() = *img_out_dog;
                //imageOutDog.write();

                yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage;
                processingMonoImage = &imageOutDog.prepare();
                processingMonoImage->resize(dl->get_dog_onoff_ipl()->width, dl->get_dog_onoff_ipl()->height);
                //IplImage *aux = seg_dog_ipl;
                IplImage *aux = dl->get_dog_onoff_ipl();
                processingMonoImage->wrapIplImage(aux); //fov_r_ipl
                imageOutDog.write();


                yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImageR;


                processingMonoImageR = &imageOutDogR.prepare();
                processingMonoImageR->resize(dr->get_dog_onoff_ipl()->width, dr->get_dog_onoff_ipl()->height);
                //IplImage *aux = seg_dog_ipl;
                IplImage *auxR = dr->get_dog_onoff_ipl();
                processingMonoImageR->wrapIplImage(auxR); //fov_r_ipl
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

    dl = NULL;
    dr = NULL;
    multiClass = NULL;
    delete dl;
    delete dr;
    delete multiClass;

    /* HACK: As the very first step is to deallocate and then allocate, some
       of the variables are not initialized, thus, thr program crashes
       when it tries to release non existing images. Setting them to NULL
       in the constructor and checking them here solves the issue.
       #amaroyo 04/02/2016

    */

    if (tempImg_ipl != NULL) cvReleaseImage(&tempImg_ipl);
    if (copyImg_ipl != NULL) cvReleaseImage(&copyImg_ipl);
    if (l_orig_ipl != NULL) cvReleaseImage(&l_orig_ipl);
    if (r_orig_ipl != NULL) cvReleaseImage(&r_orig_ipl);
    if (yuva_orig_l_ipl != NULL) cvReleaseImage(&yuva_orig_l_ipl);
    if (yuva_orig_r_ipl != NULL) cvReleaseImage(&yuva_orig_r_ipl);
    if (tmp_ipl != NULL) cvReleaseImage(&tmp_ipl);
    if (first_plane_l_ipl != NULL) cvReleaseImage(&first_plane_l_ipl);
    if (second_plane_l_ipl != NULL) cvReleaseImage(&second_plane_l_ipl);
    if (third_plane_l_ipl != NULL) cvReleaseImage(&third_plane_l_ipl);
    if (first_plane_r_ipl != NULL) cvReleaseImage(&first_plane_r_ipl);
    if (second_plane_r_ipl != NULL) cvReleaseImage(&second_plane_r_ipl);
    if (third_plane_r_ipl != NULL) cvReleaseImage(&third_plane_r_ipl);
    free(pyuva_l);
    free(pyuva_r);
    if (rec_im_ly_ipl != NULL) cvReleaseImage(&rec_im_ly_ipl);
    if (rec_im_ry_ipl != NULL) cvReleaseImage(&rec_im_ry_ipl);
    if (res_t_ipl != NULL) cvReleaseImage(&res_t_ipl);
    if (out_ipl != NULL) cvReleaseImage(&out_ipl);
    if (seg_im_ipl != NULL) cvReleaseImage(&seg_im_ipl);
    if (seg_dog_ipl != NULL) cvReleaseImage(&seg_dog_ipl);
    if (fov_l_ipl != NULL) cvReleaseImage(&fov_l_ipl);
    if (fov_r_ipl != NULL) cvReleaseImage(&fov_r_ipl);
    if (zd_prob_8u_ipl != NULL) cvReleaseImage(&zd_prob_8u_ipl);
    if (o_prob_8u_ipl != NULL) cvReleaseImage(&o_prob_8u_ipl);
    free(p_prob);
    if (template_left_ipl != NULL) cvReleaseImage(&template_left_ipl);
    if (temp_r_ipl != NULL) cvReleaseImage(&temp_r_ipl);
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

    if (RANK0_NDT1 == 0) {
        koffsetx = RANKX;
        koffsety = RANKY;
    } else {
        koffsetx = NDTX;
        koffsety = NDTY;
    }

    nclasses = 2; // set the number of the classes in the classification
    dpix_y = 0;


    //copyImg     = ippiMalloc_8u_C3( srcsize.width, srcsize.height, &psbCopy);
    copyImg_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    psbCopy = copyImg_ipl->widthStep;

    filtered_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    filtered_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);


    //l_orig      = ippiMalloc_8u_C4( srcsize.width, srcsize.height, &psb4);
    //r_orig      = ippiMalloc_8u_C4( srcsize.width, srcsize.height, &psb4);
    l_orig_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    r_orig_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    psb4 = l_orig_ipl->widthStep;

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

    dl = new DoG(foveaSize);
    dr = new DoG(foveaSize);

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

    //print("allocating process ended successfully \n \n \n");

}


void ZDFThread::get_ndt(Coord c, unsigned char *im, int w, int *list) {

    Coord n;

    int ndt_ind = 0;
    n = c + Coord(1, 0);

    if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= NDTEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[c.y * w + c.x] > NDTEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;

    ndt_ind++;
    n = c + Coord(0, 1);
    if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= NDTEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[c.y * w + c.x] > NDTEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;

    ndt_ind++;
    n = c + Coord(-1, 0);
    if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= NDTEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[c.y * w + c.x] > NDTEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;

    ndt_ind++;
    n = c + Coord(0, -1);
    if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= NDTEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[c.y * w + c.x] > NDTEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;


    if (NDTSIZE > 4) {

        //diagonals:
        ndt_ind++;
        n = c + Coord(1, 1);
        if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= NDTEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[c.y * w + c.x] > NDTEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;

        ndt_ind++;
        n = c + Coord(1, -1);
        if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= NDTEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[c.y * w + c.x] > NDTEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;

        ndt_ind++;
        n = c + Coord(-1, 1);
        if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= NDTEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[c.y * w + c.x] > NDTEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;

        ndt_ind++;
        n = c + Coord(-1, -1);
        if (abs(im[n.y * w + n.x] - im[c.y * w + c.x]) <= NDTEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[c.y * w + c.x] > NDTEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;
    }
}

double ZDFThread::cmp_ndt(int *ndt_l, int *ndt_r) {

    int s = 0;

    for (int count = 0; count < NDTSIZE; count++) {
        if (ndt_l[count] == ndt_r[count]) {
            s++;
        }
    }
    return ((double) s) / ((double) NDTSIZE);

}


void ZDFThread::get_rank(Coord c, unsigned char *im, int w, int *list) {
    Coord n;
    int i = 0;

    for (int x = -RANKX; x <= RANKX; x++) {
        for (int y = -RANKY; y <= RANKY; y++) {

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

    for (int j = 0; j < RANKSIZE; j++) {
        for (int k = j + 1; k < RANKSIZE; k++) {
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
void ZDFThread::preprocessImageHSV(IplImage *srcImage, IplImage *destImage) {
    IplImage *hsvImage = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    IplImage *first_plane_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    IplImage *second_plane_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
    IplImage *third_plane_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);

    cvCvtColor(srcImage, hsvImage, CV_RGB2HSV);
    cvSplit(hsvImage, first_plane_ipl, second_plane_ipl, third_plane_ipl, NULL);
    cvCopy(third_plane_ipl, destImage, NULL);

    cvReleaseImage(&hsvImage);
    cvReleaseImage(&first_plane_ipl);
    cvReleaseImage(&second_plane_ipl);
    cvReleaseImage(&third_plane_ipl);


}

void ZDFThread::preprocessImageYUV(IplImage *srcImage, IplImage *destImage) {
    cvCvtColor(srcImage, yuva_orig_l_ipl, CV_RGB2YUV);
    cvSplit(yuva_orig_l_ipl, first_plane_l_ipl, second_plane_l_ipl, third_plane_l_ipl, NULL);
    cvCopy(first_plane_l_ipl, destImage, NULL);

}

void ZDFThread::preprocessImageGray(IplImage *srcImage, IplImage *destImage) {
    cvCvtColor(srcImage, destImage, CV_RGB2GRAY);
}

void ZDFThread::filterInputImage(IplImage *input, IplImage *dest) {
    int i;
    const int szInImg = input->imageSize;
    unsigned char *pFilteredInpImg = (unsigned char *) dest->imageData;
    unsigned char *pCurr = (unsigned char *) input->imageData;
    float lambda = .1f;
    const float ul = 1.0f - lambda;
    for (i = 0; i < szInImg; i++) { // assuming same size
        *pFilteredInpImg = (unsigned char) (lambda * *pCurr++ + ul * *pFilteredInpImg++ + .5f);

    }
}

void ZDFThread::matchTemplate(IplImage *templateImage, IplImage *matchImage, IplImage *dstImage) {

    /// Create the result matrix
    cv::Mat img_display = cv::cvarrToMat(matchImage);
    cv::Mat result;
    int result_cols = matchImage->width - templateImage->width + 1;
    int result_rows = matchImage->height - templateImage->height + 1;

    result.create(result_rows, result_cols, CV_32FC1);

    /// Do the Matching and Normalize
    cv::matchTemplate(cv::cvarrToMat(matchImage), cv::cvarrToMat(templateImage), result, CV_TM_SQDIFF_NORMED);
    normalize(result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

    /// Localizing the best match with minMaxLoc
    double minVal;
    double maxVal;
    cv::Point minLoc;
    cv::Point maxLoc;
    cv::Point matchLoc;

    cv::minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat());
    matchLoc = minLoc;

    cvSetImageROI(matchImage, cvRect(matchLoc.x, matchLoc.y, fov_l_ipl->width, fov_l_ipl->height));
    cvCopy(matchImage, dstImage, NULL);
    cvResetImageROI(matchImage);

}


#pragma clang diagnostic pop