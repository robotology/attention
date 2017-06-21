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


void showIplImage(const IplImage * image){
    cv::imshow("",cv::cvarrToMat(image));
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
    parameters.iter_max = rf.findGroup("PARAMS").check("max_iteration", Value(1), "what did the user select?").asInt();
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
    parameters.bland_dog_thresh = rf.findGroup("PARAMS").check("bland_dog_thresh", Value(1),
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
                    zdfThread->params->bland_prob = w;
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K8: {
                    int w = command.get(2).asInt();
                    zdfThread->params->max_spread = w;
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
                    double w = zdfThread->params->bland_prob;
                    reply.addDouble(w);
                    //ok = true;
                    break;
                }

                case COMMAND_VOCAB_K8: {
                    double w = zdfThread->params->max_spread;
                    reply.addDouble(w);
                    break;
                    //ok = true;
                }

                default: {
                    break;
                }
            }
            break;
        }
        default:break;
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
    temp_l_ipl = NULL;
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

    //cv::imshow("Matrix", NULL);
    //cv::waitKey(0);


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
    cvReleaseImage(&temp_l_ipl);     //ippiFree(temp_l);
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

    outputNameDog = "/" + moduleName + "/imageDog:o";
    imageOutDog.open(outputNameDog.c_str());

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
        //yDebug("Cycle before first if \n");

        if (img_in_left != NULL && img_in_right != NULL) {

            yDebug("inside the first if \n");
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

                //HACK: The variable srcsize is not initialized till the allocate() method, the istruction is moved few lines below. #aaroyo 04/02/2016
                //IplImage *mask  = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1); 


                // TODO Assuming it's right #amaroyo 04/01/2016
                if (!allocated || img_in_left->width() != insize.width || img_in_left->height() != insize.height) {
                    deallocate();
                    allocate(img_in_left);
                }

                IplImage *mask = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 1);
                yDebug("image size %d %d", srcsize.width, srcsize.height);

                //processing for zdf
                if (scale == 1.0) {
                    //resize the images if needed
                    //copy yarp image to OPENCV
                    int x = 0, y = 0;
                    yDebug("copying the input image left............\n");

                    IplImage *inputdepth = (IplImage *) img_in_left->getIplImage();

                    yDebug("copying yarp image %d %d %d %d \n", img_in_left->width(), img_in_left->height(),
                           inputdepth->depth, inputdepth->nChannels);
                    yDebug("into IplImage %d %d %d %d \n", copyImg_ipl->width, copyImg_ipl->height, copyImg_ipl->depth,
                           copyImg_ipl->nChannels);

                    cvSetImageROI((IplImage *) img_in_left->getIplImage(), cvRect(x, y, srcsize.width, srcsize.height));
                    cvCopy((IplImage *) img_in_left->getIplImage(), copyImg_ipl, mask);
                    copyImg = (unsigned char*) copyImg_ipl->imageData;
                    yDebug("success! \n");


                    //TODO This is commented in initial file #amaroyo 08/01/2016
                    //ippiCopy_8u_C3R( img_in_left->getRawImage(),  img_in_left->getRowSize(), copyImg, psbCopy, srcsize);                 
                    //ippiCopy_8u_C3R( img_in_right->getRawImage(), img_in_right->getRowSize(), r_orig, psb, srcsize);


                    //test with YUV images instead of grayscale left
                    //ippiCopy_8u_C3AC4R( img_in_left->getRawImage(), img_in_left->getRowSize(), l_orig, psb4, srcsize );
                    cvCopy((IplImage *) img_in_left->getIplImage(), l_orig_ipl, NULL);
                    yDebug("Success copying left image");

                    //and right
                    //ippiCopy_8u_C3AC4R( img_in_right->getRawImage(), img_in_right->getRowSize(), r_orig, psb4, srcsize );
                    cvCopy((IplImage *) img_in_right->getIplImage(), r_orig_ipl, NULL);
                    yDebug("Success copying right image");
                } else {
                    yInfo("scale is not 1");
                    //scale to width,height:
                    //ippiResizeGetBufSize(inroi, inroi, 3, IPPI_INTER_CUBIC, &BufferSize);
                    //Ipp8u* pBuffer=ippsMalloc_8u(BufferSize);
                    //ippiResizeSqrPixel_8u_C3R( img_in_left->getRawImage(), insize, psb, inroi, l_orig, psb, inroi, scale, scale, 0, 0, IPPI_INTER_CUBIC, pBuffer);
                    //ippiResizeSqrPixel_8u_C3R( img_in_right->getRawImage(), insize, psb, inroi, r_orig, psb, inroi, scale, scale, 0, 0, IPPI_INTER_CUBIC, pBuffer);
                    //ippsFree(pBuffer);
                    ////the following is deprecated...use previous
                    ////ippiResize_8u_C3R( img_in_left->getRawImage(), insize, img_in_left->width() * 3, inroi, l_orig, psb, srcsize, scale, scale, IPPI_INTER_CUBIC);
                    ////ippiResize_8u_C3R( img_in_right->getRawImage(), insize, img_in_right->width() * 3, inroi, r_orig, psb, srcsize, scale, scale, IPPI_INTER_CUBIC);
                }


                yDebug("extracting YUV planes for the left \n");
                //extract yuv plane
                //ippiRGBToYUV_8u_AC4R( l_orig, psb4, yuva_orig_l, psb4, srcsize );
                //ippiCopy_8u_C4P4R( yuva_orig_l, psb4, pyuva_l, psb, srcsize );
                cvCvtColor(l_orig_ipl, yuva_orig_l_ipl, CV_RGB2YUV);
                cvSplit(yuva_orig_l_ipl, first_plane_l_ipl, second_plane_l_ipl, third_plane_l_ipl, NULL);
                first_plane_l = (unsigned char *) first_plane_l_ipl->imageData;
                second_plane_l = (unsigned char *) second_plane_l_ipl->imageData;
                third_plane_l = (unsigned char *) third_plane_l_ipl->imageData;
                pyuva_l[0] = first_plane_l;  //Y
                pyuva_l[1] = second_plane_l; //U
                pyuva_l[2] = third_plane_l;  //V
                pyuva_l[3] = tmp;

                yDebug("..............success \n");


                yDebug("extracting YUV planes for the right");
                //ippiRGBToYUV_8u_AC4R( r_orig, psb4, yuva_orig_r, psb4, srcsize );
                //ippiCopy_8u_C4P4R( yuva_orig_r, psb4, pyuva_r, psb, srcsize );
                cvCvtColor(r_orig_ipl, yuva_orig_r_ipl, CV_RGB2YUV);
                cvSplit(yuva_orig_r_ipl, first_plane_r_ipl, second_plane_r_ipl, third_plane_r_ipl, NULL);
                first_plane_r = (unsigned char *) first_plane_r_ipl->imageData;
                second_plane_r = (unsigned char *) second_plane_r_ipl->imageData;
                third_plane_r = (unsigned char *) third_plane_r_ipl->imageData;
                pyuva_r[0] = first_plane_r;  //Y
                pyuva_r[1] = second_plane_r; //U
                pyuva_r[2] = third_plane_r;  //V
                pyuva_r[3] = tmp;

                yDebug("..............success \n");


                yDebug("copying intensity channels for the left and the right...........");
                //ippiCopy_8u_C1R( first_plane_l, f_psb,  rec_im_ly, psb_in, srcsize);
                //ippiCopy_8u_C1R( first_plane_r, f_psb,  rec_im_ry, psb_in, srcsize);
                cvCopy(first_plane_l_ipl, rec_im_ly_ipl, NULL);
                cvCopy(first_plane_r_ipl, rec_im_ry_ipl, NULL);
                yDebug("success \n");

                if (acquire) {
                    yDebug("acquiring \n");
                    IplImage *tsizeMask = cvCreateImage(cvSize(tsize.width, tsize.height), IPL_DEPTH_8U, 1);
                    //ippiCopy_8u_C1R(&rec_im_ly [(( srcsize.height - tsize.height ) / 2 ) * psb_in + ( srcsize.width - tsize.width ) /2 ], psb_in, temp_l, psb_t, tsize);
                    cvSetImageROI(rec_im_ly_ipl, cvRect((srcsize.width - tsize.width) / 2,
                                                        (srcsize.height - tsize.height) / 2,
                                                        tsize.width, tsize.height));
                    cvCopy(rec_im_ly_ipl, temp_l_ipl, NULL);
                    cvResetImageROI(rec_im_ly_ipl);

                    //ippiCopy_8u_C1R(&rec_im_ry [(( srcsize.height - tsize.height ) / 2 ) * psb_in + ( srcsize.width - tsize.width ) /2 ], psb_in, temp_r, psb_t, tsize);
                    cvSetImageROI(rec_im_ry_ipl, cvRect((srcsize.width - tsize.width) / 2,
                                                        (srcsize.height - tsize.height) / 2,
                                                        tsize.width, tsize.height));
                    cvCopy(rec_im_ry_ipl, temp_r_ipl, NULL);
                    cvResetImageROI(rec_im_ry_ipl);
                }

                //******************************************************************
                //Create left fovea and find left template in left image
                yDebug("creating left fovea and left template matching \n");

                //ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ly[(( srcsize.height - tisize.height )/2)*psb_in + ( srcsize.width - tisize.width )/2],
                //               psb_in, tisize,
                //               temp_l,
                //               psb_t, tsize,
                //               res_t, psb_rest);


                cv::Mat rec_im_ly_mat(cv::cvarrToMat(rec_im_ly_ipl));
                cv::Mat temp_l_mat(cv::cvarrToMat(temp_l_ipl));
                cv::Mat res_t_mat(cv::cvarrToMat(res_t_ipl));
                yDebug("result width %d == %d \n", rec_im_ly_mat.cols - temp_l_mat.cols + 1, res_t_mat.cols);
                yDebug("result height %d == %d \n", rec_im_ly_mat.rows - temp_l_mat.rows + 1, res_t_mat.rows);


                cvMatchTemplate(rec_im_ly_ipl, temp_l_ipl, res_t_ipl, CV_TM_CCORR_NORMED);
                //cv::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());


                //ippiMaxIndx_32f_C1R( res_t, psb_rest, trsize, &max_t, &sx, &sy);
                //ippiCopy_8u_C1R( &rec_im_ly [ ( mid_y + tl_y ) * psb_in + mid_x + tl_x], psb_in, fov_l, psb_m, msize ); //original

                double minVal_l;
                double maxVal_l;
                cv::Point minLoc_l, maxLoc_l, matchLoc_l;
                //this function gives back the locations and brightness values of the brightest and darkest spots of the image #amaroyo 09/02/2016
                cv::minMaxLoc(res_t_mat, &minVal_l, &maxVal_l, &minLoc_l, &maxLoc_l, cv::Mat());

                cvSetImageROI(rec_im_ly_ipl, cvRect(mid_x, mid_y, msize.width, msize.height));
                cvCopy(rec_im_ly_ipl, fov_l_ipl, NULL);
                cvResetImageROI(rec_im_ly_ipl);



                //******************************************************************
                //Create right fovea and find right template in right image:
                yDebug("creating right fovea and right template matching \n");

                //ippiCrossCorrValid_NormLevel_8u32f_C1R(&rec_im_ry[((srcsize.height-tisize.height)/2 + dpix_y )*psb_in + (srcsize.width-tisize.width)/2],
                //	        psb_in,tisize,
                //	        temp_r,
                //	        psb_t,tsize,
                //	        res_t, psb_rest);


                cvMatchTemplate(rec_im_ry_ipl, temp_r_ipl, res_t_ipl, CV_TM_CCORR_NORMED);

                //cv::normalize(result_mat, result_mat, 0, 1, cv::NORM_MINMAX, -1, cv::Mat());

                //ippiMaxIndx_32f_C1R(res_t,psb_rest,trsize,&max_t,&sx,&sy);
                //ippiCopy_8u_C1R(&rec_im_ry[(mid_y+tr_y+dpix_y)*psb_in + mid_x+tr_x],psb_in,fov_r,psb_m,msize); // original
                double minVal_r;
                double maxVal_r;
                cv::Point minLoc_r, maxLoc_r, matchLoc_r;
                cv::minMaxLoc(res_t_mat, &minVal_r, &maxVal_r, &minLoc_r, &maxLoc_r, cv::Mat());

                cvSetImageROI(rec_im_ry_ipl, cvRect(mid_x, mid_y, msize.width, msize.height));
                cvCopy(rec_im_ry_ipl, fov_r_ipl, NULL);
                cvResetImageROI(rec_im_ry_ipl);


                //*****************************************************************
                //Star diffence of gaussian on foveated images
                yDebug("difference of gaussian on foveated images \n");
                yDebug("Sizes of Foveas, Left Width=%d, Left Height=%d, Right Width=%d, RightHeight=%d \n",
                       fov_l_ipl->width, fov_l_ipl->height, fov_r_ipl->width, fov_r_ipl->height);
                dl->proc(fov_l_ipl, psb_m);
                // as output of the previous call we get out_dog_on,_off, _onoff

                dr->proc(fov_r_ipl, psb_m);


                //*****************************************************************
                //SPATIAL ZD probability map from fov_l and fov_r:
                yDebug("computing the spatial ZD probability map from fov_l and fov_r \n");
                //perform RANK or NDT kernel comparison:
                o_prob_8u = (unsigned char *) o_prob_8u_ipl->imageData;
                zd_prob_8u = (unsigned char *) zd_prob_8u_ipl->imageData;

                yDebug("Koffsety: %d, height: %d \n", koffsety, msize.height);
                yDebug("Printing psb_m size: %d \n", psb_m);

                for (int j = koffsety; j < msize.height - koffsety; j++) {

                    c.y = j;
                    for (int i = koffsetx; i < msize.width - koffsetx; i++) {

                        c.x = i;
                        //modification #amaroyo 19/02/16
                        //char* p_dogonoff = dl->get_dog_onoff(); --this is null

                        char *p_dogonoff_l = dl->get_dog_onoff_ipl()->imageData;
                        //yDebug("Sizes Left Dog width= %d, height= %d, step = %d \n", dl->get_dog_onoff_ipl()->width, dl->get_dog_onoff_ipl()->height, dl->get_dog_onoff_ipl()->widthStep);
                        char *p_dogonoff_r = dr->get_dog_onoff_ipl()->imageData;
                        //yDebug("Sizes Right Dog width= %d, height= %d, step = %d  \n", dr->get_dog_onoff_ipl()->width, dr->get_dog_onoff_ipl()->height, dr->get_dog_onoff_ipl()->widthStep);
                        //yDebug("values %d, %d, %d, %d \n", *p_dogonoff_l, *p_dogonoff_r, p_dogonoff_l[i + j*dl->get_psb()], params->data_penalty);


                        //if either l or r textured at this retinal location: 
                        //TODO check this parameters
                        if (p_dogonoff_l[i + j * dl->get_psb()] >= params->data_penalty ||
                            p_dogonoff_r[i + j * dr->get_psb()] >= params->bland_dog_thresh) {

                            if (RANK0_NDT1 == 0) {
                                //yDebug("using RANK \n");
                                //use RANK:
                                get_rank(c, (unsigned char *) fov_l_ipl->imageData, psb_m,
                                         rank1);   //yDebug("got RANK from left\n");
                                get_rank(c, (unsigned char *) fov_r_ipl->imageData, psb_m,
                                         rank2);   //yDebug("got RANK from right\n");
                                cmp_res = cmp_rank(rank1, rank2);
                                //yDebug("compared RANKS \n");
                            } else {
                                yDebug("using NDT \n");
                                //use NDT:
                                get_ndt(c, fov_l, psb_m, ndt1);
                                get_ndt(c, fov_r, psb_m, ndt2);
                                cmp_res = cmp_ndt(ndt1, ndt2);
                            }

                            //yDebug("preparing zerodisparity probability mono channel \n");
                            zd_prob_8u[j * psb_m + i] = (unsigned char) (cmp_res * 255.0);
                        } else {
                            //yDebug("untextured in both l & r \n");
                            //untextured in both l & r, so set to bland prob (ZD):
                            zd_prob_8u[j * psb_m + i] = (unsigned char) (params->bland_prob * 255.0);//bland_prob
                        }

                        //RADIAL PENALTY:
                        //The further from the origin, less likely it's ZD, so reduce zd_prob radially:
                        //yDebug("introducing RADIAL PENALITY \n");
                        //current radius:
                        r = sqrt((c.x - msize.width / 2.0) * (c.x - msize.width / 2.0) +
                                 (c.y - msize.height / 2.0) * (c.y - msize.height / 2.0));
                        //yDebug("Penalty: %d \n", params->radial_penalty);
                        rad_pen = (int) ((r / rmax) * params->radial_penalty);//radial_penalty
                        max_rad_pen = zd_prob_8u[j * psb_m + i];
                        if (max_rad_pen < rad_pen) {
                            rad_pen = max_rad_pen;
                        }
                        //apply radial penalty
                        //yDebug("applying radial penality...... ");
                        zd_prob_8u[j * psb_m + i] -= rad_pen;
                        //yDebug("success \n");

                        //manufacture NZD prob (other):
                        o_prob_8u[psb_m * j + i] = (unsigned char) 255 - zd_prob_8u[psb_m * j + i];

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
                //cvCopy(m_class_ipl, out_ipl, NULL);
                unsigned char *pm_get_class = multiClass->get_class();
                //TODO uncomment this?  #amaroyo 04/01/2016
                //ippiCopy_8u_C1R( m->get_class(), m->get_psb(), out, psb_m, msize);



                //goto streaming;


                //evaluate result:
                yDebug("evaluating result.... \n");
                unsigned char *out = (unsigned char *) m_class_ipl->imageData;
                getAreaCoGSpread(out, psb_m, msize, &area, &cog_x, &cog_y, &spread);
                yDebug("area: %d, cog_x: %f, cog_y: %f, spread: %f", area, cog_x, cog_y, spread);
                yDebug("success \n");

                cog_x_send = cog_x;
                cog_y_send = cog_y;

                seg_im = (unsigned char *) seg_im_ipl->imageData;
                seg_dog = (unsigned char *) seg_dog_ipl->imageData;

                //debugging #amaroyo 24/02/2016
                int cont = 0;
                int eqZero = 0;

                //we have mask and image  (out)   [0/255]
                //construct masked image  (fov_l) [0..255]
                for (int j = 0; j < msize.height; j++) {
                    for (int i = 0; i < msize.width; i++) {
                        cont++;
                        if (out[i + j * psb_m] == 0) {
                            eqZero++;
                            seg_im[j * psb_m + i] = 0;
                            seg_dog[j * psb_m + i] = 0;
                        } else {
                            seg_dog[j * psb_m + i] = dr->get_dog_onoff()[j * psb_m + i];
                            seg_im[j * psb_m + i] = fov_r[j * psb_m + i];
                        }
                    }
                }


                yDebug("Total count: %d, Eq to Zero: %d", cont, eqZero);


                //goto streaming;
                //********************************************************************
                //If nice segmentation:
                yDebug("checking for nice segmentation \n");
                if (area >= params->min_area && area <= params->max_area && spread <= params->max_spread) {
                    //don't update templates to image centre any more as we have a nice target
                    acquire = false;
                    //update templates towards segmentation CoG:
                    yDebug("area:%d spread:%f cogx:%f cogy:%f - UPDATING TEMPLATE\n", area, spread, cog_x, cog_y);
                    //Bring cog of target towards centre of fovea://SNAP GAZE TO OBJECT:
                    cog_x *= params->cog_snap;
                    cog_y *= params->cog_snap;


                    //TODO uncomment this??  #amaroyo 04/01/2016
                    //floor(val + 0.5) instead of round

                    int floorFov_l =
                            (mid_x_m + ((int) floor(cog_x + 0.5))) + (mid_y_m + ((int) floor(cog_y + 0.5))) * psb_m;
                    int floorFov_r =
                            (mid_x_m + ((int) floor(cog_x + 0.5))) + (mid_y_m + ((int) floor(cog_y + 0.5))) * psb_m;

                    temp_l = &fov_l[floorFov_l];
                    temp_r = &fov_r[floorFov_r];

                    //ippiCopy_8u_C1R(&fov_l[( mid_x_m + ( (int) floor ( cog_x + 0.5 ) ) ) + ( mid_y_m + ( ( int ) floor ( cog_y + 0.5) ) ) * psb_m], psb_m, temp_l, psb_t, tsize );
                    //ippiCopy_8u_C1R(&fov_r[( mid_x_m + ( (int) floor ( cog_x + 0.5 ) ) ) + ( mid_y_m + ( ( int ) floor ( cog_y + 0.5) ) ) * psb_m], psb_m, temp_r, psb_t, tsize );



                    //We've updated, so reset waiting:
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

                        //TODO Big block -- seems to be working, why commented?  #amaroyo 04/01/2016

                        for (int j = 0; j < msize.height * psb_m; j++) {
                            if ((int) seg_im[j] > 0) {
                                top = j / psb_m;
                                //cout << "top : " << top << endl;
                                break;
                            }
                        }
                        for (int j = msize.height * psb_m; j > 0; j--) {

                            if ((int) seg_im[j] > 0) {
                                bottom = j / psb_m;
                                //cout << "bottom : " << bottom << endl;
                                break;
                            }
                        }
                        bool Tmpout = false;
                        for (int i = 0; i < msize.width; i++) {
                            for (int j = 0; j < msize.height; j++) {
                                if ((int) seg_im[i + j * psb_m] > 0) {
                                    left = i;
                                    //cout << "left : " << left << endl;
                                    Tmpout = true;
                                    break;
                                }
                            }
                            if (Tmpout)break;
                        }
                        Tmpout = false;
                        for (int i = msize.width; i > 0; i--) {
                            for (int j = 0; j < msize.height; j++) {
                                if ((int) seg_im[i + j * psb_m] > 0) {
                                    right = i;
                                    //cout << "right : " << right << endl;
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

                        img_out_temp = new ImageOf<PixelBgr>;
                        img_out_temp->resize(tempSize.width, tempSize.height);
                        IplImage *maskTempSize = cvCreateImage(cvSize(tempSize.width, tempSize.height), 8, 1);

                        for (int j = top; j < bottom + 1; j++) {
                            for (int i = left; i < right + 1; i++) {
                                if ((int) seg_im[i + j * psb_m] > 0) {
                                    int x = srcsize.width / 2 - msize.width / 2 + i;
                                    int y = srcsize.height / 2 - msize.height / 2 + j;
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



                        //TODO check  #amaroyo 04/01/2016
                        //ippiCopy_8u_C3R( tempImg, psbtemp, img_out_temp->getRawImage(), img_out_temp->getRowSize() , tempSize);
                        cvCopy(tempImg_ipl, img_out_temp->getIplImage(), NULL);

                        img_out_temp = &imageOutTemp.prepare();
                        img_out_temp->resize(l_orig_ipl->width, l_orig_ipl->height);

                        cvCopy(l_orig_ipl, img_out_temp->getIplImage(), NULL);
                        yDebug("copied image success \n");

                        imageOutTemp.write();
                        yDebug("sent image \n");

                        //TODO uncommented  #amaroyo 04/01/2016

                        // HACK: added just to make the deallocation safer. #amaroyo 04/02/2016
                        img_out_temp = NULL;
                        //delete img_out_temp;


                        //ippiFree (tempImg);
                        //cvReleaseImage(&tempImg_ipl);
                        // HACK: added just to make the deallocation safer. #amaroyo 04/02/2016
                        //tempImg_ipl = NULL;
                    }

                    //*******************************************************************
                    //finished extracting
                    update = false;
                    //retreive only the segmented object in order to send as template
                }

                if (waiting >= 25) { //acquire_wait
                    yDebug("Acquiring new target until nice seg (waiting:%d >= acquire_wait:%d)\n", waiting,
                           params->acquire_wait);//acquire_wait
                    acquire = true;
                }

                yDebug("preparing the vector target for the COG \n");
                Vector &target = cogPort.prepare();
                target.resize(2);
                target[0] = cog_x_send;
                target[1] = cog_y_send;
                cogPort.write();


                // streaming :

                /*Adding mask to copy the results to output
                  #amaroyo 08/02/2016
                */
                IplImage *maskOutput = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);


                //send it all when connections are established
                if (imageOutProb.getOutputCount() > 0) {
                    yDebug("Inside imageProb\n");
                    //TODO this is correct , change and improve efficiency in future  #amaroyo 04/01/2016
                    //ippiCopy_8u_C1R( zd_prob_8u, psb_m, img_out_prob->getRawImage(), img_out_prob->getRowSize(), msize );
                    cvCopy(zd_prob_8u_ipl, img_out_prob->getIplImage(), NULL);
                    imageOutProb.prepare() = *img_out_prob;
                    imageOutProb.write();


                    /*
                    //HACK to test output #amaroyo 04/01/2016
                    yarp::sig::ImageOf<yarp::sig::PixelMono>* processingMonoImage;
                    processingMonoImage->wrapIplImage(dl->get_dog_on_ipl());
                    imageOutProb.prepare() = *processingMonoImage;
                    imageOutProb.write();
                    */


                    yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage;
                    processingMonoImage = &imageOutProb.prepare();
                    processingMonoImage->resize(msize.width, msize.height);
                    IplImage *aux = zd_prob_8u_ipl;
                    //print("ZDF IMG 1 %08X  \n", (unsigned int) aux);
                    //cv::imshow("Matrix", cv::cvarrToMat(aux));
                    //cv::waitKey(0);

                    processingMonoImage->wrapIplImage(aux); //fov_l_ipl
                    //print("ZDF PROCESSING 1 %08X  \n", (unsigned int) processingMonoImage);
                    imageOutProb.write();


                }

                if (imageOutSeg.getOutputCount() > 0) {
                    yDebug("Inside imageSeg\n");
                    //TODO missing copy #amaroyo 12/01/2016
                    //ippiCopy_8u_C1R( seg_im, psb_m, img_out_seg->getRawImage(), img_out_seg->getRowSize(), msize );
                    //imageOutSeg.prepare() = *img_out_seg;
                    //imageOutSeg.write();

                    /*
                    yarp::sig::ImageOf<yarp::sig::PixelMono>* processingMonoImage;
                    IplImage* img = dl->get_dog_off_ipl();
                    //print("ZDF ADDRESS 1 %08X  \n", img);
                    processingMonoImage->wrapIplImage(img);
                    imageOutSeg.prepare() = *processingMonoImage;
                    //print("ZDF PROCESSING 1 %08X  \n", processingMonoImage);
                    imageOutSeg.write();
                    */


                    yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage;
                    processingMonoImage = &imageOutSeg.prepare();
                    processingMonoImage->resize(msize.width, msize.height);
                    IplImage *aux = seg_im_ipl;
                    //print("ZDF IMG 2 %08X  \n", (unsigned int) aux);
                    //cv::imshow("Matrix", cv::cvarrToMat(aux));
                    //cv::waitKey(1);

                    processingMonoImage->wrapIplImage(aux); //temp_r_ipl
                    //print("ZDF PROCESSING 2 %08X  \n", (unsigned int) processingMonoImage);
                    imageOutSeg.write();



                }
                if (imageOutDog.getOutputCount() > 0) {
                    yDebug("Inside imageDog\n");
                    //TODO missing copy #amaroyo 12/01/2016
                    //ippiCopy_8u_C1R( seg_dog, psb_m, img_out_dog->getRawImage(), img_out_dog->getRowSize(), msize );
                    //imageOutDog.prepare() = *img_out_dog;
                    //imageOutDog.write();


                    /*
                    yarp::sig::ImageOf<yarp::sig::PixelMono>* processingMonoImage;
                    IplImage* img = dl->get_dog_onoff_ipl();
                    //print("ZDF ADDRESS 2 %08X  \n", img);
                    processingMonoImage->wrapIplImage(img);
                    imageOutDog.prepare() = *processingMonoImage;
                    //print("ZDF PROCESSING 2 %08X  \n", processingMonoImage);
                    imageOutDog.write();
                    */

                    yarp::sig::ImageOf<yarp::sig::PixelMono> *processingMonoImage;
                    processingMonoImage = &imageOutDog.prepare();
                    processingMonoImage->resize(msize.width, msize.height);
                    IplImage *aux = dr->get_dog_onoff_ipl();
                    //print("ZDF IMG 3 %08X  \n", (unsigned int) aux);
                    //cv::imshow("Matrix", cv::cvarrToMat(aux));
                    //cv::waitKey(0);

                    processingMonoImage->wrapIplImage(aux); //fov_r_ipl
                    //print("ZDF PROCESSING 3 %08X  \n", (unsigned int) processingMonoImage);
                    imageOutDog.write();

                }

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
    if (temp_l_ipl != NULL) cvReleaseImage(&temp_l_ipl);
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

    width = img->width();
    height = img->height();
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

    msize.width = 100; //should be taken from ini file // was 128
    msize.height = 100; //should be taken from ini file // was 128
    tsize.width = 32;  //should be taken from ini file
    tsize.height = 32;  //should be taken from ini file

    t_lock_lr = 32;     //should be taken from ini file
    t_lock_ud = 32;     //should be taken from ini file

    tisize.width = tsize.width + 2 * t_lock_lr;
    tisize.height = tsize.height + 2 * t_lock_ud;

    trsize.width = 289;//tisize.width  - tsize.width  + 1;
    trsize.height = 209;//tisize.height - tsize.height + 1;

    mid_x = (srcsize.width - msize.width) / 2;
    mid_y = (srcsize.height - msize.height) / 2;
    mid_x_m = (msize.width - tsize.width) / 2;
    mid_y_m = (msize.height - tsize.height) / 2;

    cog_x = 0.0;
    cog_y = 0.0;
    cog_x_send = 0.0;
    cog_y_send = 0.0;

    //TODO RANK and NDT have the same values #amaroyo 19/02/2016
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


    //out          = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    //seg_im       = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    //seg_dog      = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    //fov_l        = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    //fov_r        = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    //zd_prob_8u   = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    // o_prob_8u   = ippiMalloc_8u_C1(msize.width,msize.height, &psb_m);
    out_ipl = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);
    seg_im_ipl = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);
    seg_dog_ipl = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);
    fov_l_ipl = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);
    fov_r_ipl = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);
    zd_prob_8u_ipl = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);
    o_prob_8u_ipl = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);
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

    maskMsize = cvCreateImage(cvSize(msize.width, msize.height), IPL_DEPTH_8U, 1);

    pyuva_l = (unsigned char **) malloc(4 * sizeof(unsigned char *));
    pyuva_r = (unsigned char **) malloc(4 * sizeof(unsigned char *));
    pyuva_l_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);
    pyuva_r_ipl = cvCreateImage(cvSize(srcsize.width, srcsize.height), IPL_DEPTH_8U, 3);

    //ippiSet_8u_C1R( 0, zd_prob_8u, psb_m, msize );
    cvSet(zd_prob_8u_ipl, cv::Scalar(0, 0, 0));
    //ippiSet_8u_C1R( 0, o_prob_8u, psb_m, msize );
    cvSet(o_prob_8u_ipl, cv::Scalar(0, 0, 0));

    p_prob[0] = o_prob_8u;
    p_prob[1] = zd_prob_8u;

    //templates:
    //temp_l     = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);
    //temp_r     = ippiMalloc_8u_C1(tsize.width,tsize.height, &psb_t);
    temp_l_ipl = cvCreateImage(cvSize(tsize.width, tsize.height), IPL_DEPTH_8U, 1);
    temp_r_ipl = cvCreateImage(cvSize(tsize.width, tsize.height), IPL_DEPTH_8U, 1);
    psb_t = temp_l_ipl->widthStep;

    dl = new DoG(msize);
    dr = new DoG(msize);

    multiClass = new MultiClass(msize, psb_m, nclasses, params);

    tl_x = 0;
    tl_y = 0;
    tr_x = 0;
    tr_y = 0;
    waiting = 0;
    rmax = sqrt((msize.width / 2.0) * (msize.width / 2.0)
                + (msize.height / 2.0) * (msize.height / 2.0));

    update = false;
    acquire = true;

    img_out_prob = new ImageOf<PixelMono>;
    img_out_prob->resize(msize.width, msize.height);

    img_out_seg = new ImageOf<PixelMono>;
    img_out_seg->resize(msize.width, msize.height);

    img_out_dog = new ImageOf<PixelMono>;
    img_out_dog->resize(msize.width, msize.height);

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
ZDFThread::getAreaCoGSpread(unsigned char *im_, int psb_, defSize sz_, int *parea, double *pdx, double *pdy, double *spread) {

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

#pragma clang diagnostic pop