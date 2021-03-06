#include <utility>


#include <yarp/cv/Cv.h>
#include <iCub/zdfThread.h>
#include <malloc.h>
#include <iCub/lbp.h>


using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace cv;

const int kern_sz = 5;

const Scalar blueLow(0,0,65);
const Scalar blueHigh(120,255,127);

const Scalar redLow(0,144,0);
const Scalar redHigh(255,255,173);

const Scalar yellowLow(0,85,156);
const Scalar yellowHigh(180,255,175);


const Scalar greenLow(0,0,110);
const Scalar greenHigh(255,117,189);

void ZDFThread::parametersInit() {

    const int x_topLeft = 128;
    const int y_topLeft = (params->src_height / 2) - (params->fovea_height / 2);
    fovea_rect = cv::Rect2d(x_topLeft, y_topLeft, params->fovea_width, params->fovea_height);
    foveaRecWithOffset = cv::Rect2d(x_topLeft, y_topLeft - params->offsetVertical, params->fovea_width,
                                    params->fovea_height);


    left_pyramid_DOG = cvCreateImage(cvSize(params->fovea_width, params->fovea_height), IPL_DEPTH_8U, 1);
    right_pyramid_DOG = cvCreateImage(cvSize(params->fovea_width, params->fovea_height), IPL_DEPTH_8U, 1);


    centerSurround = new CenterSurround(params->fovea_width, params->fovea_height, 8);


    if (params->rankOrNDT == 0) {
        koffsetx = params->rankX;
        koffsety = params->rankY;

        buffer1 = (int *) malloc(sizeof(int) * params->rankSize);
        buffer2 = (int *) malloc(sizeof(int) * params->rankSize);

    } else {
        koffsetx = params->ndtX;
        koffsety = params->ndtY;

        buffer1 = (int *) malloc(sizeof(int) * params->ndtSize);
        buffer2 = (int *) malloc(sizeof(int) * params->ndtSize);
    }

    fovea_size.width = params->fovea_width;
    fovea_size.height = params->fovea_height;

    multiClass = new MultiClass(fovea_size, left_pyramid_DOG->widthStep, 2, params);

    p_prob = (unsigned char **) malloc(sizeof(unsigned char *) * 2);

    sus = false;

    pBackSub = cv::createBackgroundSubtractorMOG2();

}

bool ZDFThread::threadInit() {
    /* initialize variables and create data-structures if needed */
    yDebug("Start of the Thread Initialization");
    //create all ports
    inputNameLeft = "/" + moduleName + "/imageLeft:i";
    imageInLeft.open(inputNameLeft);

    inputNameRight = "/" + moduleName + "/imageRight:i";
    imageInRight.open(inputNameRight);

    imageInMask.open("/" + moduleName + "/imageMask:i");


    outputNameProb = "/" + moduleName + "/imageProb:o";
    imageOutProb.open(outputNameProb);

    outputNameSeg = "/" + moduleName + "/imageSeg:o";
    imageOutSeg.open(outputNameSeg);

    outputNameDogLeft = "/" + moduleName + "/imageDogL:o";
    imageOutDogL.open(outputNameDogLeft);

    outputNameDogRight = "/" + moduleName + "/imageDogR:o";
    imageOutDogR.open(outputNameDogRight);

    outputNameTemp = "/" + moduleName + "/saliency:o";
    imageOutSaliency.open(outputNameTemp);

    outputNameTemp2 = "/" + moduleName + "/templateRGB:o";
    imageOutTemplateRGB.open(outputNameTemp2);

    outputNameGeometry = "/" + moduleName + "/ROIcoordinate:o";
    outputGeometry.open(outputNameGeometry);

    outputCOG.open("/" + moduleName + "/COG:o");
    parametersInit();

    yDebug("End of the Thread Initialization");
    return true;
}


void ZDFThread::setName(std::string module) {
    this->moduleName = std::move(module);
}


void ZDFThread::run() {
    while (!isStopping() ) { // the thread continues to run until isStopping() returns true

        ImageOf<PixelRgb> *img_in_left = imageInLeft.read(true);
        ImageOf<PixelRgb> *img_in_right = imageInRight.read(true);
        ImageOf<PixelRgb > *img_in_mask = imageInMask.read(params->stereoDisparity);


        if (img_in_left != nullptr && img_in_right != nullptr  && !sus) {

            cv::Mat left_mat_img = yarp::cv::toCvMat(*img_in_left);
            cv::Mat right_mat_img = yarp::cv::toCvMat(*img_in_right);



            // define the region of interest to mimick fovea
            cv::Mat left_img_roi = left_mat_img(fovea_rect);
            cv::Mat right_img_roi = right_mat_img(foveaRecWithOffset);





            // Preprocess images
            cv::cvtColor(right_img_roi, right_fovea, cv::COLOR_RGB2GRAY);
            cv::cvtColor(left_img_roi, left_fovea, cv::COLOR_RGB2GRAY);


            cv::cvtColor(right_img_roi, YUV_right, cv::COLOR_RGB2HSV);
            cv::cvtColor(left_img_roi, YUV_left, cv::COLOR_RGB2HSV);

            vector<cv::Mat> yuv_planes(3);
            cv::split(YUV_right, yuv_planes);
            YUV_right = yuv_planes[0];


            cv::split(YUV_left, yuv_planes);
            YUV_left = yuv_planes[0];





            getPyramidGaussian(right_fovea, DOG_right, 3);
            getPyramidGaussian(left_fovea, DOG_left, 3);

            cannyBlobDetection(DOG_left, DOG_left);
            cannyBlobDetection(DOG_right, DOG_right);



            processDisparityMap(DOG_right.data, DOG_left.data);


            p_prob[0] = prob_mat.data;
            p_prob[1] = zd_prob_mat.data;
            multiClass->proc(left_fovea.data, p_prob);



            cv::Mat mat_class = cv::cvarrToMat(multiClass->get_class_ipl());



            const unsigned char *out = mat_class.data;



            double minVal, maxVal;
            minMaxLoc(mat_class, &minVal, &maxVal); //find minimum and maximum intensities

            auto classSeg = static_cast<const int>(minVal);


            seg_image = cv::Mat::zeros(params->fovea_width, params->fovea_height, CV_8UC1);

            unsigned char *seg_im = seg_image.data;

            cv::Mat *segmented_img_dog = new cv::Mat(fovea_size.height, fovea_size.width, CV_8UC1, cv::Scalar(0));
            segmentedRGB = left_mat_img(fovea_rect).clone();

            const int psb_m = left_fovea.step;

            for (int j = 0; j < fovea_size.height; j++) {
                for (int i = 0; i < fovea_size.width; i++) {
                    if (out[i + j * psb_m] != classSeg) {

                        // Gray matching
                        seg_im[j * psb_m + i] = left_fovea.data[j * psb_m + i];
                        // Set the segmentation match in white
                        segmented_img_dog->data[j * psb_m + i] = 255;

                    } else {

                        segmentedRGB.at<cv::Vec3b>(j, i)[0] = 0;
                        segmentedRGB.at<cv::Vec3b>(j, i)[1] = 0;
                        segmentedRGB.at<cv::Vec3b>(j, i)[2] = 0;

                    }
                }
            }





            const int mid_x = 128;
            const int mid_y = (left_mat_img.rows - fovea_size.height) / 2;

            int topBis, bottomBis, leftBis, rightBis, numberPixelSegmented;

            medianfilter(seg_im,seg_im, 8);


            if(img_in_mask != nullptr){
                seg_mask = cv::Mat::zeros(params->fovea_width, params->fovea_height, CV_8UC1);
                cv::Mat mask = cv::Mat::zeros(params->fovea_width, params->fovea_height, CV_8UC1);;
                cv::Mat mask_mat_img = yarp::cv::toCvMat(*img_in_mask);
                mask_mat_img = mask_mat_img(fovea_rect);
                cvtColor(mask_mat_img, mask_mat_img, COLOR_RGB2GRAY);
                cv::threshold(mask_mat_img, mask,  0, 255, CV_THRESH_BINARY);


                seg_image.copyTo(seg_mask, mask);
//                bitwise_or(seg_image,mask, seg_mask);
                getRoundingBoxSegmented(&topBis, &bottomBis, &leftBis, &rightBis, &seg_mask);
                medianfilter(seg_mask.data,seg_mask.data, 3);
                numberPixelSegmented = cv::countNonZero(seg_mask);



            }

            else{
                getRoundingBoxSegmented(&topBis, &bottomBis, &leftBis, &rightBis, &seg_image);
                numberPixelSegmented = cv::countNonZero(seg_image);

            }





            const int yOneSeg = mid_y + topBis;
            const int yTwoSeg = mid_y + bottomBis;
            const int xOneSeg = mid_x + leftBis;
            const int xTwoSeg = mid_x + rightBis;

            const cv::Point pt1(xOneSeg , yOneSeg );
            const cv::Point pt2(xTwoSeg , yTwoSeg);


            cv::Rect rRect(pt1, pt2);


            if(numberPixelSegmented < params->max_area && numberPixelSegmented > params->min_area) {


                if(rRect.width > 0 ){

                    left_mat_img(rRect).copyTo(segTemplate);
                }

                if (outputGeometry.getOutputCount()) {
                    Bottle &geometry = outputGeometry.prepare();
                    geometry.clear();
                    geometry.addDouble(rRect.tl().x);
                    geometry.addDouble(rRect.tl().y);
                    geometry.addDouble(rRect.br().x);
                    geometry.addDouble(rRect.br().y);
                    outputGeometry.write();


                }

                if (outputCOG.getOutputCount()) {
                    Bottle &cog = outputCOG.prepare();
                    cog.clear();
                    const int centerX = rRect.tl().x + rRect.width / 2;
                    const int centerY = rRect.tl().y + rRect.height / 2;

                    cog.addInt(centerX);
                    cog.addInt(centerY);
                    outputCOG.write();

                }

                if (imageOutTemplateRGB.getOutputCount() > 0 && !segTemplate.empty()) {

                    yarp::sig::ImageOf<yarp::sig::PixelRgb> &outputTemplateRGB = imageOutTemplateRGB.prepare();
                    outputTemplateRGB = yarp::cv::fromCvMat<yarp::sig::PixelRgb>(segTemplate);

                    imageOutTemplateRGB.write();

                }


            }

        }

        //send it all when connections are established
        if (imageOutProb.getOutputCount() > 0) {

            yarp::sig::ImageOf<yarp::sig::PixelMono> &probMonoImage = imageOutProb.prepare();
            probMonoImage = yarp::cv::fromCvMat<yarp::sig::PixelMono>(prob_mat);
            imageOutProb.write();

        }


        if (imageOutSeg.getOutputCount() > 0) {

            yarp::sig::ImageOf<yarp::sig::PixelMono> &processingMonoImage = imageOutSeg.prepare();
            processingMonoImage = yarp::cv::fromCvMat<yarp::sig::PixelMono>(seg_mask.empty()?seg_image:seg_mask);
            imageOutSeg.write();

        }
        if (imageOutDogL.getOutputCount() > 0) {

            yarp::sig::ImageOf<yarp::sig::PixelMono> &leftDOGImage = imageOutDogL.prepare();
            leftDOGImage = yarp::cv::fromCvMat<yarp::sig::PixelMono>(DOG_left);
            imageOutDogL.write();

        }

        if (imageOutDogR.getOutputCount() > 0) {

            yarp::sig::ImageOf<yarp::sig::PixelMono> &rightDOGImage = imageOutDogR.prepare();
            rightDOGImage = yarp::cv::fromCvMat<yarp::sig::PixelMono>(DOG_right);

            imageOutDogR.write();

        }



    }
}


cv::Mat ZDFThread::getDOG(const cv::Mat &input_mat) {



    cv::Mat input_32f_mat, tmp_mat, tmp2_mat, cannyMat;
    input_mat.convertTo(input_32f_mat, CV_32F);

    cv::GaussianBlur(input_32f_mat, tmp_mat, cv::Size(kern_sz, kern_sz), params->sigma1, params->sigma1);
    cv::GaussianBlur(input_32f_mat, tmp2_mat, cv::Size(kern_sz, kern_sz), params->sigma2, params->sigma2);


    cv::Mat dog_mat = cv::abs(tmp_mat - tmp2_mat);


    double dog_mat_min, dog_mat_max;
    minMaxLoc(dog_mat, &dog_mat_min, &dog_mat_max); //find minimum and maximum intensities
    dog_mat.convertTo(dog_mat, CV_8U, 255.0 / (dog_mat_max - dog_mat_min),
                      -dog_mat_min * 255.0 / (dog_mat_max - dog_mat_min));





    return dog_mat;
}


void ZDFThread::onStop() {
    yInfo("closing ports..");
    imageInLeft.interrupt();
    imageInRight.interrupt();
    imageOutProb.interrupt();
    imageOutSeg.interrupt();
    imageOutDogL.interrupt();
    imageOutSaliency.interrupt();

    imageInLeft.close();
    imageInRight.close();
    imageOutProb.close();
    imageOutSeg.close();
    imageOutDogL.close();
    imageOutSaliency.close();


}

ZDFThread::~ZDFThread() {
    free(left_pyramid_DOG);
    free(right_pyramid_DOG);
    free(centerSurround);
}

ZDFThread::ZDFThread(MultiClass::Parameters *parameters) {
    params = parameters;

}

void ZDFThread::processDisparityMap(const unsigned char *img_left_DOG, const unsigned char *img_right_DOG) {
//*****************************************************************

    prob_mat = cv::Mat(((int) fovea_rect.height), ((int) fovea_rect.height), CV_8UC1, cv::Scalar(0));
    zd_prob_mat = cv::Mat(((int) fovea_rect.height), ((int) fovea_rect.height), CV_8UC1, cv::Scalar(0));



    //perform RANK or NDT kernel comparison:
    auto *o_prob_8u = prob_mat.data;
    auto *zd_prob_8u = zd_prob_mat.data;


    double cmp_res;
    const double rmax = sqrt((fovea_rect.width / 2.0) * (fovea_rect.width / 2.0)
                             + (fovea_rect.height / 2.0) * (fovea_rect.height / 2.0));


    for (int j = koffsety; j < fovea_rect.height - koffsety; j++) {
        c.y = j;
        for (int i = koffsetx; i < fovea_rect.width - koffsetx; i++) {
            c.x = i;
            int index = i + j * fovea_rect.width;

            //if either l or r textured at this retinal location:

            if ((img_left_DOG[index] > params->bland_dog_thresh && img_right_DOG[index] > params->bland_dog_thresh
            )) {

                if (params->rankOrNDT == 0) {
                    //use RANK:
                    get_rank(c, right_fovea.data, right_fovea.step,
                             buffer1);   //yDebug("got RANK from left\n");
                    get_rank(c, left_fovea.data, left_fovea.step,
                             buffer2);   //yDebug("got RANK from right\n");
                    cmp_res = cmp_rank(buffer1, buffer2);
                    //yDebug("compared RANKS \n");
                } else {
                    //use NDT:
                    get_ndt(c, YUV_right.data, YUV_right.step, buffer1);
                    get_ndt(c, YUV_left.data, YUV_left.step, buffer2);
                    cmp_res = cmp_ndt(buffer1, buffer2);
                }
                zd_prob_8u[index] = (unsigned char) (cmp_res * 255.0);
            } else {
                //untextured in both l & r, so set to bland prob (ZD):
                zd_prob_8u[index] = (unsigned char) (params->bland_prob * 255.0);//bland_prob
            }


            //RADIAL PENALTY:
            //The further from the origin, less likely it's ZD, so reduce zd_prob radially:
            //current radius:

            const double r = sqrt((c.x - fovea_rect.width / 2.0) * (c.x - fovea_rect.width / 2.0) +
                                  (c.y - fovea_rect.height / 2.0) * (c.y - fovea_rect.height / 2.0));

            double rad_pen = (int) ((r / rmax) * params->radial_penalty);
            const int max_rad_pen = zd_prob_8u[index];

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


void ZDFThread::get_ndt(Coord coord, unsigned char *im, int w, int *list) {

    Coord n;

    int ndt_ind = 0;
    n = coord + Coord(1, 0);

    if (abs(im[n.y * w + n.x] - im[coord.y * w + coord.x]) <= params->ndtEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[coord.y * w + coord.x] > params->ndtEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;

    ndt_ind++;
    n = coord + Coord(0, 1);
    if (abs(im[n.y * w + n.x] - im[coord.y * w + coord.x]) <= params->ndtEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[coord.y * w + coord.x] > params->ndtEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;

    ndt_ind++;
    n = coord + Coord(-1, 0);
    if (abs(im[n.y * w + n.x] - im[coord.y * w + coord.x]) <= params->ndtEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[coord.y * w + coord.x] > params->ndtEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;

    ndt_ind++;
    n = coord + Coord(0, -1);
    if (abs(im[n.y * w + n.x] - im[coord.y * w + coord.x]) <= params->ndtEQ)
        list[ndt_ind] = 0;
    else if (im[n.y * w + n.x] - im[coord.y * w + coord.x] > params->ndtEQ)
        list[ndt_ind] = 1;
    else
        list[ndt_ind] = -1;


    if (params->ndtSize > 4) {

        //diagonals:
        ndt_ind++;
        n = coord + Coord(1, 1);
        if (abs(im[n.y * w + n.x] - im[coord.y * w + coord.x]) <= params->ndtEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[coord.y * w + coord.x] > params->ndtEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;

        ndt_ind++;
        n = coord + Coord(1, -1);
        if (abs(im[n.y * w + n.x] - im[coord.y * w + coord.x]) <= params->ndtEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[coord.y * w + coord.x] > params->ndtEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;

        ndt_ind++;
        n = coord + Coord(-1, 1);
        if (abs(im[n.y * w + n.x] - im[coord.y * w + coord.x]) <= params->ndtEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[coord.y * w + coord.x] > params->ndtEQ)
            list[ndt_ind] = 1;
        else
            list[ndt_ind] = -1;

        ndt_ind++;
        n = coord + Coord(-1, -1);
        if (abs(im[n.y * w + n.x] - im[coord.y * w + coord.x]) <= params->ndtEQ)
            list[ndt_ind] = 0;
        else if (im[n.y * w + n.x] - im[coord.y * w + coord.x] > params->ndtEQ)
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

void ZDFThread::getPyramidGaussian(Mat &src, Mat &dst, int nbLevels) {
    Mat low_reso, tmp;
    Mat pyramid_edges[nbLevels];

    tmp = src;

    for(int i=0; i< nbLevels; i++) {
        pyrDown(tmp, low_reso);
        tmp = low_reso.clone();

        low_reso = getDOG(low_reso);
        pyramid_edges[i] = low_reso;
    }

    low_reso = pyramid_edges[nbLevels-1];

    for(int i= nbLevels - 1; i > 0; i--) {
        pyrUp(low_reso, tmp);
        low_reso =  pyramid_edges[i-1]  - tmp  ;
    }

    pyrUp(low_reso, dst);


}

void ZDFThread::get_rank(Coord coord, unsigned char *im, int w, int *list) {
    Coord n;
    int i = 0;

    for (int x = -params->rankX; x <= params->rankX; x++) {
        for (int y = -params->rankY; y <= params->rankY; y++) {

            n = coord + Coord(x, y);
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


/**
 * From the segmented image calcul the best rounding box by finding top, bottom, left, right point
 * @param top
 * @param bottom
 * @param left
 * @param right
 * @param segmentedImage
 */
void ZDFThread::getRoundingBoxSegmented(int *top, int *bottom, int *left, int *right, cv::Mat *segmentedImage) {
    const int width = segmentedImage->cols;
    const int height = segmentedImage->rows;
    const unsigned char *ptr_segmentedImage = segmentedImage->data;

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

void ZDFThread::cannyBlobDetection(cv::Mat &input, cv::Mat &mat_DOG) {




    const int dilation_size = 2;
    const cv::Mat element = getStructuringElement(cv::MORPH_ELLIPSE,
                                                  cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
                                                  cv::Point(dilation_size, dilation_size));


    cv::dilate(input, input, element);
//    cv::threshold(input, input,  params->bland_dog_thresh, 255, CV_THRESH_BINARY);
//
//    std::vector<std::vector<cv::Point> > cnt;
//    std::vector<cv::Vec4i> hrch;
//
//
//    findContours(input, cnt, hrch, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//    cv::fillPoly(mat_DOG, cnt, 255);






}

bool ZDFThread::isSus() const {
    return sus;
}

void ZDFThread::setSus(bool sus) {
    ZDFThread::sus = sus;
}

void ZDFThread::threshold_mask(cv::Mat &input, cv::Mat &segImg) {

    cv::Mat lab, maskR, maskB, maskG, maskY, result, copy;


    cv::cvtColor(input, lab , cv::COLOR_BGR2Lab);
    blur(lab,lab, Size(3,3));

    inRange(lab, blueLow, blueHigh, maskB);
    inRange(lab, redLow, redHigh, maskR);
    inRange(lab, yellowLow, yellowHigh, maskY);
    inRange(lab, greenLow, greenHigh, maskG);

    result = maskB + maskR + maskG + maskY;


    imshow("test", result);
    waitKey(1);
    segImg.copyTo(copy, result);

    segImg = copy.clone();



}



