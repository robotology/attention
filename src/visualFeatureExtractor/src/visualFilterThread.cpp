// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2009 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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
 * @file visualFilterThread.cpp
 * @brief Implementation of the visual filter thread (see visualFilterThread.h).
 */

#include <iCub/RC_DIST_FB_logpolar_mapper.h>
#include <iCub/visualFilterThread.h>
#include <cstring>

#define ONE_BY_ROOT_TWO 0.707106781

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;
//using namespace cv;
//using namespace iCub::logpolar;




/**********************************************************/
// general purpose definitions

bool subsampleFovea(yarp::sig::ImageOf<yarp::sig::PixelMono>& dst, const yarp::sig::ImageOf<yarp::sig::PixelMono>& src) {
    //
    if (dst.width()!=dst.height() || dst.width()>src.width() || dst.height()>src.height()) {
        cerr << "subsampleFovea: can't perform the requested operation" << endl;
        return false;
    }

    const int fov = dst.width();
    const int offset = src.height()/2-fov/2;
    const int col = src.width()/2-fov/2;
    const int bytes = fov*sizeof(PixelMono);

    for (int i = 0; i < fov; i++) {
        unsigned char *s = (unsigned char *)src.getRow(i+offset)+col*sizeof(PixelMono);
        unsigned char *d = dst.getRow(i);
        memcpy(d, s, bytes);
    }
    return true;
}

//
bool replicateBorderLogpolar(yarp::sig::Image& dest, const yarp::sig::Image& src, int maxkernelsize) {
    //
    if (src.width()+2*maxkernelsize != dest.width() || src.height()+maxkernelsize != dest.height()) {
        std::cerr << "ReplicateBorderLogpolar: images aren't correctly sized for the operation" << std::endl;
        return false;
    }

    if (src.width()%2) {
        std::cerr << "ReplicateBorderLogpolar: image width must be an even number" << std::endl;
        return false;
    }

    // copy of the image
    const int pxsize = dest.getPixelSize();
    unsigned char *d = dest.getPixelAddress (maxkernelsize, maxkernelsize);
    unsigned char *s = src.getRawImage();
    const int bytes = src.width() * pxsize;

    for (int i = 0; i < src.height(); i++) {
        memcpy(d, s, bytes);
        d += dest.getRowSize();
        s += src.getRowSize();
    }

    // memcpy of the horizontal fovea lines (rows) 
    const int sizeBlock = src.width() / 2;
    for(int i = 0; i < maxkernelsize; i++) {
        memcpy(dest.getPixelAddress(sizeBlock+maxkernelsize, maxkernelsize-1-i),
                dest.getPixelAddress(maxkernelsize, maxkernelsize+i),
                sizeBlock * pxsize);
        memcpy(dest.getPixelAddress(maxkernelsize, maxkernelsize-1-i),
                dest.getPixelAddress(sizeBlock+maxkernelsize, maxkernelsize+i),
                sizeBlock * pxsize);
    }

    // copy of the block adjacent angular positions (columns)
    const int px = maxkernelsize * pxsize;
    const int width = dest.width();

    for (int row = 0; row < dest.height(); row++) {
        memcpy (dest.getPixelAddress(width-maxkernelsize, row),
                dest.getPixelAddress(maxkernelsize, row),
                px);
        memcpy (dest.getPixelAddress(0, row),
                dest.getPixelAddress(width-maxkernelsize-maxkernelsize, row),
                px);
    }

    return true;
}
// end of general purpose logPolar methods

// beginning of logPolar methods on logPolar object

// implementation of the ILogpolarAPI interface.
bool logpolarTransformVisual::allocLookupTables(int mode, int necc, int nang, int w, int h, double overlap) {
    //
    if (allocated()) {
        // check, return false in case size has changed. need to manually free and recompute maps.
        if (mode != mode_ || necc != necc_ || nang != nang_ || w != width_ || h != height_ || overlap != overlap_) {
            cerr << "logpolarTransform: new size differ from previously allocated maps" << endl;
            return false;
        }

        cerr << "logpolarTransform: tried a reallocation of already configured maps, no action taken" << endl;
        return true;
    }

    necc_ = necc;
    nang_ = nang;
    width_ = w;
    height_ = h;
    overlap_ = overlap;
    mode_ = mode;
    const double scaleFact = RCcomputeScaleFactor ();    
    
    if (c2lTable == 0 && (mode & C2L)) {
        c2lTable = new cart2LpPixel[necc*nang];
        if (c2lTable == 0) {
            cerr << "logpolarTransform: can't allocate c2l lookup tables, wrong size?" << endl;
            return false;
        }

        RCbuildC2LMap (scaleFact, ELLIPTICAL, PAD_BYTES(w*MONO_PIXEL_SIZE, YARP_IMAGE_ALIGN));
    }

    if (l2cTable == 0 && (mode & L2C)) {
        l2cTable = new lp2CartPixel[w*h];
        if (l2cTable == 0) {
            cerr << "logPolarLibrary: can't allocate l2c lookup tables, wrong size?" << endl;
            return false;
        }

        RCbuildL2CMap (scaleFact, 0, 0, ELLIPTICAL, PAD_BYTES(nang*MONO_PIXEL_SIZE, YARP_IMAGE_ALIGN));
    }
    return true;
}

bool logpolarTransformVisual::freeLookupTables() {
    if (c2lTable)
        RCdeAllocateC2LTable ();
    if (l2cTable)
        RCdeAllocateL2CTable ();
    return true;
}

// to do: must work for IplImage in general
bool logpolarTransformVisual::cartToLogpolar(yarp::sig::ImageOf<yarp::sig::PixelMono>& lp, 
                                       const yarp::sig::ImageOf<yarp::sig::PixelMono>& cart) {
    if (!(mode_ & C2L)) {
        cerr << "logPolarLibrary: conversion to logpolar called with wrong mode set" << endl;
        return false;
    }

    // LATER: assert whether lp & cart are effectively nang * necc as the c2lTable requires.
    RCgetLpImg (lp.getRawImage(), (unsigned char *)cart.getRawImage(), c2lTable, lp.getPadding());
    return true;
}

bool logpolarTransformVisual::logpolarToCart(yarp::sig::ImageOf<yarp::sig::PixelMono>& cart,
                            const yarp::sig::ImageOf<yarp::sig::PixelMono>& lp) {
    if (!(mode_ & L2C)) {
        cerr << "logPolarLibrary: conversion to cartesian called with wrong mode set" << endl;
        return false;
    }

    // LATER: assert whether lp & cart are effectively of the correct size.
    RCgetCartImg (cart.getRawImage(), lp.getRawImage(), l2cTable, cart.getPadding());

    return true;
}

// internal implementation of the logpolarTransform class.

inline double __max64f (double x, double y) {
    return (x > y) ? x : y;
}

void logpolarTransformVisual::RCdeAllocateC2LTable ()
{
    if (c2lTable) {
        delete[] c2lTable[0].position; // iweight is contiguous to position.
        delete[] c2lTable;
    }
    c2lTable = 0;
}

void logpolarTransformVisual::RCdeAllocateL2CTable ()
{
    if (l2cTable) {
        delete[] l2cTable[0].position;
        delete[] l2cTable;
    }
    l2cTable = 0;
}

double logpolarTransformVisual::RCgetLogIndex ()
{
    double logIndex;
    logIndex = (1.0 + sin (PI / nang_)) / (1.0 - sin (PI / nang_));
    return logIndex;
}

double logpolarTransformVisual::RCcomputeScaleFactor ()
{
    double maxRad;
    double receptFieldRadius;
    double r0;
    double lambda;
    int fov;
	int cSize = width_;
	
	if (width_ > height_)
		cSize = height_;

    double totalRadius;
    double angle = (2.0 * PI / nang_);   // Angular size of one pixel
    double sinus = sin (angle / 2.0);

    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    maxRad = pow (lambda, necc_ - 1) * (r0 + 0.5 / pow (lambda, fov));

    receptFieldRadius = maxRad * 2.0 * sinus * (overlap_ + 1.0) / (2.0);
    totalRadius = maxRad + receptFieldRadius;
    totalRadius = (cSize / 2) / totalRadius;
    return totalRadius;
}

int logpolarTransformVisual::RCbuildC2LMap (double scaleFact, int mode, int padding)
{
    // store map in c2lTable which is supposedly already allocated (while the internal arrays are allocated on the fly).

    if (overlap_ <= -1.0) {
        cerr << "logpolarTransform: overlap must be greater than -1" << endl;
        return 1;
    }

    int cSize = width_;

    if (width_ > height_)
        cSize = height_;

    const double precision = 10.0;
    double angle = (2.0 * PI / nang_);   // Angular size of one pixel
    double sinus = sin (angle / 2.0);
    double tangent = sinus / cos (angle / 2.0);

    int fov;
    int lim;

    double lambda;              // Log Index
    double firstRing;           // Diameter of the receptive fields in the first ring when overlap is 0
    double *currRad;            // Distance of the center of the current ring RF's from the center of the mapping
    double *nextRad;            // Distance of the center of the next ring's RF's from the center of the mapping
    double r0;                  // lower limit of RF0 in the "pure" log polar mapping

    double x0, y0;
    double locX, locY, locRad;
    double *radii;
    double *tangaxis;
    double *radialaxis;
    double *focus;
    double *radii2;
    double A;
    double L;
    double step;
    double F0x, F0y, F1x, F1y;
    double maxaxis;

    int rho, theta, j, sz;

    double *sintable;
    double *costable;
    int intx, inty;
    bool found;
    int mapsize;
    float *weight;

    // intiialization starts more or less here.
    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    firstRing = (1.0 / (lambda - 1)) - (int) (1.0 / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    // main table pointer (temporary).
    cart2LpPixel *table = c2lTable;

    // temporary.
    tangaxis = new double[necc_];
    radialaxis = new double[necc_];
    focus = new double[necc_];
    radii2 = new double[necc_];
    currRad = new double[necc_];
    nextRad = new double[necc_];

    if ((tangaxis == 0) || (radialaxis == 0) || (focus == 0) || (radii2 == 0) || (currRad == 0) || (nextRad == 0))
        goto C2LAllocError;

    /************************
     * RF's size Computation *
     ************************/

    for (rho = 0; rho < necc_; rho++) {
        if (rho < fov)
            if (rho == 0) {
                currRad[rho] = firstRing * 0.5;
                nextRad[rho] = firstRing;
            }
            else {
                currRad[rho] = rho + firstRing - 0.5;
                nextRad[rho] = currRad[rho] + 1.0;
            }
        else {
            currRad[rho] = pow (lambda, rho) * (r0 + 0.5 / pow (lambda, fov));
            nextRad[rho] = lambda * currRad[rho];
        }
        tangaxis[rho] =
            scaleFact * 2.0 * currRad[rho] * sinus * (overlap_ + 1.0) / (2.0);

        radialaxis[rho] =
            scaleFact * (nextRad[rho] - currRad[rho]) * (overlap_ + 1.0);

        if (rho < fov)
            radialaxis[rho] /= 2.0;
        else
            radialaxis[rho] /= (1.0 + lambda);

        if ((rho < fov) && (mode == ELLIPTICAL)) {
            A = radialaxis[rho] * radialaxis[rho];
            L = scaleFact * currRad[rho] * (overlap_ + 1.0);
            L = L * L;
            tangaxis[rho] = tangent * sqrt (L - A);
        }
    }
    radialaxis[0] = 0.5 * scaleFact * (firstRing) * (overlap_ + 1.0);

    if (mode == RADIAL)
        radii = radialaxis;
    else
        radii = tangaxis;

    for (rho = 0; rho < necc_; rho++) {
        if (mode != ELLIPTICAL)
            focus[rho] = 0;
        else {
            focus[rho] =
                -sqrt (fabs (radialaxis[rho] * radialaxis[rho] -
                             tangaxis[rho] * tangaxis[rho]));
        }

        if (tangaxis[rho] >= radialaxis[rho])
            focus[rho] = -focus[rho];

        radii2[rho] = radii[rho] * radii[rho];
    }

    sintable = new double[nang_];
    costable = new double[nang_];

    if ((sintable == 0) || (costable == 0))
        goto C2LAllocError;

    for (j = 0; j < nang_; j++) {
        sintable[j] = sin (angle * (j + 0.5));
        costable[j] = cos (angle * (j + 0.5));
    }

    // compute overall table size for contiguous allocation (position & weigth).
    sz = 0;
    for (rho = 0; rho < necc_; rho++) {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max64f (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;
        if (step > 1) step = 1;

        mapsize = (int) (precision * lim * precision * lim + 1);
        sz += (mapsize * nang_);
    }

    table->position = new int[sz * 2];
    if (table->position == 0)
        goto C2LAllocError;
    table->iweight = table->position + sz;

    for (rho = 0; rho < necc_; rho++) {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max64f (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;

        if (step > 1)
            step = 1;

        mapsize = (int) (precision * lim * precision * lim + 1);
        weight = new float[mapsize];
        if (weight == 0)
            goto C2LAllocError;

        for (theta = 0; theta < nang_; theta++) {
            //
            memset (table->position, 0, mapsize);
            memset (weight, 0, mapsize);

            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0) {
                F0x = x0 - focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else {
                F0x = x0 - focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }

            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step) {

                    intx = (int) (locX + width_ / 2);
                    inty = (int) (locY + height_ / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis) {
                        if ((inty < height_) && (inty >= 0)) {
                            if ((intx < width_) && (intx >= 0)) {
                                found = false;
                                j = 0;
                                while ((j < mapsize) && (table->position[j] != 0)) {
                                    //
                                    if (table->position[j] == (MONO_PIXEL_SIZE * (inty * (width_ + padding) + intx))) {
                                        weight[j]++;
                                        found = true;
                                        j = mapsize;
                                    }
                                    j++;
                                }

                                if (!found)
                                    for (j = 0; j < mapsize; j++) {
                                        if (table->position[j] == 0) {
                                            table->position[j] = MONO_PIXEL_SIZE * (inty * (width_ + padding) + intx);
                                            weight[j]++;
                                            break;
                                        }
                                    }
                            }
                        }
                    }
                }

            for (j = 0; j < mapsize; j++)
                if (weight[j] == 0)
                    break;

            table->divisor = j;

            float sum = 0.0;
            int k;
            for (k = 0; k < j; k++)
                sum += weight[k];

            for (k = 0; k < j; k++) {
                weight[k] = weight[k] / sum;
                table->iweight[k] = (int) (weight[k] * 65536.0);
            } 

            if (theta != nang_-1 || rho != necc_-1) {
                table[1].position = table->position + mapsize;
                table[1].iweight = table->iweight + mapsize;
            }
            table++;
        }

        delete[] weight;    // :(
    }

    // clean up temporaries.
    if (tangaxis) delete[] tangaxis;
    if (radialaxis) delete[] radialaxis;
    if (focus) delete[] focus;
    if (radii2) delete[] radii2;
    if (currRad) delete[] currRad;
    if (nextRad) delete[] nextRad;
    if (sintable) delete[] sintable;
    if (costable) delete[] costable;
    return 0;

C2LAllocError:
    // clean up temporaries.
    if (tangaxis) delete[] tangaxis;
    if (radialaxis) delete[] radialaxis;
    if (focus) delete[] focus;
    if (radii2) delete[] radii2;
    if (currRad) delete[] currRad;
    if (nextRad) delete[] nextRad;
    if (sintable) delete[] sintable;
    if (costable) delete[] costable;
    cerr << "logpolarTransform: memory allocation issue, no tables generated" << endl;
    return 2;
}

void logpolarTransformVisual::RCgetLpImg (unsigned char *lpImg, unsigned char *cartImg, cart2LpPixel * Table, int padding)
{
    int r[1];
    int t = 0;

    unsigned char *img = lpImg;

    for (int i = 0; i < necc_; i++, img+=padding) {
        for (int j = 0; j < nang_; j++) {
            r[0] = 0;
            t = 0;

            const int div = Table->divisor;
            int *pos = Table->position;
            int *w = Table->iweight;
            for (int i = 0; i < div; i++, pos++, w++) {
                int *d = r;
                unsigned char *in = &cartImg[*pos];
                //*d++ += *in++ * *w;
                //*d++ += *in++ * *w;
                *d += *in * *w;
                t += *w;   
            }

            *img++ = (unsigned char)(r[0] / t);
            //*img++ = (unsigned char)(r[1] / t);
            //*img++ = (unsigned char)(r[2] / t);

            Table++;
        }
    }
}

void logpolarTransformVisual::RCgetCartImg (unsigned char *cartImg, unsigned char *lpImg, lp2CartPixel * Table, int padding)
{
    int k, i, j;
    int tempPixel[1];
    unsigned char *img = cartImg;

    for (k = 0; k < height_; k++, img += padding) {
        for (j = 0; j < width_; j++) {
            tempPixel[0] = 0;
            //tempPixel[1] = 0;
            //tempPixel[2] = 0;

            if (Table->iweight != 0) {
                for (i = 0; i < Table->iweight; i++) {
                    int *d = tempPixel;
                    unsigned char *lp = &lpImg[Table->position[i]];
                    //*d++ += *lp++;
                    //*d++ += *lp++;
                    *d += *lp;
                }

                *img++ = *tempPixel/ Table->iweight;
                //*img++ = tempPixel[1] / Table->iweight;
                //*img++ = tempPixel[2] / Table->iweight;
            }
            else {
                *img++ = 0;
                //*img++ = 0;
                //*img++ = 0;
            }

            Table++;
        }
    }
}

// inverse logpolar.
int logpolarTransformVisual::RCbuildL2CMap (double scaleFact, int hOffset, int vOffset, int mode, int padding)
{
    if (overlap_ <= -1.0) {
        cerr << "logpolarTransform: overlap must be greater than -1" << endl;
        return 1;
    }

	int cSize = width_;
	
	if (width_ > height_)
		cSize = height_;

    double angle = (2.0 * PI / nang_);   // Angular size of one pixel
    double sinus = sin (angle / 2.0);
    double tangent = sinus / cos (angle / 2.0);
    int fov;                    // Number of rings in fovea
    int lim;

    double lambda;              // Log Index
    double firstRing;           // Diameter of the receptive fields in the first ring when overlap is 0
    double *currRad;            // Distance of the center of the current ring RF's from the center of the mapping
    double *nextRad;            // Distance of the center of the next ring's RF's from the center of the mapping
    double r0;                  // lower limit of RF0 in the "pure" log polar mapping

    double x0, y0;              // Cartesian Coordinates
    double locX, locY, locRad;
    double *radii;
    double *tangaxis;
    double *radialaxis;
    double *focus;
    double *radii2;
    double A;
    double L;
    int *partCtr;
    double step;
    double F0x, F0y, F1x, F1y;
    double maxaxis;

    int rho, theta, j, memSize;
    bool found;
    const double precision = 10.0;

    // main table pointer (temporary).
    lp2CartPixel *table = l2cTable;

    // temporary counter (per pixel).
    partCtr = new int[width_ * height_];
    if (partCtr == 0)
        goto L2CAllocError;

    memset (partCtr, 0, width_ * height_ * sizeof (int));

    lambda = (1.0 + sinus) / (1.0 - sinus);
    fov = (int) (lambda / (lambda - 1));
    firstRing = (1.0 / (lambda - 1)) - (int) (1.0 / (lambda - 1));
    r0 = 1.0 / (pow (lambda, fov) * (lambda - 1));

    tangaxis = new double[necc_];
    radialaxis = new double[necc_];
    focus = new double[necc_];
    radii2 = new double[necc_];
    currRad = new double[necc_];
    nextRad = new double[necc_];

    if ((tangaxis == 0) || (radialaxis == 0) || (focus == 0) || (radii2 == 0) || (currRad == 0) || (nextRad == 0))
        goto L2CAllocError;

    /************************
     * RF's size Computation *
     ************************/

    for (rho = 0; rho < necc_; rho++) {
        if (rho < fov)
            if (rho == 0) {
                currRad[rho] = firstRing * 0.5;
                nextRad[rho] = firstRing;
            }
            else {
                currRad[rho] = rho + firstRing - 0.5;
                nextRad[rho] = currRad[rho] + 1.0;
            }
        else {
            currRad[rho] = pow (lambda, rho) * (r0 + 0.5 / pow (lambda, fov));
            nextRad[rho] = lambda * currRad[rho];
        }
        
        tangaxis[rho] =
            scaleFact * 2.0 * currRad[rho] * sinus * (overlap_ + 1.0) / (2.0);

        radialaxis[rho] =
            scaleFact * (nextRad[rho] - currRad[rho]) * (overlap_ + 1.0);

        if (rho < fov)
            radialaxis[rho] /= 2.0;
        else
            radialaxis[rho] /= (1.0 + lambda);

        if ((rho < fov) && (mode == ELLIPTICAL))
        {
            A = radialaxis[rho] * radialaxis[rho];
            L = scaleFact * currRad[rho] * (overlap_ + 1.0);
            L = L * L;
            tangaxis[rho] = tangent * sqrt (L - A);
        }
    }

    radialaxis[0] = 0.5 * scaleFact * (firstRing) * (overlap_ + 1.0);

    if (mode == RADIAL)
        radii = radialaxis;
    else
        radii = tangaxis;

    for (rho = 0; rho < necc_; rho++) {
        if (mode != ELLIPTICAL)
            focus[rho] = 0;
        else {
            focus[rho] =
                -sqrt (fabs (radialaxis[rho] * radialaxis[rho] -
                             tangaxis[rho] * tangaxis[rho]));
        }
        if (tangaxis[rho] >= radialaxis[rho])
            focus[rho] = -focus[rho];

        radii2[rho] = radii[rho] * radii[rho];
    }
    
    double *sintable;
    double *costable;
    int intx, inty;

    sintable = new double[nang_];
    costable = new double[nang_];
    if ((sintable == 0) || (costable == 0))
        goto L2CAllocError;

    for (j = 0; j < nang_; j++) {
        sintable[j] = sin (angle * (j + 0.5));  // Angular positions of the centers of the RF's
        costable[j] = cos (angle * (j + 0.5));
    }

    memSize = 0;
    for (rho = 0; rho < necc_; rho++) {
        //
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max64f (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;
        if (step > 1)
            step = 1;

        for (theta = 0; theta < nang_; theta++) {
            //
            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0) {
                F0x = x0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else {
                F0x = x0 - focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }
            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step) {
                    //
                    intx = (int) (locX + width_ / 2);
                    inty = (int) (locY + height_ / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis)
                        if ((inty + vOffset < height_) && (inty + vOffset >= 0))
                            if ((intx + hOffset < width_)
                                && (intx + hOffset >= 0)) {
                                //
                                partCtr[(inty + vOffset) * width_ + intx + hOffset]++;
                                memSize ++;
                            }
                }
        }
    }

    table->position = new int[memSize]; // contiguous allocation.
    if (table->position == 0)
        goto L2CAllocError;
    memset(table->position, -1, sizeof(int) * memSize);
    table->iweight = 0;

    for (j = 1; j < width_ * height_; j++) {
        table[j].position = table[j-1].position + partCtr[j-1];
        table[j].iweight = 0;
    }

    for (rho = 0; rho < necc_; rho++) {
        if ((mode == RADIAL) || (mode == TANGENTIAL))
            lim = (int) (radii[rho] + 1.5);
        else
            lim = (int) (__max64f (tangaxis[rho], radialaxis[rho]) + 1.5);

        step = lim / precision;

        if (step > 1)
            step = 1;

        for (theta = 0; theta < nang_; theta++) {
            //
            x0 = scaleFact * currRad[rho] * costable[theta];
            y0 = scaleFact * currRad[rho] * sintable[theta];

            if (focus[rho] >= 0) {
                F0x = x0 - focus[rho] * sintable[theta];
                F0y = y0 + focus[rho] * costable[theta];
                F1x = x0 + focus[rho] * sintable[theta];
                F1y = y0 - focus[rho] * costable[theta];
                maxaxis = tangaxis[rho];
            }
            else {
                F0x = x0 - focus[rho] * costable[theta];
                F0y = y0 - focus[rho] * sintable[theta];
                F1x = x0 + focus[rho] * costable[theta];
                F1y = y0 + focus[rho] * sintable[theta];
                maxaxis = radialaxis[rho];
            }
            if ((mode == RADIAL) || (mode == TANGENTIAL))
                maxaxis = radii[rho];

            for (locX = (x0 - lim); locX <= (x0 + lim); locX += step)
                for (locY = (y0 - lim); locY <= (y0 + lim); locY += step) {
                    intx = (int) (locX + width_ / 2);
                    inty = (int) (locY + height_ / 2);

                    locRad =
                        sqrt ((locX - F0x) * (locX - F0x) +
                              (locY - F0y) * (locY - F0y));
                    locRad +=
                        sqrt ((locX - F1x) * (locX - F1x) +
                              (locY - F1y) * (locY - F1y));

                    if (locRad < 2 * maxaxis)
                        if ((inty + vOffset < height_) && (inty + vOffset >= 0))
                            if ((intx + hOffset < width_) && (intx + hOffset >= 0)) {
                                found = false;

                                for (j = 0; j < partCtr[(inty + vOffset) * width_ + intx + hOffset]; j++) {
                                    if (table [(inty + vOffset) * width_ + intx + hOffset].position[j] == MONO_PIXEL_SIZE * (rho * nang_ + theta) + (padding * rho)) {
                                        found = true;
                                        break;
                                    }
                                }

                                if (!found) {
                                    table[(inty + vOffset) * width_ + intx + hOffset].position[table [(inty + vOffset) * width_ + intx + hOffset].iweight] = MONO_PIXEL_SIZE * (rho * nang_ + theta) + (padding * rho);
                                    table[(inty + vOffset) * width_ + intx + hOffset].iweight++;
                                }
                            }
                }
        }
    }

    if (tangaxis) delete[] tangaxis;
    if (radialaxis) delete[] radialaxis;
    if (focus) delete[] focus;
    if (radii2) delete[] radii2;
    if (currRad) delete[] currRad;
    if (nextRad) delete[] nextRad;
    if (sintable) delete[] sintable;
    if (costable) delete[] costable;
    if (partCtr) delete [] partCtr;
    return 0;

L2CAllocError:
    if (tangaxis) delete[] tangaxis;
    if (radialaxis) delete[] radialaxis;
    if (focus) delete[] focus;
    if (radii2) delete[] radii2;
    if (currRad) delete[] currRad;
    if (nextRad) delete[] nextRad;
    if (sintable) delete[] sintable;
    if (costable) delete[] costable;
    if (partCtr) delete [] partCtr;
    cerr << "logpolarTransform: memory allocation issue, no tables generated" << endl;
    return 2;
}




// end of log polar transform
/************************************************************/


const int maxKernelSize = 5;

// defining seperate vectors (horizontal and vertical) for gaussians of size 5x5 (sigma 1) and 7x7 (sigma 3)
// method to do so could be: 1. [u s v] = svd(G) 2. Normalize columns of u and v, by sqrt of largest singular
// value ie s(1,1)
static float G7[7] = {      -0.1063f,
                           -0.1403f,
                           -0.1658f,
                           -0.1752f,
                           -0.1658f,
                           -0.1403f,
                           -0.1063f
                    };

static float G5[5] = { -0.0545f,
                       -0.2442f,
                       -0.4026f,
                       -0.2442f,
                       -0.0545f
                     };

// these are values for Gabor params 1.2, 128, 0 , 5, theta=0. Imprecise values for now!
static float Gab7V[7] = { -0.0298,
                           -0.1703,
                           -0.4844,
                           -0.6863,
                           -0.4844,
                           -0.1703,
                           -0.0298
                        };

static float Gab7H[7] = { -0.0001,   -0.0186,   -0.3354,   -0.8799,   -0.3354,   -0.0186,   -0.0001 };


template<class T>
inline T max(T a, T b, T c) {    
    if(b > a) a = b;
    if(c > a) a = c;
    return a;
}

visualFilterThread::visualFilterThread() {
    
    inputExtImage      = new ImageOf<PixelRgb>;
    inputImage         = new ImageOf<PixelRgb>;
    cartImage         = new ImageOf<PixelRgb>;
    
    inputImageFiltered = new ImageOf<PixelRgb>;
    logPolarImage      = new ImageOf<PixelRgb>;
    
    redGreen           = new ImageOf<PixelMono>;
    cartRedGreen       = new ImageOf<PixelMono>;
    cartGreenRed       = new ImageOf<PixelMono>;
    cartBlueYellow       = new ImageOf<PixelMono>;
    upSampleRG          = new ImageOf<PixelMono>;
    upSampleGR          = new ImageOf<PixelMono>;
    upSampleBY          = new ImageOf<PixelMono>;
    greenRed           = new ImageOf<PixelMono>;
    blueYellow         = new ImageOf<PixelMono>;
    pyImage            = new ImageOf<PixelMono>;
    
    
    
    //getKernels(sigma, lambda, psi, gamma);
    sigma = 1.2;
    gLambda = 128;
    psi = 0;
    gamma = 5;
    kernelUsed = 2;
    dwnSam = 2;

    // 7x7 kernel for negative gaussian 
    float K[] = {
        0.0113f, 0.0149f, 0.0176f, 0.0186f, 0.0176f, 0.0149f, 0.0113f,
        0.0149f, 0.0197f, 0.0233f, 0.0246f, 0.0233f, 0.0197f, 0.0149f,
        0.0176f, 0.0233f, 0.0275f, 0.0290f, 0.0275f, 0.0233f, 0.0176f,
        0.0186f, 0.0246f, 0.0290f, 0.0307f, 0.0290f, 0.0246f, 0.0186f,
        0.0176f, 0.0233f, 0.0275f, 0.0290f, 0.0275f, 0.0233f, 0.0176f,
        0.0149f, 0.0197f, 0.0233f, 0.0246f, 0.0233f, 0.0197f, 0.0149f,
        0.0113f, 0.0149f, 0.0176f, 0.0186f, 0.0176f, 0.0149f, 0.0113f
    };

    kernel = cvCreateMat( 7, 7, CV_32FC1 );
    cvSetData ( kernel, (float*)K, sizeof ( float ) * 7 );
        
    getKernels();

          
    
    edges = new ImageOf<PixelMono>;    
    lambda = 0.1f;
    resized = false;

    xSizeValue = 320 ;         
    ySizeValue = 240;          // y dimension of the remapped cartesian image
    overlap = 1.0;         // overlap in the remapping
    numberOfRings = 152;      // number of rings in the remapping
    numberOfAngles = 252;     // number of angles in the remapping
    
    St = yarp::os::Stamp(0,0);
}

visualFilterThread::~visualFilterThread() {
    
    delete inputExtImage;
    delete inputImageFiltered;
    delete inputImage;
    delete cartImage;
    delete edges;
    delete pyImage;
    delete redGreen;
    delete cartRedGreen;
    delete cartGreenRed;
    delete cartBlueYellow;
    delete upSampleRG;
    delete upSampleGR;
    delete upSampleBY;
    delete greenRed;
    delete blueYellow;
    delete logPolarImage;
    for(int i=0; i<4; ++i){
        if(gabKer[i]!=0)
            cvReleaseMatHeader(&gabKer[i]);
    }
    printf("Calling destructor \n");
    //delete dwnImage;

    
}

bool visualFilterThread::threadInit() {
    printf("opening ports \n");
    /* open ports */ 
    if (!imagePortIn.open(getName("/image:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imagePortOut.open(getName("/image:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    if (!imagePortExt.open(getName("/imageExt:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!rgPort.open(getName("/rg:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!grPort.open(getName("/gr:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!byPort.open(getName("/by:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!pyImgPort.open(getName("/pyImg:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    //initializing logpolar mapping
    cout << "||| initializing the logpolar mapping" << endl;

    if (!trsf.allocLookupTables(L2C, numberOfRings, numberOfAngles, xSizeValue, ySizeValue, overlap)) {
        cerr << "can't allocate lookup tables" << endl;
        return false;
    }
    cout << "|-| lookup table allocation done" << endl;

    if (!lpMono.allocLookupTables(BOTH, numberOfRings, numberOfAngles, xSizeValue, ySizeValue, overlap)) {
        cerr << "can't allocate lookup tables for mono" << endl;
        return false;
    }
    cout << "|-| lookup table allocation for mono done" << endl;


    
    return true;
}

void visualFilterThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string visualFilterThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void visualFilterThread::run() {
    
    
    while (isStopping() != true) {
        inputImage = imagePortIn.read(true);
        
        if (inputImage != NULL) {
            if (!resized) {
                //printf("new image found %d %d", inputImage->width(),inputImage->height() );
                resize(inputImage->width(), inputImage->height());
                int sizecvRedPlus = cvRedPlus->width;
    
                //printf("image successfull ");
                resized = true;
            }
            else {
                filterInputImage();
            }            
            resizeCartesian(320,240);
            //printf("red plus dimension in resize1  %d %d \n", cvRedPlus->width, cvRedPlus->height);
             

            //filtering input image
            //printf("filtering \n");
            filterInputImage();
             //printf("red plus dimension in resize2  %d %d \n", cvRedPlus->width, cvRedPlus->height);
 
            // extend logpolar input image
            extender(inputImage, maxKernelSize);
             //printf("red plus dimension in resize3  %d %d \n", cvRedPlus->width, cvRedPlus->height);
            
            // extract RGB and Y planes
            extractPlanes();
             //printf("red plus dimension in resize4  %d %d \n", cvRedPlus->width, cvRedPlus->height);
                      
            // gaussian filtering of the of RGB and Y
            filtering();
            

            // colourOpponency map construction
            //printf("before colour opponency \n");
            colourOpponency();
            // apply sobel operators on the colourOpponency maps and combine via maximisation of the 3 edges

            
            edgesExtract();
        
            
            // sending the edge image on the outport            
            // the copy to the port object can be avoided...
            
            if((edges!=0)&&(imagePortOut.getOutputCount())) {
                imagePortOut.prepare() = *(edges);
                imagePortOut.write();
            }
            if((redGreen!=0)&&(rgPort.getOutputCount())) {
                rgPort.prepare() = *(upSampleRG);
                rgPort.write();
            }
            if((greenRed!=0)&&(grPort.getOutputCount())) {
                grPort.prepare() = *(cartRedGreen);
                grPort.write();
            }
            if((blueYellow!=0)&&(byPort.getOutputCount())) {
                byPort.prepare() = *(cartBlueYellow);
                byPort.write();
            }
            if((inputExtImage!=0)&&(imagePortExt.getOutputCount())) {
                imagePortExt.prepare() = *(inputExtImage);
                imagePortExt.write();
                }
            if((pyImage!=0)&&(pyImgPort.getOutputCount())) {
                pyImgPort.prepare() = *(pyImage);
                pyImgPort.write();
            }
            
        }
   }
}

void visualFilterThread::resizeCartesian(int width,int height) {
    cartImage->resize(width, height);
    width_cart = width;
    height_cart = height;
}


void visualFilterThread::resize(int width_orig,int height_orig) {


    this->width_orig = width_orig;
    this->height_orig = height_orig;
    
    this->width = width_orig+2*maxKernelSize;
    this->height = height_orig+maxKernelSize;
    
    //printf("width after reposition %d %d \n", width , height);


    //resizing yarp image 
    redGreen->resize(width, height);    
    edges->resize(width_orig, height_orig);
    inputImageFiltered->resize(width_orig, height_orig);
    inputImageFiltered->zero();
    inputExtImage->resize(width,height);


    CvSize cvLogSize = cvSize(width, height);
    CvSize cvCartSize = cvSize(320,240);

    cartRedGreen->resize(320, 240);    
    cartGreenRed->resize(320, 240);    
    cartBlueYellow->resize(320, 240);

    upSampleRG->resize(320, 240);  
    upSampleGR->resize(320, 240);  
    upSampleBY->resize(320, 240);  
    
    //allocate IplImages for color planes
    cvRedPlane = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvGreenPlane = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvBluePlane = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvYellowPlane = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );

    //allocate IplImages for color opponents
    redG = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cartRedG = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );
    greenR = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    blueY = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );

    //allocate IplImages for positive and negative gaussian convolution on image planes
    cvRedMinus  = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvRedPlus   = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    tmpRedPlus  = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    tmpRedMinus = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );

    cvGreenMinus    = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvGreenPlus     = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    tmpGreenPlus    = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    tmpGreenMinus   = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );

    cvYellowMinus = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvBluePlus = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    tmpYellowMinus  = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    tmpBluePlus     = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );

    
    
    //allocate IplImages for horizontal and vertical components of color opponents (after Sobel operator is applied)
    hRG = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    vRG = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    hGR = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    vGR = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    hBY = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    vBY = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );

    //allocate temporary 16 bit deep IplImages for Sobel operator result
    tempHRG = cvCreateImage(cvLogSize,IPL_DEPTH_16S, 1 );
    tempVRG = cvCreateImage(cvLogSize,IPL_DEPTH_16S, 1 );
    tempHGR = cvCreateImage(cvLogSize,IPL_DEPTH_16S, 1 );
    tempVGR = cvCreateImage(cvLogSize,IPL_DEPTH_16S, 1 );
    tempHBY = cvCreateImage(cvLogSize,IPL_DEPTH_16S, 1 );
    tempVBY = cvCreateImage(cvLogSize,IPL_DEPTH_16S, 1 );


    
    //allocate space for openCV images for Gabor
    intensityImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    filteredIntensityImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    filteredIntensityImage1 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    filteredIntensityImage2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    totImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 ); // this is later resized
    dwnSample2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample4 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample8 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample4 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample8 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    
    dwnSample2a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample4a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample8a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample2a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample4a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample8a = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );

    dwnSample2b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample4b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSample8b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample2b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample4b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSample8b = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    

   
    
   
    
}

void visualFilterThread::filterInputImage() {
    int i;
    const int sz = inputImage->getRawImageSize();
    unsigned char * pFiltered = inputImageFiltered->getRawImage();
    unsigned char * pCurr = inputImageFiltered->getRawImage();
    const float ul = 1.0f - lambda;
    for (i = 0; i < sz; i++) {
        *pFiltered = (unsigned char)(lambda * *pCurr++ + ul * *pFiltered + .5f);
        pFiltered ++;
    }
}

ImageOf<PixelRgb>* visualFilterThread::extender(ImageOf<PixelRgb>* inputOrigImage, int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*inputExtImage, *inputOrigImage, maxSize);
    return inputExtImage;
}

void visualFilterThread::cartremap(ImageOf<PixelRgb>* cartesianImage, ImageOf<PixelRgb>* logpolarImage) {
    trsf.logpolarToCart (*cartesianImage, *logpolarImage);
}

void visualFilterThread::extractPlanes() {
    // We extract color planes from the RGB image. Planes are red,blue, green and yellow (AM of red & green)
    uchar* shift[3];
    uchar* yellowP;
    uchar* tmpIntensityImage;
    unsigned char* inputPointer;
    
    // Pointers to raw openCV monochrome image
    shift[0] = (uchar*) cvRedPlane->imageData; 
    shift[1] = (uchar*) cvGreenPlane->imageData;
    shift[2] = (uchar*) cvBluePlane->imageData;
    yellowP  = (uchar*) cvYellowPlane->imageData;

    // Pointer to raw extended input (RGB) image
    inputPointer = inputExtImage->getRawImage();
    uchar* originInputImage = inputPointer;
    uchar* originCVRedPlane = (uchar*) cvRedPlane->imageData;
    uchar* originCVGreenPlane = (uchar*) cvGreenPlane->imageData;
    uchar* originCVBluePlane = (uchar*) cvBluePlane->imageData;
    uchar* originCVYellowPlane = (uchar*) cvYellowPlane->imageData;
    int widthInputImage = inputExtImage->getRowSize();
    int widthcvR = cvRedPlane->widthStep;
    int widthcvG = cvGreenPlane->widthStep;
    int widthcvB = cvBluePlane->widthStep;
    int widthcvY = cvYellowPlane->widthStep;

    /* We can avoid padding for openCV images*/
    int paddingMono = inputExtImage->getPadding(); 
    int padding3C = inputExtImage->getPadding(); 

    const int h = inputExtImage->height();
    const int w = inputExtImage->width();
    for(int r = 0; r < h; r++) {

        inputPointer = originInputImage + r* widthInputImage;
        shift[0] = originCVRedPlane + r* widthcvR;
        shift[1] = originCVGreenPlane + r*widthcvG;
        shift[2] = originCVBluePlane + r*widthcvB;
        yellowP = originCVYellowPlane + r*widthcvY;
        
        for(int c = 0; c < w; c++) {
            *shift[0] = *inputPointer++;
            *shift[1] = *inputPointer++;
            *shift[2] = *inputPointer++;

            *yellowP++ = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));
            
            shift[0]++;
            shift[1]++;
            shift[2]++;
        }

        
    }

}

void visualFilterThread::filtering() {
    // We gaussian blur the image planes extracted before, one with positive Gaussian and then negative
    
    //Positive
    convolve1D(5,G5,cvRedPlane,tmpRedPlus,1.0,1);
    convolve1D(5,G5,tmpRedPlus,cvRedPlus,1.0,0);

    convolve1D(5,G5,cvGreenPlane,tmpGreenPlus,1.0,1);
    convolve1D(5,G5,tmpGreenPlus,cvGreenPlus,1.0,0);

    convolve1D(5,G5,cvBluePlane,tmpBluePlus,1.0,1);
    convolve1D(5,G5,tmpBluePlus,cvBluePlus,1.0,0);
   
    
    
    //Negative
    convolve1D(7,G7,cvRedPlane,tmpRedMinus,1.0,1);
    convolve1D(7,G7,tmpRedMinus,cvRedMinus,1.0,0);

    convolve1D(7,G7,cvGreenPlane,tmpGreenMinus,1.0,1);
    convolve1D(7,G7,tmpGreenMinus,cvGreenMinus,1.0,0);

    convolve1D(7,G7,cvYellowPlane,tmpYellowMinus,1.0,1);
    convolve1D(7,G7,tmpYellowMinus,cvYellowMinus,1.0,0);

    int crn[4]={5,5,252,152};
    IplImage* tmp;
    

    
    
}

void visualFilterThread::colourOpponency() {
    
    // we want RG = (G- - R+)/2 -128, GR = (R- - G+)/2 and BY = (Y- -B+)/2 -128. These values are obtained after filtering with positive and negative Gaussian.

    const int h = height;
    const int w = width;
    

    //printf("inside the colourOpponency ");
    int pad = redG->widthStep - redG->width;
    //uchar* originRG = 

    uchar* rMinus = (uchar*)cvRedMinus->imageData;
    uchar* rPlus = (uchar*)cvRedPlus->imageData;

    uchar* gMinus = (uchar*)cvGreenMinus->imageData;
    uchar* gPlus = (uchar*)cvGreenPlus->imageData;;

    uchar* yMinus = (uchar*)cvYellowMinus->imageData;
    uchar* bPlus = (uchar*)cvBluePlus->imageData;

    uchar* RG = (uchar*)redG->imageData;
    uchar* GR = (uchar*)greenR->imageData;
    uchar* BY = (uchar*)blueY->imageData;

    for(int r = 0; r < h; r++) {
        for(int c = 0; c < w; c++) {
            
            *RG++ = ((*rPlus >> 1) + 128 - (*gMinus >> 1) );
            *GR++ = ((*gPlus >> 1) + 128 - (*rMinus >> 1) );
            *BY++ = ((*bPlus >> 1) + 128 - (*yMinus >> 1) );

            rMinus++;
            rPlus++;
            gMinus++;
            gPlus++;
            yMinus++;
            bPlus++;
        }

        rMinus += pad;
        rPlus  += pad;
        gMinus += pad;
        gPlus  += pad;
        yMinus += pad;
        bPlus  += pad;
        RG += pad;
        GR += pad;
        BY += pad;

    }

 /*
    cvNamedWindow("test1");
    cvShowImage("test1",redG);
    cvNamedWindow("test2");
    cvShowImage("test2",greenR);
    cvNamedWindow("test3");
    cvShowImage("test3",blueY);
    cvWaitKey(0);*/

    if(redG == NULL || greenR == NULL) return;
    IplImage* tmpRedGreen;
    tmpRedGreen = cvCreateImage(cvSize(252,152),IPL_DEPTH_8U, 1 );
    IplImage* tmpGreenRed;
    tmpGreenRed = cvCreateImage(cvSize(252,152),IPL_DEPTH_8U, 1 );
    IplImage* tmpBlueYellow;
    tmpBlueYellow = cvCreateImage(cvSize(252,152),IPL_DEPTH_8U, 1 );
    int cor[4]={5,5,257,157};
    cropImage(cor,redG,tmpRedGreen);
    cropImage(cor,greenR,tmpGreenRed);
    cropImage(cor,blueY,tmpBlueYellow);
    
    redGreen->zero();
    openCVtoYARP(tmpRedGreen,redGreen,1);
    
    greenRed->zero();
    openCVtoYARP(tmpGreenRed,greenRed,1);
    blueYellow->zero();
    openCVtoYARP(tmpBlueYellow,blueYellow,1);
    
         
    // Converting R+G- to cartesian , similarly for others
    lpMono.logpolarToCart (*cartRedGreen, *redGreen);
    lpMono.logpolarToCart (*cartGreenRed, *greenRed);
    lpMono.logpolarToCart (*cartBlueYellow, *blueYellow);

    float weight[3]= {.33,.33,.33};

    downSampleImage((IplImage*)cartRedGreen->getIplImage(), dwnSample2a,2);
    //downSampleImage(greenR, dwnSample4a,2);
    //downSampleImage(blueY, dwnSample8a,2);
    
    

    // filter downsampled images
    dwnSample2Fila = cvCreateImage(cvGetSize(dwnSample2a),IPL_DEPTH_8U, 1 );
    //dwnSample4Fila = cvCreateImage(cvGetSize(dwnSample4a),IPL_DEPTH_8U, 1 );
    //dwnSample8Fila = cvCreateImage(cvGetSize(dwnSample8a),IPL_DEPTH_8U, 1 );

    IplImage* tmpdwnSample2Fil;
    tmpdwnSample2Fil = cvCreateImage(cvGetSize(dwnSample2a),IPL_DEPTH_8U, 1 );
    convolve1D(7,Gab7H,dwnSample2a,tmpdwnSample2Fil,.1,0); // convolve with horizontal 
    convolve1D(7,Gab7V,tmpdwnSample2Fil,dwnSample2Fila,.1,1);   
    //cvFilter2D(dwnSample2a,dwnSample2Fila,gabKer[kernelUsed],cvPoint(-1,-1));
    //cvFilter2D(dwnSample4a,dwnSample4Fila,gabKer[kernelUsed],cvPoint(-1,-1));
    //cvFilter2D(dwnSample8a,dwnSample8Fila,gabKer[kernelUsed],cvPoint(-1,-1));

    

    //up-sample the filtered images
    upSampleImage(dwnSample2Fila,upSample2a,2);
    //upSampleImage(dwnSample4Fila,upSample4a,2);
    //upSampleImage(dwnSample8Fila,upSample8a,2);    
    

    // add the images with defined weightages
    //IplImage* imagesToAdd2[3]= {upSample2a,upSample4a,upSample8a};
    //float weight[3]= {.33,.33,.33};
    //cvSet(filteredIntensityImage2, cvScalar(0));
    //maxImages(imagesToAdd2,1,filteredIntensityImage2,weight);
    

    /*
    // downsample the images by dimension 4
    downSampleImage(redG, dwnSample4,4);
    downSampleImage(greenR, dwnSample2,4);
    downSampleImage(blueY, dwnSample8,4);
  
    // filter downsampled images
    dwnSample2Fil = cvCreateImage(cvGetSize(dwnSample2),IPL_DEPTH_8U, 1 );
    dwnSample4Fil = cvCreateImage(cvGetSize(dwnSample4),IPL_DEPTH_8U, 1 );
    dwnSample8Fil = cvCreateImage(cvGetSize(dwnSample8),IPL_DEPTH_8U, 1 );

    cvFilter2D(dwnSample2,dwnSample2Fil,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample4,dwnSample4Fil,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample8,dwnSample8Fil,gabKer[kernelUsed],cvPoint(-1,-1));

    //up-sample the filtered images
    upSampleImage(dwnSample2Fil,upSample2,4);
    upSampleImage(dwnSample4Fil,upSample4,4);
    upSampleImage(dwnSample8Fil,upSample8,4);

    // add the images with defined weightages
    IplImage* imagesToAdd[3]= {upSample2,upSample4,upSample8};
    cvSet(filteredIntensityImage, cvScalar(0));
    maxImages(imagesToAdd,3,filteredIntensityImage,weight);
    */
    
    //-------------------------------------------------------------------
    
    
    //----------------------------------------------------------------------
    /*
    // downsample the images by dimension 2
    downSampleImage(redG, dwnSample4b,8);
    downSampleImage(greenR, dwnSample2b,8);
    downSampleImage(blueY, dwnSample8b,8);

    // filter downsampled images
    dwnSample2Filb = cvCreateImage(cvGetSize(dwnSample2b),IPL_DEPTH_8U, 1 );
    dwnSample4Filb = cvCreateImage(cvGetSize(dwnSample4b),IPL_DEPTH_8U, 1 );
    dwnSample8Filb = cvCreateImage(cvGetSize(dwnSample8b),IPL_DEPTH_8U, 1 );

    cvFilter2D(dwnSample2b,dwnSample2Filb,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample4b,dwnSample4Filb,gabKer[kernelUsed],cvPoint(-1,-1));
    cvFilter2D(dwnSample8b,dwnSample8Filb,gabKer[kernelUsed],cvPoint(-1,-1));

    //up-sample the filtered images
    upSampleImage(dwnSample2Filb,upSample2b,8);
    upSampleImage(dwnSample4Filb,upSample4b,8);
    upSampleImage(dwnSample8Filb,upSample8b,8);

    // add the images with defined weightages
    IplImage* imagesToAdd8[3]= {upSample2b,upSample4b,upSample8b};
    //float weight[3]= {.33,.33,.33};
    cvSet(filteredIntensityImage1, cvScalar(0));
    maxImages(imagesToAdd8,3,filteredIntensityImage1,weight);
    */
 
    /********************************************/
    
    
    //convert openCV image to YARP image
    //float wt[3]={.05,.45,.5};
    //IplImage* imagesToAddX[3]={filteredIntensityImage2,filteredIntensityImage,filteredIntensityImage1};
    //cvSet(totImage,cvScalar(0));
    //addImages(imagesToAddX,1,totImage,wt);

    
    upSampleRG->zero();
    openCVtoYARP(upSample2a,upSampleRG,1);
    

    
  
}



void visualFilterThread::edgesExtract() {

    
    // Since Sobel doesnt do scaling, destination is 16 bit
    cvSobel(redG,tempHRG,1,0,3);   
    cvSobel(redG,tempVRG,0,1,3);
    cvSobel(greenR,tempHGR,1,0,3);
    cvSobel(greenR,tempVGR,0,1,3);
    cvSobel(blueY,tempHBY,1,0,3);
    cvSobel(blueY,tempVBY,0,1,3);

    short* ptrtempHRG;
    char* temphRG;
    short maxVal = -256;
   temphRG = hRG->imageData;
    minVal = 257; 

            


    // Scaling the destination back to 8 bit
    cvConvertScaleAbs(tempHRG,hRG,1.00,0.0);
    cvConvertScaleAbs(tempVRG,vRG,1.00,0.0);
    cvConvertScaleAbs(tempHGR,hGR,1.00,0.0);
    cvConvertScaleAbs(tempVGR,vGR,1.00,0.0);
    cvConvertScaleAbs(tempHBY,hBY,1.00,0.0);
    cvConvertScaleAbs(tempVBY,vBY,1.00,0.0);

    
    ptrtempHRG = (short*)tempHRG->imageData;
    maxVal = 256;
    for(int i = 0; i<width; ++i){
        for(int j=0; j<height; ++j){
            if(maxVal > *ptrtempHRG) maxVal = *ptrtempHRG; 
            ptrtempHRG++;
        }
    }
    
    //clearing up the previous value
    edges->zero();
    uchar* pedges=edges->getRawImage();
    const int pad_edges = edges->getPadding();

    // can be optimized later
    uchar* originPedges = edges->getRawImage();
    int widthEdges = edges->getRowSize();
    uchar* originHRG = (uchar*)hRG->imageData + hRG->widthStep * (height - height_orig);
    uchar* originHGR = (uchar*)hGR->imageData + hGR->widthStep * (height - height_orig);
    uchar* originHBY = (uchar*)hBY->imageData + hBY->widthStep * (height - height_orig);
    uchar* originVRG = (uchar*)vRG->imageData + vRG->widthStep * (height - height_orig);
    uchar* originVGR = (uchar*)vGR->imageData + vGR->widthStep * (height - height_orig);
    uchar* originVBY = (uchar*)vBY->imageData + vBY->widthStep * (height - height_orig);

    int widthStepCV = hRG->widthStep;
    
    
    /*cvNamedWindow("test1");
    cvShowImage("test1",hRG);
    cvNamedWindow("test2");
    cvShowImage("test2",redG);
    cvNamedWindow("test3");
    cvShowImage("test3",cvRedPlus);
    cvWaitKey(0);*/
    //cvShowImage( "test1", hBY); cvWaitKey(0);

    uchar* ptrHRG = (uchar*)hRG->imageData;
    uchar* ptrVRG = (uchar*)vRG->imageData;
    uchar* ptrHGR = (uchar*)hGR->imageData;
    uchar* ptrVGR = (uchar*)vGR->imageData;
    uchar* ptrHBY = (uchar*)hBY->imageData;
    uchar* ptrVBY = (uchar*)vBY->imageData;

    //The extra portion of extended image can be neglected, hence the pointers are shifted accordingly. 
    int j = (hRG->widthStep)*(height-height_orig)+ (width-width_orig)/2; 

    // At the end of the original width, extended image pointers need to jump more 
    int gapInWidth = (width - width_orig)/2;
    const int padXtnd = gapInWidth + (hRG->widthStep - hRG->width);

    ptrHRG += j; ptrVRG += j;
    ptrHGR += j; ptrVGR += j;
    ptrHBY += j; ptrVBY += j;


    for (int row = 0; row < height_orig; row++) {
        for (int col = 0; col < width_orig; col++) {

            double rg = (*ptrHRG ) * (*ptrHRG ) + (*ptrVRG ) * (*ptrVRG );
            double gr = (*ptrHGR ) * (*ptrHGR ) + (*ptrVGR) * (*ptrVGR);
            double by = (*ptrHBY) * (*ptrHBY ) + (*ptrVBY ) * (*ptrVBY );
            if (row < height_orig) {
                *pedges = (unsigned char)(sqrt(max<double> (rg, gr, by))*ONE_BY_ROOT_TWO); //*(255.0 / 1024)); //normalised with theoric max-response 1448.16, 362.03                
            }
            else
                *pedges = 0;
            
            pedges++;
            ptrHRG++; ptrVRG++;
            ptrHGR++; ptrVGR++;
            ptrHBY++; ptrVBY++;
        }
        // padding
        pedges += pad_edges;
        //ptrHRG += padXtnd; ptrVRG += padXtnd;
        //ptrHGR += padXtnd; ptrVGR += padXtnd;
        //ptrHBY += padXtnd; ptrVBY += padXtnd;
        ptrHRG = originHRG + (row+1)*widthStepCV + gapInWidth;
        ptrVRG = originVRG + (row+1)*widthStepCV + gapInWidth;
        ptrHGR = originHGR + (row+1)*widthStepCV + gapInWidth;
        ptrVGR = originVGR + (row+1)*widthStepCV + gapInWidth;
        ptrHBY = originHBY + (row+1)*widthStepCV + gapInWidth;
        ptrVBY = originVBY + (row+1)*widthStepCV + gapInWidth;
    }
    
}

void visualFilterThread::setPar(int par, double value) {
    if(par == 1) this->sigma = value;
    else if(par == 2 ) this->gLambda = value;
    else if(par == 3) this->psi = value;
    else if(par == 4) this->gamma = value;
    else if(par == 5) this->kernelUsed = value;
    else if(par == 6) this->dwnSam = value;
    else if(par == 7) this->whichScale = value;
    getKernels();
}

void visualFilterThread::getKernels() {
    
    double sigmaX   = sigma;
    double sigmaY   = sigma/gamma;
    double theta[4] = {0, PI/4.0, PI/2.0, 3.0*PI/4.0};    
    int stdDev = 3;
    
    for(int i = 0; i < 4 ; ++i) {     
        double xLim = max<double>(1.0,max<double>(abs(cos(theta[i])*sigmaX*stdDev),abs(sin(theta[i])*sigmaY*stdDev)));
        double yLim = max<double>(1.0,max<double>(abs(sin(theta[i])*sigmaX*stdDev),abs(cos(theta[i])*sigmaY*stdDev)));
        
        double xStepSize = xLim/(KERNEL_ROW/2);
        double yStepSize = yLim/(KERNEL_COL/2);
        //float* tmpKer = gaborKernel[0][0];
        double xThre = xStepSize/2;
        double yThre = yStepSize/2;
        int k = 0; int j = 0;
        int maxi = 0, maxj = 0, maxk = 0;
        
        
        for(double xV = -xLim; xV < xLim + xThre; xV += xStepSize) {
            k=0;
            for(double yV = -yLim; yV < yLim + yThre; yV += yStepSize) {
                double xT = xV * cos(theta[i]) + yV * sin(theta[i]);
                double yT = -xV * sin(theta[i]) + yV * cos(theta[i]);
                gK[i][j][k]= exp(-.5*(xT*xT/(sigmaX*sigmaX) + yT*yT/(sigmaY*sigmaY)))*cos(2.0*PI*xT/gLambda + psi)/(2.0*PI*sigmaX*sigmaY);
                printf("%f , ",gK[i][j][k]);
                gK[i][j][k] *= .3;
                k++;
            }
            
            j++;
            printf("\n");
        }

        printf("\n\n\n");
    } 

}

void visualFilterThread::downSampleImage(IplImage* OrigImg,IplImage* DwnImg, int factor) {

    // prepare the destination image, can be avoided for optimization
    cvReleaseImage(&DwnImg);
    DwnImg = cvCreateImage(cvSize(OrigImg->width/factor,OrigImg->height/factor),IPL_DEPTH_8U, 1 );
    cvSet(DwnImg,cvScalar(0));
    //printf("Down sampling: Original image size%d,%d \n",OrigImg->width,OrigImg->height);

    uchar* tmpOrigImg = (uchar*)OrigImg->imageData;
    uchar* tmpDwnImg = (uchar*)DwnImg->imageData;    
    uchar* origin = (uchar*)OrigImg->imageData;
    uchar* originDwn = (uchar*)DwnImg->imageData;

    // actual width of images, including paddings    
    int origWidth = OrigImg->widthStep;
    int dwnWidth = DwnImg->widthStep;

    for(int i=0; i<DwnImg->height*factor; ++i){
        //jump to beginning of row
        tmpOrigImg = origin + (i)*origWidth;
        for(int j=0; j<DwnImg->width*factor; ++j){
            //printf("i,j, %d,%d by fact %d,%d counter%d ",i,j,i/factor,j/factor,counter);
            *(tmpDwnImg + ((i)/factor)*dwnWidth + (j)/factor) += *tmpOrigImg/(factor*factor);
            tmpOrigImg++;
        }
        
        
    } 
      
                    
}

void visualFilterThread::upSampleImage(IplImage* OrigImg,IplImage* UpImg, int factor) {

    // prepare the destination image
    cvReleaseImage(&UpImg);
    UpImg = cvCreateImage(cvSize(OrigImg->width*factor,OrigImg->height*factor),IPL_DEPTH_8U, 1 );
    cvSet(UpImg,cvScalar(0));

    uchar* tmpOrigImg = (uchar*)OrigImg->imageData;    
    uchar* tmpUpImg = (uchar*)UpImg->imageData;
    uchar* originUpImg = (uchar*)UpImg->imageData;
    
    // actual width of images, including paddings
    int UpWidth = UpImg->widthStep;
    int OrigWidth = OrigImg->widthStep;

    for(int i=0; i<UpImg->height; ++i){
        for(int j=0; j<UpImg->width; ++j){
            *tmpUpImg = *(tmpOrigImg + (i/factor)*OrigWidth + j/factor);
            tmpUpImg++;
        }
        // jump to next row
        tmpUpImg = originUpImg + (i+1)*UpWidth;        
    }

}            

void visualFilterThread::downSampleMultiScales(IplImage* OrigImg) {

    // assuming 3 scaled down images are prepared already
    // prepare the destination image, can be avoided for optimization
    cvReleaseImage(&dwnSample2);
    dwnSample2 = cvCreateImage(cvSize(OrigImg->width/2,OrigImg->height/2),IPL_DEPTH_8U, 1 );
    cvSet(dwnSample2,cvScalar(0));
    cvReleaseImage(&dwnSample4);
    dwnSample4 = cvCreateImage(cvSize(OrigImg->width/4,OrigImg->height/4),IPL_DEPTH_8U, 1 );
    cvSet(dwnSample2,cvScalar(0));
    cvReleaseImage(&dwnSample8);
    dwnSample8 = cvCreateImage(cvSize(OrigImg->width/8,OrigImg->height/8),IPL_DEPTH_8U, 1 );
    cvSet(dwnSample2,cvScalar(0));

    uchar* tmpOrigImg = (uchar*)OrigImg->imageData;
    uchar* origin = (uchar*)OrigImg->imageData;    
    uchar* tmpDwnImg2 = (uchar*)dwnSample2->imageData;    
    uchar* originDwn2 = (uchar*)dwnSample2->imageData;
    uchar* tmpDwnImg4 = (uchar*)dwnSample4->imageData;    
    uchar* originDwn4 = (uchar*)dwnSample4->imageData;
    uchar* tmpDwnImg8 = (uchar*)dwnSample8->imageData;    
    uchar* originDwn8 = (uchar*)dwnSample8->imageData;

    // actual width of images, including paddings    
    int origWidth = OrigImg->widthStep;
    int dwnWidth2 = dwnSample2->widthStep;
    int dwnWidth4 = dwnSample4->widthStep;
    int dwnWidth8 = dwnSample8->widthStep;

    int htUp = (OrigImg->height/8)*8;
    int wdUp = (OrigImg->width/8)*8;
    for(int i=0; i<htUp; ++i){
        for(int j=0; j<wdUp; ++j){
            //printf("i,j, %d,%d by fact %d,%d counter%d ",i,j,i/factor,j/factor,counter);
            *(tmpDwnImg2 + (i/2)*dwnWidth2 + j/2) += *tmpOrigImg/(4);
            *(tmpDwnImg4 + (i/4)*dwnWidth4 + j/4) += *tmpOrigImg/(16);
            *(tmpDwnImg8 + (i/8)*dwnWidth8 + j/8) += *tmpOrigImg/(64);
            tmpOrigImg++;
        }
        //jump to next row
        tmpOrigImg = origin + (i+1)*origWidth;
        
    }
                       
}

void visualFilterThread::upSampleMultiScales(IplImage* UpImg) {

    // assuming the destination image UpImg is prepared
    

    uchar* tmpOutImg = (uchar*)UpImg->imageData;
    uchar* originUpImg = (uchar*)UpImg->imageData;   
    uchar* tmpUpImg2 = (uchar*)dwnSample2->imageData;
    uchar* originUpImg2 = (uchar*)dwnSample2->imageData;
    uchar* tmpUpImg4 = (uchar*)dwnSample4->imageData;
    uchar* originUpImg4 = (uchar*)dwnSample4->imageData;
    uchar* tmpUpImg8 = (uchar*)dwnSample8->imageData;
    uchar* originUpImg8 = (uchar*)dwnSample8->imageData;
    
    // actual width of images, including paddings
    int UpWidth2 = dwnSample2->widthStep;
    int UpWidth4 = dwnSample4->widthStep;
    int UpWidth8 = dwnSample8->widthStep;
    int OrigWidth = UpImg->widthStep;

    float weight2 = .33;
    float weight4 = .33;
    float weight8 = .33;

    for(int i=0; i<UpImg->height; ++i){
        for(int j=0; j<UpImg->width; ++j){
            *tmpOutImg = weight2*(*(tmpUpImg2 + (i/2)*UpWidth2 + j/2))
                        + weight4*(*(tmpUpImg4 + (i/4)*UpWidth4 + j/4))
                        + weight8*(*(tmpUpImg8 + (i/8)*UpWidth8 + j/8));
            tmpOutImg++;
        }
        // jump to next row
        tmpOutImg = originUpImg + (i+1)*OrigWidth;        
    }

} 

void visualFilterThread::maxImages(IplImage** ImagesTobeAdded, int numberOfImages,IplImage* resultantImage, float* weights) {
    
    
    uchar* resImageOrigin = (uchar*)resultantImage->imageData;
    int resImageWidth = resultantImage->widthStep;
    uchar* tmpResultImage = (uchar*)resultantImage->imageData;
    for(int i=0; i< numberOfImages; ++i){
        IplImage* tmpImageTobeAdded = ImagesTobeAdded[i];
        if(tmpImageTobeAdded != 0){   // save yourself some pain    
            uchar* tmpImage = (uchar*)tmpImageTobeAdded->imageData;
            //Images must be same size, type etc
            int h = tmpImageTobeAdded->height;
            int w = tmpImageTobeAdded->width;
            if(h > resultantImage->height || w > resultantImage->width) {
                printf("Image too big. \n");
                return ;
            }
            float itsWt = weights[i];
            uchar* imgToBeAddOrigin = (uchar*)tmpImageTobeAdded->imageData;
            int imgToBeAddWidth = tmpImageTobeAdded->widthStep;
            for(int j=0; j<h; ++j){
                tmpImage = imgToBeAddOrigin + j* imgToBeAddWidth; 
                tmpResultImage = resImageOrigin + j*resImageWidth;
                for(int k=0; k<w; ++k){
                    unsigned char valNow = (unsigned char)(*tmpImage);
                    if(*tmpResultImage < valNow) *tmpResultImage = valNow; 
                    //*tmpResultImage = (unsigned char)(*tmpImage) * itsWt;
                    *tmpImage++;
                    *tmpResultImage++;
                }
            }
        }
    }

}

void visualFilterThread::addImages(IplImage** ImagesTobeAdded, int numberOfImages,IplImage* resultantImage, float* weights) {
    
    
    uchar* resImageOrigin = (uchar*)resultantImage->imageData;
    int resImageWidth = resultantImage->widthStep;
    uchar* tmpResultImage = (uchar*)resultantImage->imageData;
    for(int i=0; i< numberOfImages; ++i){
        IplImage* tmpImageTobeAdded = ImagesTobeAdded[i];
        if(tmpImageTobeAdded != NULL){   // save yourself some pain    
            uchar* tmpImage = (uchar*)tmpImageTobeAdded->imageData;
            //Images must be same size, type etc
            int h = tmpImageTobeAdded->height;
            int w = tmpImageTobeAdded->width;
            if(h > resultantImage->height || w > resultantImage->width) {
                printf("Image too big. \n");
                return ;
            }
            float itsWt = weights[i];
            uchar* imgToBeAddOrigin = (uchar*)tmpImageTobeAdded->imageData;
            int imgToBeAddWidth = tmpImageTobeAdded->widthStep;
            for(int j=0; j<h; ++j){
                tmpImage = imgToBeAddOrigin + j* imgToBeAddWidth; 
                tmpResultImage = resImageOrigin + j*resImageWidth;
                for(int k=0; k<w; ++k){
                    unsigned char valNow = (unsigned char)(*tmpImage)* itsWt;
                    //if(*tmpResultImage < valNow) *tmpResultImage = valNow;
                    if((valNow + *tmpResultImage) > 255) *tmpResultImage = 255;
                    else *tmpResultImage = *tmpResultImage + valNow ;
                    *tmpImage++;
                    *tmpResultImage++;
                }
            }
        }
    }

}


void visualFilterThread::openCVtoYARP(IplImage* originalImage, ImageOf<PixelMono>* yarpedImage, int type = 1) {

    //assuming everything is well
    yarpedImage->resize(originalImage->width,originalImage->height);
    if(type==1){
        uchar* originCVImage = (uchar*)originalImage->imageData;
        uchar* originYARPImage = yarpedImage->getRawImage();

        //padding might be different
        int widthCV = originalImage->widthStep;
        int widthY = yarpedImage->getRowSize();
        int minWid = widthCV>widthY? widthY:widthCV;
        for(int i=0; i<originalImage->height; ++i){
            uchar* ptrCV = originCVImage + i*widthCV;
            uchar* ptrY = originYARPImage + i*widthY;
            memcpy(ptrY,ptrCV,minWid);
        }
    }
}

/*
void visualFilterThread::getLinearlySeperableKernel(int sizeOfKernel,float* kernel, float* vertical, float* horizontal, float* pars){

    //Fetch kernel in Eigen matrix
    Eigen::MatrixXf Kernel(sizeOfKernel,sizeOfKernel);
    float* tmpKernel;
    tmpKernel = kernel;
    for(int i=0;i<sizeOfKernel;++i){
        for(int j=0;j<sizeOfKernel;++j){
            Kernel<<(*tmpKernel);
            tmpKernel++;
        }
    }

    //Declare horizontal and vertical vectors
    Eigen::VectorXf vert(sizeOfKernel);
    Eigen::VectorXf horz(sizeOfKernel);
    
    JacobiSVD<MatrixXf> svd(Kernel, ComputeThinU | ComputeThinV);
    float s = sqrt(svd.singularValues()(0));
    vert = svd.matrixU().cols(0)*s;
    horz = svd.matrixV().cols(0)*s;

    vertical = vert;
    horizontal = horz;   

} */

void visualFilterThread::convolve1D(int vecSize, float* vec, IplImage* img, IplImage* resImg, float factor,int direction){
    int ROIRowStart = vecSize/2;
    int ROIRowEnd = img->height-vecSize/2;
    int ROIColEnd = img->width - vecSize/2;
    int ROIColStart = vecSize/2;    
    int vecStart = -vecSize/2;
    int vecEnd = vecSize/2;
    int rowSize = img->widthStep;
    int resRowSize = resImg->widthStep;
    uchar* mat = (uchar*)img->imageData;
    uchar* res = (uchar*)resImg->imageData;
    float* midVec = vec + vecSize/2; // middle of linear kernel
    int pixelPos = 0; int pixPos =0;
    if(direction == 0){ //horizontal convolution
        for(int i=0;i<resImg->height;++i){
            for(int j=0;j<resImg->width;++j){
                pixelPos = i*resRowSize;
                //float* tmpVec = midVec;
                pixPos = j;
                float sum = *midVec * *(mat+pixelPos+pixPos);
                pixPos--;
                for(int k=0; k<vecSize/2 && pixPos>0; k++, pixPos--){
                    sum += (*(mat+pixelPos+pixPos))* (*(midVec-k));
                    //tmpVec++;
                }
                pixPos = j+1;
                for(int k=0; k<vecSize/2 && pixPos<img->width; k++, pixPos++){
                    sum += (*(mat+pixelPos+pixPos))* (*(midVec+k));
                    //tmpVec++;
                }
                *(res+i*resRowSize+j)=(unsigned char)sum*factor; // averaged sum
            }
        } 
    } 
    else {
        for(int i=0;i<resImg->height;++i){
            for(int j=0;j<resImg->width;++j){
                pixelPos = j;
                //float* tmpVec = midVec;
                pixPos = i;
                float sum = *midVec * *(mat+pixPos*rowSize+pixelPos);
                pixPos--;
                for(int k=0; k<vecSize/2 && pixPos>0; k++, pixPos--){
                    sum += (*(mat+pixelPos+pixPos*rowSize))* (*(midVec-k));
                    //tmpVec++;
                }
                pixPos = i+1;
                for(int k=0; k<vecSize/2 && pixPos<img->height; k++, pixPos++){
                    sum += (*(mat+pixelPos+pixPos*rowSize))* (*(midVec+k));
                    //tmpVec++;
                }
                *(res+i*resRowSize+j)=(unsigned char)sum*factor; // averaged sum
            }
        } 
    }  
                
        
}


void visualFilterThread::cropImage(int* corners, IplImage* imageToBeCropped, IplImage* retImage){

    // very lame cropping
    int imgWidth = corners[2]-corners[0];
    int imgHeight = corners[3]-corners[1];
    cvReleaseImage(&retImage);
    retImage = cvCreateImage(cvSize(imgWidth,imgHeight),IPL_DEPTH_8U, 1 );

    
    uchar* originDestImg = (uchar*)retImage->imageData;
    uchar* originSourImg = (uchar*)imageToBeCropped->imageData;
    int widthDest = retImage->widthStep;
    int widthSour = imageToBeCropped->widthStep;
    for(int i = 0; i<imgHeight; ++i){
        for(int j = 0; j<imgWidth; ++j){
           *(originDestImg+ i * widthDest + j) = *(originSourImg + (i+corners[1])*widthSour + j + corners[0]);
        }
    } 
    return ;

    // What if we could do something like below?? Answer: Yes but only when Sobel and other stuffs are ours and not standard ones since size and padding are now not intutive
    // in short many openCV assertions would fail!
    //retImage->imageData = imageToBeCropped->imageData;
    //retImage->width = imageToBeCropped->width;
    //retImage->widthStep = imageToBeCropped->widthStep; etc etc

    // Change the start of the pixels to be the small (internal) image region.
    /*
    imageToBeCropped->imageData = &imageToBeCropped->imageData[corners[0] * imageToBeCropped->nChannels + corners[1] * imageToBeCropped->widthStep];
    // Use the small image, but leave 'widthStep' to refer to the large image.
    imageToBeCropped->width = corners[2] - corners[0] ;
    imageToBeCropped->height = corners[3] - corners[1] ;
    imageToBeCropped->imageSize = imageToBeCropped->height * imageToBeCropped->widthStep;
    */ 
    
}



void visualFilterThread::threadRelease() {
    resized = false;
    /*
    if(cvRedPlane != NULL) cvReleaseImage(&cvRedPlane);
    if(cvGreenPlane != NULL) cvReleaseImage(&cvGreenPlane);
    if(cvBluePlane != NULL) cvReleaseImage(&cvBluePlane);
    if(cvYellowPlane != NULL) cvReleaseImage(&cvYellowPlane);
    if(cvRedPlus != NULL) cvReleaseImage(&cvRedPlus);
    if(cvRedMinus != NULL) cvReleaseImage(&cvRedMinus);
    if(cvGreenPlus != NULL) cvReleaseImage(&cvGreenPlus);
    if(cvGreenMinus != NULL) cvReleaseImage(&cvGreenMinus);
    if(cvBluePlus != NULL) cvReleaseImage(&cvBluePlus);
    if(cvYellowMinus != NULL) cvReleaseImage(&cvYellowMinus);
    if(redG != NULL) cvReleaseImage(&redG);
    if(greenR != NULL) cvReleaseImage(&greenR);
    if(blueY != NULL) cvReleaseImage(&blueY);
    if(hRG != NULL) cvReleaseImage(&hRG);
    if(vRG != NULL) cvReleaseImage(&vRG);
    if(hGR != NULL) cvReleaseImage(&hGR);
    if(vGR != NULL) cvReleaseImage(&vGR);
    if(hBY != NULL) cvReleaseImage(&hBY);
    if(vBY != NULL) cvReleaseImage(&vBY);
    if(tempHRG != NULL) cvReleaseImage(&tempHRG);
    if(tempVRG != NULL) cvReleaseImage(&tempVRG);
    if(tempHGR != NULL) cvReleaseImage(&tempHGR);
    if(tempVGR != NULL) cvReleaseImage(&tempVGR);
    if(tempHBY != NULL) cvReleaseImage(&tempHBY);
    if(tempVBY != NULL) cvReleaseImage(&tempVBY);
    if(dwnSample2 != NULL) cvReleaseImage(&dwnSample2);
    if(dwnSample4 != NULL) cvReleaseImage(&dwnSample4);
    if(dwnSample8 != NULL) cvReleaseImage(&dwnSample8);
    if(dwnSample2Fil != NULL) cvReleaseImage(&dwnSample2Fil);
    if(dwnSample4Fil != NULL) cvReleaseImage(&dwnSample4Fil);
    if(dwnSample8Fil != NULL) cvReleaseImage(&dwnSample8Fil);
    if(upSample2 != NULL) cvReleaseImage(&upSample2);
    if(upSample4 != NULL) cvReleaseImage(&upSample4);
    if(upSample8 != NULL) cvReleaseImage(&upSample8);
    if(intensityImage != NULL) cvReleaseImage(&intensityImage); 
    if(filteredIntensityImage != NULL) cvReleaseImage(&filteredIntensityImage);
    if(dwnImage != NULL) cvReleaseImage(&dwnImage);
    */
    
}

void visualFilterThread::onStop() {
    imagePortIn.interrupt();
    imagePortOut.interrupt();
    imagePortExt.interrupt();
    rgPort.interrupt();
    grPort.interrupt();
    byPort.interrupt();
    
    imagePortOut.close();
    imagePortExt.close();
    rgPort.close();
    grPort.close();
    byPort.close();
    imagePortIn.close();
}

