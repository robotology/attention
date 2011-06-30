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
#define ONE_BY_ROOT_THREE 0.577350269

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
static float Gab7V0[7] = { -0.0298,
                           -0.1703,
                           -0.4844,
                           -0.6863,
                           -0.4844,
                           -0.1703,
                           -0.0298
                        };
static float Gab7V45[7] = { -0.0298,
                           -0.1703,
                           -0.4844,
                           -0.6863,
                           -0.4844,
                           -0.1703,
                           -0.0298
                        };
static float Gab7V90[7] = { -0.0298,
                           -0.1703,
                           -0.4844,
                           -0.6863,
                           -0.4844,
                           -0.1703,
                           -0.0298
                        };
static float Gab7VM45[7] = { -0.0298,
                           -0.1703,
                           -0.4844,
                           -0.6863,
                           -0.4844,
                           -0.1703,
                           -0.0298
                        };

static float Gab7H0[7] = { -0.0001,   -0.0186,   -0.3354,   -0.8799,   -0.3354,   -0.0186,   -0.0001 };
static float Gab7H45[7] = { -0.0001,   -0.0186,   -0.3354,   -0.8799,   -0.3354,   -0.0186,   -0.0001 };
static float Gab7H90[7] = { -0.0001,   -0.0186,   -0.3354,   -0.8799,   -0.3354,   -0.0186,   -0.0001 };
static float Gab7HM45[7] = { -0.0001,   -0.0186,   -0.3354,   -0.8799,   -0.3354,   -0.0186,   -0.0001 };
/*
static float Gabor0[7][7]={0.000004 , 0.000507 , 0.009155 , 0.024018 , 0.009155 , 0.000507 , 0.000004 , 
0.000023 , 0.002895 , 0.052271 , 0.137134 , 0.052271 , 0.002895 , 0.000023 , 
0.000066 , 0.008234 , 0.148672 , 0.390039 , 0.148672 , 0.008234 , 0.000066 , 
0.000094 , 0.011666 , 0.210643 , 0.552621 , 0.210643 , 0.011666 , 0.000094 , 
0.000066 , 0.008234 , 0.148672 , 0.390039 , 0.148672 , 0.008234 , 0.000066 , 
0.000023 , 0.002895 , 0.052271 , 0.137134 , 0.052271 , 0.002895 , 0.000023 , 
0.000004 , 0.000507 , 0.009155 , 0.024018 , 0.009155 , 0.000507 , 0.000004 };
static float Gabor45[7][7]={0.034029 , 0.011588 , 0.000071 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.011588 , 0.160103 , 0.039997 , 0.000181 , 0.000000 , 0.000000 , 0.000000 , 
0.000071 , 0.039997 , 0.405435 , 0.074309 , 0.000246 , 0.000000 , 0.000000 , 
0.000000 , 0.000181 , 0.074309 , 0.552621 , 0.074309 , 0.000181 , 0.000000 , 
0.000000 , 0.000000 , 0.000246 , 0.074309 , 0.405435 , 0.039997 , 0.000071 , 
0.000000 , 0.000000 , 0.000000 , 0.000181 , 0.039997 , 0.160103 , 0.011588 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000071 , 0.011588 , 0.034029 };
static float Gabor90[7][7]={0.000004 , 0.000023 , 0.000066 , 0.000094 , 0.000066 , 0.000023 , 0.000004 , 
0.000507 , 0.002895 , 0.008234 , 0.011666 , 0.008234 , 0.002895 , 0.000507 , 
0.009155 , 0.052271 , 0.148672 , 0.210643 , 0.148672 , 0.052271 , 0.009155 , 
0.024018 , 0.137134 , 0.390039 , 0.552621 , 0.390039 , 0.137134 , 0.024018 , 
0.009155 , 0.052271 , 0.148672 , 0.210643 , 0.148672 , 0.052271 , 0.009155 , 
0.000507 , 0.002895 , 0.008234 , 0.011666 , 0.008234 , 0.002895 , 0.000507 , 
0.000004 , 0.000023 , 0.000066 , 0.000094 , 0.000066 , 0.000023 , 0.000004 };
static float GaborM45[7][7]={0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000071 , 0.011588 , 0.034029 , 
0.000000 , 0.000000 , 0.000000 , 0.000181 , 0.039997 , 0.160103 , 0.011588 , 
0.000000 , 0.000000 , 0.000246 , 0.074309 , 0.405435 , 0.039997 , 0.000071 , 
0.000000 , 0.000181 , 0.074309 , 0.552621 , 0.074309 , 0.000181 , 0.000000 , 
0.000071 , 0.039997 , 0.405435 , 0.074309 , 0.000246 , 0.000000 , 0.000000 , 
0.011588 , 0.160103 , 0.039997 , 0.000181 , 0.000000 , 0.000000 , 0.000000 , 
0.034029 , 0.011588 , 0.000071 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };
*/

/*float Gabor0[7][7]={0.000000 , 0.000000 , 0.000000 , 0.144106 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.822803 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 2.340231 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 3.315728 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 2.340231 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.822803 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.144106 , 0.000000 , 0.000000 , 0.000000 };*/
/*
float Gabor45[7][7]={0.204177 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.960616 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 2.432608 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 3.315728 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 2.432608 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.960616 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.204177 };

float Gabor90[7][7]={0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.144106 , 0.822803 , 2.340231 , 3.315728 , 2.340231 , 0.822803 , 0.144106 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };

float GaborM45[7][7]={0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.204177 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.960616 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 2.432608 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 0.000000 , 3.315728 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.000000 , 2.432608 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.000000 , 0.960616 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
0.204177 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 };
*/
float Gabor45[KERNEL_ROW][KERNEL_COL];
float Gabor90[KERNEL_ROW][KERNEL_COL];
float GaborM45[KERNEL_ROW][KERNEL_COL];
float Gabor0[KERNEL_ROW][KERNEL_COL];
/*={0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000001 , 0.000001 , 0.000001 , 0.000002 , 0.000002 , 0.000002 , 0.000001 , 0.000001 , 0.000001 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 
-0.000000 , -0.000000 , -0.000001 , -0.000002 , -0.000003 , -0.000005 , -0.000007 , -0.000010 , -0.000011 , -0.000012 , -0.000011 , -0.000010 , -0.000007 , -0.000005 , -0.000003 , -0.000002 , -0.000001 , -0.000000 , -0.000000 , 
0.000000 , 0.000001 , 0.000003 , 0.000005 , 0.000010 , 0.000016 , 0.000023 , 0.000031 , 0.000037 , 0.000039 , 0.000037 , 0.000031 , 0.000023 , 0.000016 , 0.000010 , 0.000005 , 0.000003 , 0.000001 , 0.000000 , 
-0.000001 , -0.000002 , -0.000005 , -0.000011 , -0.000021 , -0.000034 , -0.000050 , -0.000066 , -0.000079 , -0.000083 , -0.000079 , -0.000066 , -0.000050 , -0.000034 , -0.000021 , -0.000011 , -0.000005 , -0.000002 , -0.000001 , 
0.000001 , 0.000003 , 0.000008 , 0.000017 , 0.000031 , 0.000050 , 0.000074 , 0.000098 , 0.000116 , 0.000122 , 0.000116 , 0.000098 , 0.000074 , 0.000050 , 0.000031 , 0.000017 , 0.000008 , 0.000003 , 0.000001 , 
-0.000001 , -0.000003 , -0.000007 , -0.000014 , -0.000026 , -0.000042 , -0.000062 , -0.000082 , -0.000097 , -0.000103 , -0.000097 , -0.000082 , -0.000062 , -0.000042 , -0.000026 , -0.000014 , -0.000007 , -0.000003 , -0.000001 , 
-0.000000 , -0.000001 , -0.000002 , -0.000004 , -0.000008 , -0.000013 , -0.000019 , -0.000025 , -0.000029 , -0.000031 , -0.000029 , -0.000025 , -0.000019 , -0.000013 , -0.000008 , -0.000004 , -0.000002 , -0.000001 , -0.000000 , 
0.000003 , 0.000008 , 0.000018 , 0.000037 , 0.000068 , 0.000112 , 0.000165 , 0.000218 , 0.000258 , 0.000272 , 0.000258 , 0.000218 , 0.000165 , 0.000112 , 0.000068 , 0.000037 , 0.000018 , 0.000008 , 0.000003 , 
-0.000006 , -0.000015 , -0.000034 , -0.000070 , -0.000129 , -0.000213 , -0.000314 , -0.000414 , -0.000489 , -0.000517 , -0.000489 , -0.000414 , -0.000314 , -0.000213 , -0.000129 , -0.000070 , -0.000034 , -0.000015 , -0.000006 , 
0.000007 , 0.000018 , 0.000041 , 0.000084 , 0.000155 , 0.000256 , 0.000377 , 0.000498 , 0.000588 , 0.000622 , 0.000588 , 0.000498 , 0.000377 , 0.000256 , 0.000155 , 0.000084 , 0.000041 , 0.000018 , 0.000007 , 
-0.000006 , -0.000015 , -0.000034 , -0.000070 , -0.000129 , -0.000213 , -0.000314 , -0.000414 , -0.000489 , -0.000517 , -0.000489 , -0.000414 , -0.000314 , -0.000213 , -0.000129 , -0.000070 , -0.000034 , -0.000015 , -0.000006 , 
0.000003 , 0.000008 , 0.000018 , 0.000037 , 0.000068 , 0.000112 , 0.000165 , 0.000218 , 0.000258 , 0.000272 , 0.000258 , 0.000218 , 0.000165 , 0.000112 , 0.000068 , 0.000037 , 0.000018 , 0.000008 , 0.000003 , 
-0.000000 , -0.000001 , -0.000002 , -0.000004 , -0.000008 , -0.000013 , -0.000019 , -0.000025 , -0.000029 , -0.000031 , -0.000029 , -0.000025 , -0.000019 , -0.000013 , -0.000008 , -0.000004 , -0.000002 , -0.000001 , -0.000000 , 
-0.000001 , -0.000003 , -0.000007 , -0.000014 , -0.000026 , -0.000042 , -0.000062 , -0.000082 , -0.000097 , -0.000103 , -0.000097 , -0.000082 , -0.000062 , -0.000042 , -0.000026 , -0.000014 , -0.000007 , -0.000003 , -0.000001 , 
0.000001 , 0.000003 , 0.000008 , 0.000017 , 0.000031 , 0.000050 , 0.000074 , 0.000098 , 0.000116 , 0.000122 , 0.000116 , 0.000098 , 0.000074 , 0.000050 , 0.000031 , 0.000017 , 0.000008 , 0.000003 , 0.000001 , 
-0.000001 , -0.000002 , -0.000005 , -0.000011 , -0.000021 , -0.000034 , -0.000050 , -0.000066 , -0.000079 , -0.000083 , -0.000079 , -0.000066 , -0.000050 , -0.000034 , -0.000021 , -0.000011 , -0.000005 , -0.000002 , -0.000001 , 
0.000000 , 0.000001 , 0.000003 , 0.000005 , 0.000010 , 0.000016 , 0.000023 , 0.000031 , 0.000037 , 0.000039 , 0.000037 , 0.000031 , 0.000023 , 0.000016 , 0.000010 , 0.000005 , 0.000003 , 0.000001 , 0.000000 , 
-0.000000 , -0.000000 , -0.000001 , -0.000002 , -0.000003 , -0.000005 , -0.000007 , -0.000010 , -0.000011 , -0.000012 , -0.000011 , -0.000010 , -0.000007 , -0.000005 , -0.000003 , -0.000002 , -0.000001 , -0.000000 , -0.000000 , 
0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000001 , 0.000001 , 0.000001 , 0.000002 , 0.000002 , 0.000002 , 0.000001 , 0.000001 , 0.000001 , 0.000000 , 0.000000 , 0.000000 , 0.000000 , 0.000000  };
*/

template<class T>
inline T max(T a, T b, T c) {    
    if(b > a) a = b;
    if(c > a) a = c;
    return a;
}

visualFilterThread::visualFilterThread() {
    
    //inputExtImage       = new ImageOf<PixelRgb>;
    //inputImage          = new ImageOf<PixelRgb>;
    yImage              = new ImageOf<PixelMono>;
    yFilImage           = new ImageOf<PixelMono>;
    uvOrigImage         = new ImageOf<PixelMono>;
    uvImage             = new ImageOf<PixelMono>;
    uvFilImage          = new ImageOf<PixelMono>;
    inputImageFiltered  = new ImageOf<PixelRgb>;
    cartImage           = new ImageOf<PixelRgb>;    
    logPolarImage       = new ImageOf<PixelRgb>;
    
    edges               = new ImageOf<PixelMono>;    
    intensImg           = new ImageOf<PixelMono>;
    cartIntensImg       = new ImageOf<PixelMono>;
    
    redGreen            = new ImageOf<PixelMono>;
    greenRed            = new ImageOf<PixelMono>;
    blueYellow          = new ImageOf<PixelMono>;
    
    cartRedGreen        = new ImageOf<PixelMono>;
    cartGreenRed        = new ImageOf<PixelMono>;
    cartBlueYellow      = new ImageOf<PixelMono>;
    
    upSampleRGyarp      = new ImageOf<PixelMono>;
    upSampleGRyarp      = new ImageOf<PixelMono>;
    upSampleBYyarp      = new ImageOf<PixelMono>;

    gabor0              = new ImageOf<PixelMono>;
    gabor45             = new ImageOf<PixelMono>;
    gabor90             = new ImageOf<PixelMono>;
    gaborM45            = new ImageOf<PixelMono>;
    
    // Let us initialize IplImage pointers to NULL

    hRG =vRG=hGR=vGR=hBY=vBY= NULL;

    //16 bit image to avoid overflow in Sobel operator
    tempHRG=tempVRG=tempHGR=tempVGR=tempHBY=tempVBY=NULL;

    //down-sampled and up-sampled images for applying Gabor filter 
    dwnSampleRG=dwnSampleGR=dwnSampleBY=dwnSampleRGFil=dwnSampleGRFil=dwnSampleBYFil=upSampleRG=upSampleGR=upSampleBY=NULL;

    /******************/
    //down-sampled and up-sampled images for applying Gabor filter 
    dwnSampleRGa=dwnSampleGRa=dwnSampleBYa=dwnSampleRGFila=dwnSampleGRFila=dwnSampleBYFila=upSampleRGa=upSampleGRa=upSampleBYa=NULL;
    //down-sampled and up-sampled images for applying Gabor filter 
    dwnSampleRGb=dwnSampleGRb=dwnSampleBYb=dwnSampleRGFilb=dwnSampleGRFilb=dwnSampleBYFilb=upSampleRGb=upSampleGRb=upSampleBYb=NULL;
    

    intensityImage=filteredIntensityImage=filteredIntensityImage1=filteredIntensityImage2=totImage=dwnImage=NULL;

    tmpBlueYellow = tmpRedGreen = tmpGreenRed =NULL;
    
    //Default values for Gabor filter used
    /*intSigma = 800;
    sigma = 8;
    gLambda = 1.0/3.0; //3*sigma;//128;
    psi = 0;
    gamma = .25;
    kernelUsed = 2;
    dwnSam = 2;*/

    //default values
    /*for(int i=0;i<4;++i){
        intLambda[i] = 33;      // factor 100
        intSigma[i] = 80;      // factor 10
        intGamma[i] = 25;       // factor 100
        intPsi[i] = 0;          // factor 100
        intFilScale[i] =10;   // factor 10
        intFilShift[i] = 256;
    }*/

    intLambda[0] = 101;      // factor 100
    intSigma[0] = 105;      // factor 10
    intGamma[0] = 25;       // factor 100
    intPsi[0] = 0;          // factor 100
    intFilScale[0] =1;      // factor 10
    intFilShift[0] = 276;
    
    intLambda[1] = 101;      // factor 100
    intSigma[1] = 63;      // factor 10
    intGamma[1] = 25;       // factor 100
    intPsi[1] = 0;          // factor 100
    intFilScale[1] =1;   // factor 10
    intFilShift[1] = 244;
    
    intLambda[2] = 105;      // factor 100
    intSigma[2] = 105;      // factor 10
    intGamma[2] = 25;       // factor 100
    intPsi[2] = 0;          // factor 100
    intFilScale[2] =1;   // factor 10
    intFilShift[2] = 244;
    
    intLambda[3] = 84;      // factor 100
    intSigma[3] = 74;      // factor 10
    intGamma[3] = 25;       // factor 100
    intPsi[3] = 0;          // factor 100
    intFilScale[3] =1;   // factor 10
    intFilShift[3] = 251;
       

    orient0[0]=0;
    orient0[1]=255;
    
    orient45[0]=0;
    orient45[1]=255;

    orient90[0]=0;
    orient90[1]=255;

    orientM45[0]=0;
    orientM45[1]=255;

    

    getKernels();

          
    
    lambda = 0.3f;
    resized = false;

    //Logpolar to cartesian and vice versa
    xSizeValue = 320 ;         
    ySizeValue = 240;          // y dimension of the remapped cartesian image
    overlap = 1.0;         // overlap in the remapping
    numberOfRings = 152;      // number of rings in the remapping
    numberOfAngles = 252;     // number of angles in the remapping


    /********************/
    /*IplImage* nowT, *now0,*now45,*now90,*nowM45, *conv;
    nowT = cvCreateImage(cvSize(252,152),IPL_DEPTH_8U, 3 );
    now0 = cvCreateImage(cvSize(252,152),IPL_DEPTH_8U, 1 );
    now45 = cvCreateImage(cvSize(252,152),IPL_DEPTH_8U, 1 );
    now90 = cvCreateImage(cvSize(252,152),IPL_DEPTH_8U, 1 );
    nowM45 = cvCreateImage(cvSize(252,152),IPL_DEPTH_8U, 1 );
    conv  = cvCreateImage(cvSize(252,152),IPL_DEPTH_8U, 1 );
    nowT = cvLoadImage("/home/icub/Desktop/check.bmp");
    cvCvtColor( nowT, conv, CV_RGB2GRAY );
    convolve2D(KERNEL_ROW,KERNEL_COL,&Gabor0[0][0], conv, now0, 700.0,0); 
    convolve2D(KERNEL_ROW,KERNEL_COL,&Gabor45[0][0], conv, now45, 700.0,0); 
    convolve2D(KERNEL_ROW,KERNEL_COL,&Gabor90[0][0], conv, now90, 500.0,0); 
    convolve2D(KERNEL_ROW,KERNEL_COL,&GaborM45[0][0], conv, nowM45, 700.0,0); 
    cvNamedWindow("now0");
    cvShowImage("now0",now0);
    cvNamedWindow("now45");
    cvShowImage("now45",now45);
    cvNamedWindow("now90");
    cvShowImage("now90",now90);
    cvNamedWindow("nowM45");
    cvShowImage("nowM45",nowM45);
    cvNamedWindow("conv");
    cvShowImage("conv",conv);
    cvWaitKey(0);
    cvReleaseImage(&nowT);
    cvReleaseImage(&now0);
    cvReleaseImage(&now45);
    cvReleaseImage(&now90);
    cvReleaseImage(&nowM45);
    cvReleaseImage(&conv);*/
    
    
}

visualFilterThread::~visualFilterThread() {
    
    //delete inputExtImage;
    delete inputImageFiltered;
    //delete inputImage;
    delete yImage;
    delete yFilImage;
    delete uvImage;
    delete uvFilImage;
    delete uvOrigImage;
    delete cartImage;
    delete logPolarImage;
    

    delete edges;
    delete intensImg;
    delete cartIntensImg;

    delete redGreen;
    delete greenRed;
    delete blueYellow;

    delete cartRedGreen;
    delete cartGreenRed;
    delete cartBlueYellow;

    delete upSampleRGyarp;
    delete upSampleGRyarp;
    delete upSampleBYyarp;

    delete gabor0;
    delete gabor45;
    delete gabor90;
    delete gaborM45;

    printf("Called destructor \n");
    
    
}

bool visualFilterThread::threadInit() {
    printf("opening ports \n");
    /* open ports */ 
    

    if (!yPortIn.open(getName("/y:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!uvPortIn.open(getName("/uv:i").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    
    if (!imagePortExt.open(getName("/uvExt:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    
    if (!pyImgPort.open(getName("/edges:o").c_str())) {
        cout << ": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }

    // Opening ports for gabor-filtered images
    if (!gaborPort0.open(getName("/gabor0:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!gaborPort45.open(getName("/gabor45:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!gaborPort90.open(getName("/gabor90:o").c_str())) {
        cout <<": unable to open port "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
    if (!gaborPortM45.open(getName("/gaborM45:o").c_str())) {
        cout <<": unable to open port "  << endl;
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
        //inputImage  = imagePortIn.read(true);
        yImage      = yPortIn.read(true);
        uvOrigImage = uvPortIn.read(true);
        
        if (yImage != NULL && uvOrigImage != NULL && yImage->width() == uvOrigImage->width() 
            && yImage->height() == uvOrigImage->height() ) {
            if (!resized) {
                //printf("new image found %d %d", inputImage->width(),inputImage->height() );
                //resize(inputImage->width(), inputImage->height());
                resize(yImage->width(), yImage->height());
                //resize(uvOrigImage->width(), uvOrigImage->height());
                //int sizecvRedPlus = cvRedPlus->width;
    
                //printf("image successfull ");
                resized = true;
            }
            else {
                //filterInputImage();
            }            
            resizeCartesian(320,240);
            //printf("red plus dimension in resize1  %d %d \n", cvRedPlus->width, cvRedPlus->height);
             

            //filtering input image
            //printf("filtering \n");
            // filter y image
            filterInputImage();
             //printf("red plus dimension in resize2  %d %d \n", cvRedPlus->width, cvRedPlus->height);
 
            // extend logpolar input image
            //extender(inputImage, maxKernelSize);
            extender(uvOrigImage, maxKernelSize);
             //printf("red plus dimension in resize3  %d %d \n", cvRedPlus->width, cvRedPlus->height);
            
            // extract RGB and Y planes
            //extractPlanes();
             //printf("red plus dimension in resize4  %d %d \n", cvRedPlus->width, cvRedPlus->height);
                      
            // gaussian filtering of the of RGB and Y
            //filtering();
            

            // colourOpponency map construction
            //printf("before colour opponency \n");
            orientation();
            // apply sobel operators on the colourOpponency maps and combine via maximisation of the 3 edges

            
            edgesExtract();
        
            
            // sending the edge image on the outport            
            // the copy to the port object can be avoided...
            
            
            if((uvImage!=0)&&(imagePortExt.getOutputCount())) {
                imagePortExt.prepare() = *(uvImage);
                imagePortExt.write();
                }
            if((edges!=0)&&(pyImgPort.getOutputCount())) {
                pyImgPort.prepare() = *(edges);
                pyImgPort.write();
            }

            if((gabor0 != 0) && (gaborPort0.getOutputCount())) {
                gaborPort0.prepare() = *(gabor0);
                gaborPort0.write();
            }
            if((gabor45 != 0) && (gaborPort45.getOutputCount())) {
                gaborPort45.prepare() = *(gabor45);
                gaborPort45.write();
            }
            if((gabor90 != 0) && (gaborPort90.getOutputCount())) {
                gaborPort90.prepare() = *(gabor90);
                gaborPort90.write();
            }
            if((gaborM45 != 0) && (gaborPortM45.getOutputCount())) {
                gaborPortM45.prepare() = *(gaborM45);
                gaborPortM45.write();
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

    CvSize cvLogSize = cvSize(width, height);
    CvSize cvCropLogSize = cvSize(width_orig,height_orig);
    CvSize cvCartSize = cvSize(320,240);
    CvSize cvCartDwnSize = cvSize(320/DWN_SAMPLE,240/DWN_SAMPLE);
    CvSize cvCartDwnSize2 = cvSize(320/DWN_SAMPLE2,240/DWN_SAMPLE2);
    
    //printf("width after reposition %d %d \n", width , height);


    //resizing yarp image 
    redGreen->resize(width, height);
    greenRed->resize(width, height);
    blueYellow->resize(width, height);
    
    edges->resize(width_orig, height_orig);
    inputImageFiltered->resize(width_orig, height_orig);
    inputImageFiltered->zero();
    //inputExtImage->resize(width,height);

    uvOrigImage->resize(width_orig, height_orig);
    uvFilImage->resize(width_orig, height_orig);
    uvImage->resize(width,height);
    yFilImage->resize(width_orig, height_orig);
    yImage->resize(width,height);
    intensImg->resize(width,height);
    cartIntensImg->resize(320,240); 

    tmpRedGreen = cvCreateImage(cvCropLogSize,IPL_DEPTH_8U, 1 );
    tmpGreenRed = cvCreateImage(cvCropLogSize,IPL_DEPTH_8U, 1 );
    tmpBlueYellow = cvCreateImage(cvCropLogSize,IPL_DEPTH_8U, 1 );   

    cartRedGreen->resize(320, 240);    
    cartGreenRed->resize(320, 240);    
    cartBlueYellow->resize(320, 240);

    upSampleRGyarp->resize(320, 240);  
    upSampleGRyarp->resize(320, 240);  
    upSampleBYyarp->resize(320, 240); 

 
    
    //allocate IplImages for color planes
    cvRedPlane = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvGreenPlane = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvBluePlane = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvYellowPlane = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );

    //allocate IplImages for color opponents
    redG = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
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

    cvYellowMinus   = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
    cvBluePlus      = cvCreateImage(cvLogSize,IPL_DEPTH_8U, 1 );
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

    cvGabor0 = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );
    cvGabor45 = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );
    cvGabor90 = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );
    cvGaborM45 = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );

    gabor0->resize(320,240);
    gabor45->resize(320,240);
    gabor90->resize(320,240);
    gaborM45->resize(320,240);
    
    intensityImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    filteredIntensityImage = cvCreateImage(cvCropLogSize,IPL_DEPTH_8U, 1 );
    filteredIntensityImage1 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    filteredIntensityImage2 = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    totImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnImage = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 ); // this is later resized
    dwnSampleRG  = cvCreateImage(cvCartDwnSize,IPL_DEPTH_8U, 1 );
    dwnSampleGR  = cvCreateImage(cvCartDwnSize2,IPL_DEPTH_8U, 1 );  // CHANGE!
    dwnSampleBY  = cvCreateImage(cvCartDwnSize,IPL_DEPTH_8U, 1 );
    upSampleRG   = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );
    upSampleGR = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );
    upSampleBY = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );

    
    dwnSampleRGa = cvCreateImage(cvCartDwnSize,IPL_DEPTH_8U, 1 );
    dwnSampleGRa = cvCreateImage(cvCartDwnSize2,IPL_DEPTH_8U, 1 );  // CHANGE!
    dwnSampleBYa = cvCreateImage(cvCartDwnSize,IPL_DEPTH_8U, 1 );
    upSampleRGa = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );
    upSampleGRa = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );
    upSampleBYa = cvCreateImage(cvCartSize,IPL_DEPTH_8U, 1 );

    /*dwnSampleRGb = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSampleGRb = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    dwnSampleBYb = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSampleRGb = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSampleGRb = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );
    upSampleBYb = cvCreateImage(cvSize(width,height),IPL_DEPTH_8U, 1 );*/ 

    cvNamedWindow("Parameters for Gabor Filters",CV_GUI_NORMAL);
    cvResizeWindow("Parameters for Gabor Filters",600,400);
    cvNamedWindow("Parameters for Gabor Filters1",CV_GUI_NORMAL);
    cvResizeWindow("Parameters for Gabor Filters1",600,400);
    cvNamedWindow("Parameters for Scaling Images",CV_GUI_NORMAL);
    cvResizeWindow("Parameters for Scaling Images",600,400);
    
    cvCreateTrackbar("Sig 0", "Parameters for Gabor Filters", &intSigma[0], 1000, NULL); // no callback
    cvCreateTrackbar("Sig 45", "Parameters for Gabor Filters", &intSigma[1], 1000, NULL); // no callback
    cvCreateTrackbar("Sig 90", "Parameters for Gabor Filters", &intSigma[2], 1000, NULL); // no callback
    cvCreateTrackbar("Sig -45", "Parameters for Gabor Filters", &intSigma[3], 1000, NULL); // no callback

    cvCreateTrackbar("Gam 0", "Parameters for Gabor Filters", &intGamma[0], 1000, NULL); // no callback
    cvCreateTrackbar("Gam 45", "Parameters for Gabor Filters", &intGamma[1], 1000, NULL); // no callback
    cvCreateTrackbar("Gam 90", "Parameters for Gabor Filters", &intGamma[2], 1000, NULL); // no callback
    cvCreateTrackbar("Gam -45", "Parameters for Gabor Filters", &intGamma[3], 1000, NULL); // no callback

    cvCreateTrackbar("Psi 0", "Parameters for Gabor Filters1", &intPsi[0], 1000, NULL); // no callback
    cvCreateTrackbar("Psi 45", "Parameters for Gabor Filters1", &intPsi[1], 1000, NULL); // no callback
    cvCreateTrackbar("Psi 90", "Parameters for Gabor Filters1", &intPsi[2], 1000, NULL); // no callback
    cvCreateTrackbar("Psi -45", "Parameters for Gabor Filters1", &intPsi[3], 1000, NULL); // no callback

    cvCreateTrackbar("Lambda 0", "Parameters for Gabor Filters1", &intLambda[0], 1000, NULL); // no callback
    cvCreateTrackbar("Lambda 45", "Parameters for Gabor Filters1", &intLambda[1], 1000, NULL); // no callback
    cvCreateTrackbar("Lambda 90", "Parameters for Gabor Filters1", &intLambda[2], 1000, NULL); // no callback
    cvCreateTrackbar("Lambda -45", "Parameters for Gabor Filters1", &intLambda[3], 1000, NULL); // no callback

    cvCreateTrackbar("Scale 0", "Parameters for Gabor Filters", &intFilScale[0], 100, NULL); // no callback
    cvCreateTrackbar("Scale 45", "Parameters for Gabor Filters", &intFilScale[1], 100, NULL); // no callback
    cvCreateTrackbar("Scale 90", "Parameters for Gabor Filters", &intFilScale[2], 100, NULL); // no callback
    cvCreateTrackbar("Scale -45", "Parameters for Gabor Filters", &intFilScale[3], 100, NULL); // no callback
    
    cvCreateTrackbar("Shift 0", "Parameters for Gabor Filters1", &intFilShift[0], 512, NULL); // no callback
    cvCreateTrackbar("Shift 45", "Parameters for Gabor Filters1", &intFilShift[1], 512, NULL); // no callback
    cvCreateTrackbar("Shift 90", "Parameters for Gabor Filters1", &intFilShift[2], 512, NULL); // no callback
    cvCreateTrackbar("Shift -45", "Parameters for Gabor Filters1", &intFilShift[3], 512, NULL); // no callback
    
    cvCreateTrackbar("Max Neg 0", "Parameters for Scaling Images", &orient0[0], 10000, NULL); // no callback
    cvCreateTrackbar("Max Neg 45", "Parameters for Scaling Images", &orient45[0], 10000, NULL); // no callback
    cvCreateTrackbar("Max Neg 90", "Parameters for Scaling Images", &orient90[0], 10000, NULL); // no callback
    cvCreateTrackbar("Max Neg M45", "Parameters for Scaling Images", &orientM45[0], 10000, NULL); // no callback
    
    cvCreateTrackbar("Max 0", "Parameters for Scaling Images", &orient0[1], 10000, NULL); // no callback
    cvCreateTrackbar("Max 45", "Parameters for Scaling Images", &orient45[1], 10000, NULL); // no callback
    cvCreateTrackbar("Max 90", "Parameters for Scaling Images", &orient90[1], 10000, NULL); // no callback
    cvCreateTrackbar("Max M45", "Parameters for Scaling Images", &orientM45[1], 10000, NULL); // no callback
    
    
    
   
    
}

void visualFilterThread::filterInputImage() {
    int i;
    //const int szY = yImage->getRawImageSize();
    const int szUV = uvOrigImage->getRawImageSize();
    unsigned char * pFilteredY = yFilImage->getRawImage();
    unsigned char * pCurrY = yImage->getRawImage();
    unsigned char * pFilteredUV = uvFilImage->getRawImage();
    unsigned char * pCurrUV = uvOrigImage->getRawImage();
    const float ul = 1.0f - lambda;
    for (i = 0; i < szUV; i++) { // assuming same size
        *pFilteredUV = (unsigned char)(lambda * *pCurrUV++ + ul * *pFilteredUV++ + .5f);
        *pFilteredY = (unsigned char)(lambda * *pCurrY++ + ul * *pFilteredY++ + .5f);
        
    }
}
/*
ImageOf<PixelRgb>* visualFilterThread::extender(ImageOf<PixelRgb>* inputOrigImage, int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*inputExtImage, *inputOrigImage, maxSize);
    return inputExtImage;
}
*/

ImageOf<PixelMono>* visualFilterThread::extender(ImageOf<PixelMono>* uvOrigImage, int maxSize) {
    iCub::logpolar::replicateBorderLogpolar(*uvImage, *uvFilImage, maxSize);
    return uvImage;
}

void visualFilterThread::cartremap(ImageOf<PixelRgb>* cartesianImage, ImageOf<PixelRgb>* logpolarImage) {
    trsf.logpolarToCart (*cartesianImage, *logpolarImage);
}

void visualFilterThread::extractPlanes() {

/*  ********* Depricated ***************
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
    uchar* ptrIntensityImg = (uchar*) intensityImage->imageData;

    // Pointer to raw extended input (RGB) image
    inputPointer = inputExtImage->getRawImage();
    uchar* originInputImage = inputPointer;
    uchar* originCVRedPlane = (uchar*) cvRedPlane->imageData;
    uchar* originCVGreenPlane = (uchar*) cvGreenPlane->imageData;
    uchar* originCVBluePlane = (uchar*) cvBluePlane->imageData;
    uchar* originCVYellowPlane = (uchar*) cvYellowPlane->imageData;
    uchar* originCVIntensityImg = (uchar*)intensityImage->imageData;
    int widthInputImage = inputExtImage->getRowSize();
    int widthcvR = cvRedPlane->widthStep;
    int widthcvG = cvGreenPlane->widthStep;
    int widthcvB = cvBluePlane->widthStep;
    int widthcvY = cvYellowPlane->widthStep;
    int widthcvI = intensityImage->widthStep;

    // We can avoid padding for openCV images
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
        ptrIntensityImg = originCVIntensityImg + r*widthcvI;
        
        for(int c = 0; c < w; c++) {
            *shift[0] = *inputPointer++;
            *shift[1] = *inputPointer++;
            *shift[2] = *inputPointer++;

            *yellowP++ = (unsigned char)((*shift[0] >> 1) + (*shift[1] >> 1));
            *ptrIntensityImg++ = ONE_BY_ROOT_THREE * sqrt(*shift[0] * *shift[0] +*shift[1] * *shift[1] +*shift[2] * *shift[2]);
            shift[0]++;
            shift[1]++;
            shift[2]++;
        }

        
    }
*/
}

void visualFilterThread::filtering() {
    // We gaussian blur the image planes extracted before, one with positive Gaussian and then negative
    
    //Positive
    convolve1D(5,G5,cvRedPlane,tmpRedPlus,.5,0);
    convolve1D(5,G5,tmpRedPlus,cvRedPlus,.5,1);

    convolve1D(5,G5,cvGreenPlane,tmpGreenPlus,.5,0);
    convolve1D(5,G5,tmpGreenPlus,cvGreenPlus,.5,1);

    convolve1D(5,G5,cvBluePlane,tmpBluePlus,.5,0);
    convolve1D(5,G5,tmpBluePlus,cvBluePlus,.5,1);
   
    
    
    //Negative
    convolve1D(7,G7,cvRedPlane,tmpRedMinus,.5,0);
    convolve1D(7,G7,tmpRedMinus,cvRedMinus,.5,1);

    convolve1D(7,G7,cvGreenPlane,tmpGreenMinus,.5,0);
    convolve1D(7,G7,tmpGreenMinus,cvGreenMinus,.5,1);

    convolve1D(7,G7,cvYellowPlane,tmpYellowMinus,.5,0);
    convolve1D(7,G7,tmpYellowMinus,cvYellowMinus,.5,1);

    int crn[4]={5,5,252,152};
    
    

    /*cvNamedWindow("intensityImage");
    cvShowImage("intensityImage",intensityImage);
    cvNamedWindow("test2");
    cvShowImage("test2",cvRedMinus);    
    cvWaitKey(0);*/
    

    
    
}

void visualFilterThread::orientation() {
    
    // we want RG = (G- - R+)/2 -128, GR = (R- - G+)/2 and BY = (Y- -B+)/2 -128. These values are obtained after filtering with positive and negative Gaussian.

    const int h = height;
    const int w = width;
    
/*
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

 
    // Cropping the extended part of these log-polar color opponency maps. LATER: Can be accomodated inside the loop above.
    
    
    int cor[4]={5,5,257,157};
    cropImage(cor,redG,tmpRedGreen);
    cropImage(cor,greenR,tmpGreenRed);
    cropImage(cor,blueY,tmpBlueYellow);

    cropImage(cor,intensityImage,filteredIntensityImage);

    redGreen->resize(252,152);
    greenRed->resize(252,152);
    blueYellow->resize(252,152);
    
    cropImage(cor,redG,(IplImage*)redGreen->getIplImage());
    cropImage(cor,greenR,(IplImage*)greenRed->getIplImage());
    cropImage(cor,blueY,(IplImage*)blueYellow->getIplImage());

   
    
    // Preparing to send these openCV images to YARP port
    //printf("Size of YARP rG, gR, bY %d,%d : %d,%d : %d,%d \n",redGreen->width(),redGreen->height(),greenRed->width(),greenRed->height(),blueYellow->width(),blueYellow->height());
    //printf("Size of openCV rG, gR, bY %d,%d : %d,%d : %d,%d \n",tmpRedGreen->width,tmpRedGreen->height,tmpGreenRed->width,tmpGreenRed->height,tmpBlueYellow->width,tmpBlueYellow->height);
    
    redGreen->zero();
    openCVtoYARP(tmpRedGreen,redGreen,1);
    intensImg->zero();
    openCVtoYARP(filteredIntensityImage,intensImg,1);    
    greenRed->zero();
    openCVtoYARP(tmpGreenRed,greenRed,1);
    blueYellow->zero();
    openCVtoYARP(tmpBlueYellow,blueYellow,1);

*/
    
       
    // Converting regular R+G- to cartesian , similarly for others
    //lpMono.logpolarToCart (*cartRedGreen, *redGreen);
    //lpMono.logpolarToCart (*cartGreenRed, *greenRed);
    //lpMono.logpolarToCart (*cartBlueYellow, *blueYellow);
    //lpMono.logpolarToCart (*cartIntensImg, *intensImg);
    lpMono.logpolarToCart (*cartRedGreen, *yImage);
    //lpMono.logpolarToCart (*cartGreenRed, *uvImage);
    
    
    float weight[3]= {.33,.33,.33};

    // Downsample these cartesian color opponency maps to scale 4
    //int DWN_SAMPLE = 4;
    //downSampleImage((IplImage*)cartRedGreen->getIplImage(), dwnSampleRGa,DWN_SAMPLE);
    downSampleImage((IplImage*)cartRedGreen->getIplImage(), dwnSampleRGa,DWN_SAMPLE);
    //downSampleImage((IplImage*)cartGreenRed->getIplImage(), dwnSampleGRa,DWN_SAMPLE);
    downSampleImage((IplImage*)cartRedGreen->getIplImage(), dwnSampleGRa,DWN_SAMPLE2);
    

    
    
    // Some local tmp images allocated
    dwnSampleRGFila = cvCreateImage(cvGetSize(dwnSampleRGa),IPL_DEPTH_8U, 1 );
    dwnSampleGRFila = cvCreateImage(cvGetSize(dwnSampleGRa),IPL_DEPTH_8U, 1 );
    

    

    cvShowImage("Parameters for Gabor Filters",NULL);
    cvShowImage("Parameters for Gabor Filters1",NULL);
    cvWaitKey(20);
    getKernels();

    float scaleFactor = 100;
    int shift = 0;
    /**********************************************************************************************/
    /*     For orientation 0 degrees                                                              */
    /**********************************************************************************************/

    
   /* 
    convolve1D(7,Gab7H0,dwnSampleRGa,tmpdwnSampleRGFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7V0,tmpdwnSampleRGFil,dwnSampleRGFila,scaleFactor,0,1); // convolve with vertical
  
    convolve1D(7,Gab7H0,dwnSampleGRa,tmpdwnSampleGRFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7V0,tmpdwnSampleGRFil,dwnSampleGRFila,scaleFactor,0,1); // convolve with vertical 
 
    convolve1D(7,Gab7H0,dwnSampleBYa,tmpdwnSampleBYFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7V0,tmpdwnSampleBYFil,dwnSampleBYFila,scaleFactor,0,1);// convolve with vertical 

    */

    
 
    convolve2D(KERNEL_ROW,KERNEL_COL,&Gabor0[0][0], dwnSampleRGa, dwnSampleRGFila, scaleFactor,intFilShift[0],orient0);  
    convolve2D(KERNEL_ROW,KERNEL_COL,&Gabor0[0][0], dwnSampleGRa, dwnSampleGRFila, scaleFactor,intFilShift[0],orient0);  
    
    
    
    // Removing the pixels just outside the circle that get corrupted due to filtering. LATER: This can be taken care of during filtering.
    int centerImg0[2] = {dwnSampleRGFila->height/2, dwnSampleRGFila->width/2};
    int centerImg20[2] = {dwnSampleGRFila->height/2, dwnSampleGRFila->width/2};
    cropCircleImage(centerImg0,dwnSampleRGFila->height/2,dwnSampleRGFila);
    cropCircleImage(centerImg20,dwnSampleGRFila->height/2,dwnSampleGRFila);
    

    //up-sample the filtered images
    upSampleImage(dwnSampleRGFila,upSampleRGa,DWN_SAMPLE);
    upSampleImage(dwnSampleGRFila,upSampleGRa,DWN_SAMPLE2);
    

    // Add these 3 color opponent maps. We may take max when adding
    float wt[3]={.33,.33,.33};
    IplImage* imgs2Add0[2]={upSampleRGa,upSampleGRa};
    maxImages(imgs2Add0,2,cvGabor0);

    

     

    /**********************************************************************************************/
    /*     For orientation 45 degrees                                                              */
    /**********************************************************************************************/

/*
    convolve1D(7,Gab7H45,dwnSampleRGa,tmpdwnSampleRGFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7V45,tmpdwnSampleRGFil,dwnSampleRGFila,scaleFactor,0,1); // convolve with vertical
  
    convolve1D(7,Gab7H45,dwnSampleGRa,tmpdwnSampleGRFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7V45,tmpdwnSampleGRFil,dwnSampleGRFila,scaleFactor,0,1); // convolve with vertical 
 
    convolve1D(7,Gab7H45,dwnSampleBYa,tmpdwnSampleBYFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7V45,tmpdwnSampleBYFil,dwnSampleBYFila,scaleFactor,0,1);// convolve with vertical  

*/
    shift = 0;
    
    convolve2D(KERNEL_ROW,KERNEL_COL,&Gabor45[0][0], dwnSampleRGa, dwnSampleRGFila, 1.0*scaleFactor,intFilShift[1],orient45);  
    convolve2D(KERNEL_ROW,KERNEL_COL,&Gabor45[0][0], dwnSampleGRa, dwnSampleGRFila, 1.0*scaleFactor,intFilShift[1],orient45);  
    
    shift =0;

    /*cvNamedWindow("test4");
    cvShowImage("test4",dwnSampleRGa);
    cvNamedWindow("test5");
    cvShowImage("test5",dwnSampleRGFila);
    cvNamedWindow("test6");
    cvShowImage("test6",cvGaborM45);
    cvWaitKey(0);*/
    
     
     

    // Removing the pixels just outside the circle that get corrupted due to filtering. LATER: This can be taken care of during filtering.
    int centerImg45[2] = {dwnSampleRGFila->height/2, dwnSampleRGFila->width/2};
    int centerImg245[2] = {dwnSampleGRFila->height/2, dwnSampleGRFila->width/2};
    cropCircleImage(centerImg45,dwnSampleRGFila->height/2,dwnSampleRGFila);
    cropCircleImage(centerImg245,dwnSampleGRFila->height/2,dwnSampleGRFila);
   

    //up-sample the filtered images
    upSampleImage(dwnSampleRGFila,upSampleRGa,DWN_SAMPLE);
    upSampleImage(dwnSampleGRFila,upSampleGRa,DWN_SAMPLE2);

    // Add these 3 color opponent maps. We may take max when adding
    //float wt[3]={.33,.33,.33};
    IplImage* imgs2Add45[2]={upSampleRGa,upSampleGRa };
    maxImages(imgs2Add45,2,cvGabor45); 

    
    /**********************************************************************************************/
    /*     For orientation 90 degrees                                                              */
    /**********************************************************************************************/

/*
    convolve1D(7,Gab7H90,dwnSampleRGa,tmpdwnSampleRGFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7V90,tmpdwnSampleRGFil,dwnSampleRGFila,scaleFactor,0,1); // convolve with vertical
  
    convolve1D(7,Gab7H90,dwnSampleGRa,tmpdwnSampleGRFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7V90,tmpdwnSampleGRFil,dwnSampleGRFila,scaleFactor,0,1); // convolve with vertical 
 
    convolve1D(7,Gab7H90,dwnSampleBYa,tmpdwnSampleBYFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7V90,tmpdwnSampleBYFil,dwnSampleBYFila,scaleFactor,0,1);// convolve with vertical    
  */
    
    convolve2D(KERNEL_ROW,KERNEL_COL,&Gabor90[0][0], dwnSampleRGa, dwnSampleRGFila, scaleFactor,intFilShift[2],orient90);  
    convolve2D(KERNEL_ROW,KERNEL_COL,&Gabor90[0][0], dwnSampleGRa, dwnSampleGRFila, scaleFactor,intFilShift[2],orient90);  
      

    // Removing the pixels just outside the circle that get corrupted due to filtering. LATER: This can be taken care of during filtering.
    int centerImg90[2] = {dwnSampleRGFila->height/2, dwnSampleRGFila->width/2};
    int centerImg290[2] = {dwnSampleGRFila->height/2, dwnSampleGRFila->width/2};
    cropCircleImage(centerImg90,dwnSampleRGFila->height/2,dwnSampleRGFila);
    cropCircleImage(centerImg290,dwnSampleGRFila->height/2,dwnSampleGRFila);

    //up-sample the filtered images
    upSampleImage(dwnSampleRGFila,upSampleRGa,DWN_SAMPLE);
    upSampleImage(dwnSampleGRFila,upSampleGRa,DWN_SAMPLE2);

    // Add these 3 color opponent maps. We may take max when adding
    //float wt[3]={.33,.33,.33};
    IplImage* imgs2Add90[2]={upSampleRGa,upSampleGRa};
    maxImages(imgs2Add90,2,cvGabor90); 

    
    /**********************************************************************************************/
    /*     For orientation minus 45 degrees                                                              */
    /**********************************************************************************************/

/*
    convolve1D(7,Gab7HM45,dwnSampleRGa,tmpdwnSampleRGFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7VM45,tmpdwnSampleRGFil,dwnSampleRGFila,scaleFactor,0,1); // convolve with vertical
  
    convolve1D(7,Gab7HM45,dwnSampleGRa,tmpdwnSampleGRFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7VM45,tmpdwnSampleGRFil,dwnSampleGRFila,scaleFactor,0,1); // convolve with vertical 
 
    convolve1D(7,Gab7HM45,dwnSampleBYa,tmpdwnSampleBYFil,scaleFactor,0,0); // convolve with horizontal 
    convolve1D(7,Gab7VM45,tmpdwnSampleBYFil,dwnSampleBYFila,scaleFactor,0,1);// convolve with vertical    
 */
    
    convolve2D(KERNEL_ROW,KERNEL_COL,&GaborM45[0][0], dwnSampleRGa, dwnSampleRGFila, scaleFactor,intFilShift[3],orientM45);  
    convolve2D(KERNEL_ROW,KERNEL_COL,&GaborM45[0][0], dwnSampleGRa, dwnSampleGRFila, scaleFactor,intFilShift[3],orientM45);

         

    // Removing the pixels just outside the circle that get corrupted due to filtering. LATER: This can be taken care of during filtering.
    int centerImgM45[2] = {dwnSampleRGFila->height/2, dwnSampleRGFila->width/2};
    int centerImgM245[2] = {dwnSampleGRFila->height/2, dwnSampleGRFila->width/2};
    cropCircleImage(centerImgM45,dwnSampleRGFila->height/2,dwnSampleRGFila);
    cropCircleImage(centerImgM245,dwnSampleGRFila->height/2,dwnSampleGRFila);

    //up-sample the filtered images
    upSampleImage(dwnSampleRGFila,upSampleRGa,DWN_SAMPLE);
    upSampleImage(dwnSampleGRFila,upSampleGRa,DWN_SAMPLE2);

    // Add these 3 color opponent maps. We may take max when adding
    //float wt[3]={.33,.33,.33};
    IplImage* imgs2AddM45[2]={upSampleRGa,upSampleGRa };
    maxImages(imgs2AddM45,2,cvGaborM45); 

    
    /********** End of oriented gabor filtering  *********************************/

    
    
    //Preparing the upsampled image to be sent to port
    openCVtoYARP(upSampleRGa,upSampleRGyarp,1);
    openCVtoYARP(upSampleGRa,upSampleGRyarp,1);
    

    openCVtoYARP(cvGabor0,gabor0,1);
    openCVtoYARP(cvGabor45,gabor45,1);
    openCVtoYARP(cvGabor90,gabor90,1);
    openCVtoYARP(cvGaborM45,gaborM45,1);

    /*cvNamedWindow("test4");
    cvShowImage("test4",(IplImage*)gabor0->getIplImage());
    cvNamedWindow("test5");
    cvShowImage("test5",cvGabor45);
    cvNamedWindow("test6");
    cvShowImage("test6",cvGaborM45);
    cvWaitKey(0);*/
    

    // local tmp images freed
    cvReleaseImage(&dwnSampleRGFila);
    cvReleaseImage(&dwnSampleGRFila);
    
       

    
  
}



void visualFilterThread::edgesExtract() {

    /*
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

    */
    cvSobel((IplImage*)uvImage->getIplImage(),tempHRG,1,0,3);   
    cvSobel((IplImage*)uvImage->getIplImage(),tempVRG,0,1,3);

    // Scaling the destination back to 8 bit
    cvConvertScaleAbs(tempHRG,hRG,.2,0.0);
    cvConvertScaleAbs(tempVRG,vRG,.2,0.0);

    //clearing up the previous value
    edges->zero();
    uchar* pedges=edges->getRawImage();
    const int pad_edges = edges->getPadding();

    // can be optimized later
    uchar* originPedges = edges->getRawImage();
    int widthEdges = edges->getRowSize();
    uchar* originHRG = (uchar*)hRG->imageData + hRG->widthStep * (height - height_orig);
    uchar* originVRG = (uchar*)vRG->imageData + vRG->widthStep * (height - height_orig);
    int widthStepCV = hRG->widthStep;

    uchar* ptrHRG = (uchar*)hRG->imageData;
    uchar* ptrVRG = (uchar*)vRG->imageData;
    
    //The extra portion of extended image can be neglected, hence the pointers are shifted accordingly. 
    int j = (hRG->widthStep)*(height-height_orig)+ (width-width_orig)/2; 

    // At the end of the original width, extended image pointers need to jump more 
    int gapInWidth = (width - width_orig)/2;
    const int padXtnd = gapInWidth + (hRG->widthStep - hRG->width);

    ptrHRG += j; ptrVRG += j;
    

    for (int row = 0; row < height_orig; row++) {
        for (int col = 0; col < width_orig; col++) {

            double rg = (*ptrHRG ) * (*ptrHRG ) + (*ptrVRG ) * (*ptrVRG );
            if (row < height_orig) {
                *pedges = (unsigned char)(sqrt(rg)); //*(255.0 / 1024)); //normalised with theoric max-response 1448.16, 362.03                
            }
            else
                *pedges = 0;
            
            pedges++;
            ptrHRG++; ptrVRG++;
            
        }
        // padding
        pedges += pad_edges;
        //ptrHRG += padXtnd; ptrVRG += padXtnd;
        //ptrHGR += padXtnd; ptrVGR += padXtnd;
        //ptrHBY += padXtnd; ptrVBY += padXtnd;
        ptrHRG = originHRG + (row+1)*widthStepCV + gapInWidth;
        ptrVRG = originVRG + (row+1)*widthStepCV + gapInWidth;        
    }

    
    
    
    
}

void visualFilterThread::setPar(int par, double value) {
   /* if(par == 1) this->sigma = value;
    else if(par == 2 ) this->gLambda = value;
    else if(par == 3) this->psi = value;
    else if(par == 4) this->gamma = value;
    else if(par == 5) this->kernelUsed = value;
    else if(par == 6) this->dwnSam = value;
    else if(par == 7) this->whichScale = value;
    getKernels();*/
}

void visualFilterThread::getKernels() {
    
    double theta[4] = {0, PI/4.0, PI/2.0, -1.0*PI/4.0};    
        
    for(int i = 0; i < 4 ; ++i) {


        psi[i] = intPsi[i]*.01;
        gLambda[i] = intLambda[i]*.01;
        sigma[i] = intSigma[i]*.1;
        gamma[i] = intGamma[i]*.01;
        filScale[i] = intFilScale[i]*.1;

        double sigmaX   = sigma[i]; 
        double sigmaY   = sigma[i]/gamma[i];
        double sigmaGrt = sigmaX>sigmaY ? sigmaX : sigmaY;
        double filterSize = KERNEL_ROW; // x and y
        double fact     = 1.0/(gLambda[i]*filterSize);//(6*sigmaGrt+1));
        int stdDev = 3;     
        double xLim = max<double>(1.0,max<double>(abs(cos(theta[i])*sigmaX*stdDev),abs(sin(theta[i])*sigmaY*stdDev)));
        double yLim = max<double>(1.0,max<double>(abs(sin(theta[i])*sigmaX*stdDev),abs(cos(theta[i])*sigmaY*stdDev)));
        
        
        double xStepSize = xLim/(KERNEL_ROW/2);
        double yStepSize = yLim/(KERNEL_COL/2);
        //float* tmpKer = gaborKernel[0][0];
        double xThre = xStepSize/2;
        double yThre = yStepSize/2;
        int k = 0; int j = 0;
        int maxi = 0, maxj = 0, maxk = 0;

        xLim = yLim = filterSize/2;
        xStepSize = yStepSize =1; 
        xThre = yThre = -.001;
        
        
        for(double xV = -xLim; xV < xLim + xThre; xV += xStepSize) {
            k=0;
            for(double yV = -yLim; yV < yLim + yThre; yV += yStepSize) {
                double xT = xV * cos(theta[i]) + yV * sin(theta[i]);
                double yT = -xV * sin(theta[i]) + yV * cos(theta[i]);
                double val= filScale[i]*exp(-.5*(xT*xT/(sigmaX*sigmaX) + yT*yT/(sigmaY*sigmaY)))*(cos(2.0*PI*xT*fact + psi[i])/(2.0*PI*sigmaX*sigmaY) +sin(2.0*PI*xT*fact + psi[i])/(2.0*PI*sigmaX*sigmaY));
                //printf("%f , ",val);
                if(i==0) Gabor0[j][k]=val;
                else if(i==1) Gabor45[j][k]=val;
                else if(i==2) Gabor90[j][k]=val;
                else if(i==3) GaborM45[j][k]=val;
                //gK[i][j][k] *= .3;
                k++;
            }
            
            j++;
            //printf("\n");
        }
        //printf("\n\n\n");
    } 

}

void visualFilterThread::downSampleImage(IplImage* OrigImg,IplImage* DwnImg, int factor) {

    // prepare the destination image, can be avoided for optimization
    //cvReleaseImage(&DwnImg);
    //DwnImg = cvCreateImage(cvSize(OrigImg->width/factor,OrigImg->height/factor),IPL_DEPTH_8U, 1 );
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
    //if(UpImg){
       // cvReleaseImage(&UpImg);
    //}
    
    //UpImg = cvCreateImage(cvSize(OrigImg->width*factor,OrigImg->height*factor),IPL_DEPTH_8U, 1 );
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
    cvReleaseImage(&dwnSampleRG);
    dwnSampleRG = cvCreateImage(cvSize(OrigImg->width/2,OrigImg->height/2),IPL_DEPTH_8U, 1 );
    cvSet(dwnSampleRG,cvScalar(0));
    cvReleaseImage(&dwnSampleGR);
    dwnSampleGR = cvCreateImage(cvSize(OrigImg->width/4,OrigImg->height/4),IPL_DEPTH_8U, 1 );
    cvSet(dwnSampleRG,cvScalar(0));
    cvReleaseImage(&dwnSampleBY);
    dwnSampleBY = cvCreateImage(cvSize(OrigImg->width/8,OrigImg->height/8),IPL_DEPTH_8U, 1 );
    cvSet(dwnSampleRG,cvScalar(0));

    uchar* tmpOrigImg = (uchar*)OrigImg->imageData;
    uchar* origin = (uchar*)OrigImg->imageData;    
    uchar* tmpDwnImg2 = (uchar*)dwnSampleRG->imageData;    
    uchar* originDwn2 = (uchar*)dwnSampleRG->imageData;
    uchar* tmpDwnImg4 = (uchar*)dwnSampleGR->imageData;    
    uchar* originDwn4 = (uchar*)dwnSampleGR->imageData;
    uchar* tmpDwnImg8 = (uchar*)dwnSampleBY->imageData;    
    uchar* originDwn8 = (uchar*)dwnSampleBY->imageData;

    // actual width of images, including paddings    
    int origWidth = OrigImg->widthStep;
    int dwnWidth2 = dwnSampleRG->widthStep;
    int dwnWidth4 = dwnSampleGR->widthStep;
    int dwnWidth8 = dwnSampleBY->widthStep;

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
    uchar* tmpUpImg2 = (uchar*)dwnSampleRG->imageData;
    uchar* originUpImg2 = (uchar*)dwnSampleRG->imageData;
    uchar* tmpUpImg4 = (uchar*)dwnSampleGR->imageData;
    uchar* originUpImg4 = (uchar*)dwnSampleGR->imageData;
    uchar* tmpUpImg8 = (uchar*)dwnSampleBY->imageData;
    uchar* originUpImg8 = (uchar*)dwnSampleBY->imageData;
    
    // actual width of images, including paddings
    int UpWidth2 = dwnSampleRG->widthStep;
    int UpWidth4 = dwnSampleGR->widthStep;
    int UpWidth8 = dwnSampleBY->widthStep;
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

void visualFilterThread::maxImages(IplImage** ImagesTobeAdded, int numberOfImages,IplImage* resultantImage) {
    
    cvSet(resultantImage,cvScalar(0));
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
                continue ;
            }
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
    
    //cvSet(resultantImage,cvScalar(0));
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

    if(yarpedImage->getRawImage() == NULL) return; //printf("YarpImage is NULL  \n");
    //else printf("YARP image %d \n",*yarpedImage->getRawImage());
    //assuming everything is well, we set padding etc
    yarpedImage->resize(originalImage->width,originalImage->height);
    yarpedImage->zero();
    if(type==1){
        uchar* originCVImage = (uchar*)originalImage->imageData;
        uchar* originYARPImage = yarpedImage->getRawImage();

        //printf("Value of openCV image %d and YARPimage%d \n",*originCVImage,*originYARPImage);
        //padding might be different
        int widthCV = originalImage->widthStep;
        int widthY = yarpedImage->getRowSize();
        int minWid = widthCV>widthY? widthY:widthCV;
        for(int i=0; i<originalImage->height; ++i){
            //printf("%dGoing to copy %d bytes ",i,minWid);
            uchar* ptrCV = originCVImage + i*widthCV;
            uchar* ptrY = originYARPImage + i*widthY;
            //printf("from%d  ",*ptrCV);
            //printf("to%d \n",*ptrY);
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

void visualFilterThread::convolve1D(int vecSize, float* vec, IplImage* img, IplImage* resImg, float factor,int shift,int direction, int maxVal){
    
    //IplImage* resImg = cvCreateImage(cvGetSize(img),IPL_DEPTH_16S, 1 );
    float maxPixelVal = 0;
    float minPixelVal = -256;
    int maxRange = 255;
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
                sum *= factor; 
                if(sum>maxPixelVal) maxPixelVal=sum;
                else if(sum<minPixelVal) minPixelVal=sum;               
                *(res+i*resRowSize+j)=sum;//+ shift;//<0?0: sum>maxVal? maxVal:sum;
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
                sum *= factor;
                if(sum>maxPixelVal) maxPixelVal=sum;
                else if(sum<minPixelVal) minPixelVal=sum;  
                *(res+i*resRowSize+j)=sum;//<0?0: sum>maxVal? maxVal:sum;
            }
        } 
    }
/*
    for(int i=0; i<resultImg->height; ++i){
        for(int j=0; j<resultImg->width; ++j){
            *(resultImg->imageData + i*(resultImg->widthStep) + j)= (unsigned char)(maxRange*(*(resImg->imageData + i*(resImg->widthStep) + j)-minPixelVal)/(maxPixelVal-minPixelVal));
        }
    }  
    cvReleaseImage(&resImg);  
*/         
        
}


void visualFilterThread::convolve2D(int rowSize,int colSize, float* ker, IplImage* img, IplImage* resImg, float factor,int shift, int* rangeOfPixels, int maxVal){
    
    //printf("Entering convolution\n");
    //IplImage* resFImg;
    //resFImg = cvCreateImage(cvGetSize(img),IPL_DEPTH_32F, 1 );
    //cvSet(resFImg,cvScalar(0.0));
    int minPixelVal =1024;
    int maxPixelVal =-1024;
    int range =255;

    uchar* ptrImg = (uchar*)img->imageData;
    uchar* ptrResImg = (uchar*)resImg->imageData;
    //float* ptrResImg = (float*)resFImg->imageData;
    uchar* ptrOriginResImg = (uchar*)resImg->imageData;
    //float* ptrOriginResImg = (float*)resFImg->imageData;
    int imgRowSize = img->widthStep;
    int resRowSize = resImg->widthStep/sizeof(uchar);
    //int resRowSize = resFImg->widthStep/sizeof(float);
    int effectiveWidth = rowSize;
    int effectiveHeight = colSize;
    float* kerStartPt = ker;
    float* kerNowPt = ker;
    float norm =1.0;
    uchar* imgStartPt = (uchar*)img->imageData;
    uchar* imgNowPt = (uchar*)img->imageData;
    float sumP =0; int countP =0;
    shift -= 256;
    float scaleF = 2.55*(*rangeOfPixels);//255.0/((float)(*(rangeOfPixels+1)+*rangeOfPixels));
    float shft = *(rangeOfPixels+1)* .01;
    for(int i=0;i<resImg->height;++i){
        ptrResImg = ptrOriginResImg + i*resRowSize;
        effectiveHeight = min(i+colSize/2,img->height)-max(0,i-colSize/2)+1;
        for(int j=0;j<resImg->width;++j){
            effectiveWidth = min(j+rowSize/2,img->width)-max(0,j-rowSize/2)+1;
            kerStartPt = ker + max(0,colSize/2 -i)*rowSize + max(0,rowSize/2-j);
            imgStartPt = (uchar*)img->imageData + (i-effectiveHeight/2)*imgRowSize + (j - effectiveWidth/2);
            kerNowPt = ker + max(0,colSize/2 -i)*rowSize + max(0,rowSize/2-j);
            imgNowPt = (uchar*)img->imageData + (i-effectiveHeight/2)*imgRowSize + (j - effectiveWidth/2);
            float sum =0;
            for(int k=0; k<effectiveHeight; k++){
                kerNowPt = kerStartPt + k* rowSize;
                imgNowPt = imgStartPt + k* imgRowSize;
                for(int l=0; l<effectiveWidth;l++){
                    //float imgNow = *imgNowPt;
                    //float imgNow = (*kerNowPt * (int)*imgNowPt);
                    //printf("Kernel positions now%d\n",kerNowPt-ker);         
                    sum += (*kerNowPt * *imgNowPt);
                    //if(imgNow<0) printf("TThis has value neg%f for pos%d,%d \n", imgNow,i,j);
                    kerNowPt++; imgNowPt++;
                }
            }

            // We have to scale. shift and cut the overflow
            sum *= factor;
            sum += shift;
            
            if(sum<minPixelVal) minPixelVal = sum;
            if(sum>maxPixelVal) maxPixelVal = sum;
            /*if(rangeOfPixels != NULL) {
            float avg = (*(rangeOfPixels+1) +*rangeOfPixels)/2.0;
            sum = abs(sum-avg);
            sum = 255.0*sum/(max(*(rangeOfPixels+1),abs(*rangeOfPixels)));
            }*/

            *ptrResImg =(rangeOfPixels != NULL) ?scaleF*(sum + shft):
                        sum>255?255:sum<0?0:2*(abs(sum-128)) ;//+ shift;
            sumP += *ptrResImg; countP++;
            ptrResImg++;
        }
    }

    if(rangeOfPixels != NULL){
        *rangeOfPixels = (*rangeOfPixels + minPixelVal)/2.0;
        *(rangeOfPixels+1) = (*(rangeOfPixels+1)+maxPixelVal)/2.0;
        //printf("The min is%d and maximum is%d for this image avg%f and count%d\n",minPixelVal,maxPixelVal,sumP/(float)countP,countP);
    
    }
    
/*
    float scalingFact = 255.0/(maxPixelVal-minPixelVal);

    cvNamedWindow("FloatImage");
    cvShowImage("FloatImage",resImg);
    cvNamedWindow("OrigImage");
    cvShowImage("OrigImage",img);
    cvWaitKey(0);*/
   /********* DOESNT WORK ************/
   /*double scalingFact = 255.0/(maxPixelVal -minPixelVal);
   for(int i=0; i<resImg->height; ++i){
        for(int j=0; j<resImg->width; ++j){
            *(resImg->imageData + i*(resImg->widthStep) + j)= (unsigned char)((*(resFImg->imageData + i*(resFImg->widthStep)/sizeof(float) + j)-minPixelVal)*scalingFact);
        }
    }
    cvReleaseImage(&resFImg);*/
   /**************************************/ 

}

void visualFilterThread::cropImage(int* corners, IplImage* imageToBeCropped, IplImage* retImage){

    // very lame cropping
    int imgWidth = corners[2]-corners[0];
    int imgHeight = corners[3]-corners[1];
    //cvReleaseImage(&retImage);
    //retImage = cvCreateImage(cvSize(imgWidth,imgHeight),IPL_DEPTH_8U, 1 );
    cvSet(retImage,cvScalar(0));

    
    uchar* originDestImg = (uchar*)retImage->imageData;
    uchar* originSourImg = (uchar*)imageToBeCropped->imageData;
    int widthDest = retImage->widthStep;
    int widthSour = imageToBeCropped->widthStep;
    uchar* sourceRow = originSourImg;
    uchar* destRow = originDestImg;
    size_t stride = imgWidth*sizeof(uchar);
    for(int i = 0; i<imgHeight; ++i){
        //for(int j = 0; j<imgWidth; ++j){
           //*(originDestImg+ i * widthDest + j) = *(originSourImg + (i+corners[1])*widthSour + j + corners[0]);
        //}
        memcpy((originDestImg+ i * widthDest),(originSourImg + (i+corners[1])*widthSour + corners[0]),stride);
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

void visualFilterThread::cropCircleImage(int* center, float radius, IplImage* srcImg) {
    radius -= 4;
    for(int i=0; i< srcImg->height; ++i){
        for(int j=0; j< srcImg->width; ++j){
            if((i - center[0])*(i - center[0]) + (j - center[1])*(j - center[1]) >= radius*radius) {
                *(srcImg->imageData + i*srcImg->widthStep + j) = 0; //blacken the pixel out of circle
            }
        }
    }
}


void visualFilterThread::threadRelease() {
    

    trsf.freeLookupTables();
    lpMono.freeLookupTables();
    
    printf("Releasing thread \n");
    if(resized){
        cvReleaseImage(&cvRedPlane);
        cvReleaseImage(&cvGreenPlane);
        cvReleaseImage(&cvBluePlane);
        cvReleaseImage(&cvYellowPlane);

        cvReleaseImage(&redG);
        cvReleaseImage(&greenR);
        cvReleaseImage(&blueY);
        
        cvReleaseImage(&cvRedPlus);
        cvReleaseImage(&cvRedMinus);
        cvReleaseImage(&tmpRedPlus);
        cvReleaseImage(&tmpRedMinus);
        
        cvReleaseImage(&cvGreenPlus);
        cvReleaseImage(&cvGreenMinus);
        cvReleaseImage(&tmpGreenPlus);
        cvReleaseImage(&tmpGreenMinus);
        
        cvReleaseImage(&cvBluePlus);
        cvReleaseImage(&cvYellowMinus);
        cvReleaseImage(&tmpBluePlus);
        cvReleaseImage(&tmpYellowMinus);
        cvReleaseImage(&hRG);
        cvReleaseImage(&vRG);
        cvReleaseImage(&hGR);
        cvReleaseImage(&vGR);
        cvReleaseImage(&hBY);
        cvReleaseImage(&vBY);

        cvReleaseImage(&tempHRG);
        cvReleaseImage(&tempVRG);
        cvReleaseImage(&tempHGR);
        cvReleaseImage(&tempVGR);
        cvReleaseImage(&tempHBY);
        cvReleaseImage(&tempVBY);

        cvReleaseImage(&intensityImage); 
        cvReleaseImage(&filteredIntensityImage);
        cvReleaseImage(&filteredIntensityImage1);
        cvReleaseImage(&filteredIntensityImage2);
        cvReleaseImage(&totImage);
        
        cvReleaseImage(&dwnImage);

        cvReleaseImage(&dwnSampleRG);
        cvReleaseImage(&dwnSampleGR);
        cvReleaseImage(&dwnSampleBY);
        cvReleaseImage(&upSampleRG);
        cvReleaseImage(&upSampleGR);
        cvReleaseImage(&upSampleBY);
        
        cvReleaseImage(&dwnSampleRGa);
        cvReleaseImage(&dwnSampleGRa);
        cvReleaseImage(&dwnSampleBYa);
        cvReleaseImage(&upSampleRGa);
        cvReleaseImage(&upSampleGRa);
        cvReleaseImage(&upSampleBYa);
        
        cvReleaseImage(&dwnSampleRGb);
        cvReleaseImage(&dwnSampleGRb);
        cvReleaseImage(&dwnSampleBYb);
        cvReleaseImage(&upSampleRGb);
        cvReleaseImage(&upSampleGRb);
        cvReleaseImage(&upSampleBYb);
        

        cvReleaseImage(&dwnSampleRGFil);
        cvReleaseImage(&dwnSampleGRFil);
        cvReleaseImage(&dwnSampleBYFil);
        cvReleaseImage(&dwnSampleRGFila);
        cvReleaseImage(&dwnSampleGRFila);
        cvReleaseImage(&dwnSampleBYFila);
        
        cvReleaseImage(&dwnSampleRGFilb);
        cvReleaseImage(&dwnSampleGRFilb);
        cvReleaseImage(&dwnSampleBYFilb); 
     
        cvReleaseImage(&tmpRedGreen);
        cvReleaseImage(&tmpGreenRed);
        cvReleaseImage(&tmpBlueYellow);
    }
    printf("Release complete!\n");
    resized = false;    
    
}

void visualFilterThread::onStop() {

    printf("calling on-stop\n");
    yPortIn.interrupt();
    uvPortIn.interrupt();
    imagePortExt.interrupt();
    gaborPort0.interrupt();
    gaborPort45.interrupt();
    gaborPort90.interrupt();
    gaborPortM45.interrupt();
    
    yPortIn.close();
    uvPortIn.close();
    imagePortExt.close();
    gaborPort0.close();
    gaborPort45.close();
    gaborPort90.close();
    gaborPortM45.close();
    
    printf("done with on-stop\n");
    
}


