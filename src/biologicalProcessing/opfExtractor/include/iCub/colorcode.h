// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

// colorcode.cpp
//
// Color encoding of flow vectors
// adapted from the color circle idea described at
//   http://members.shaw.ca/quadibloc/other/colint.htm
//
// Daniel Scharstein, 4/2007
// added tick marks and out-of-range coding 6/05/07

/**
 * @file plotterThread.h
 * @brief Definition of a thread that sends frame-based representation 
 * (see plotterthread.h).
 */

#ifndef _COLOR_CODE_H_
#define _COLOR_CODE_H_

#include <stdlib.h>
#include <math.h>
#include <stdio.h>

typedef unsigned char uchar;

#define MAXCOLS 60
#define PI 3.1415


void setcols(int r, int g, int b, int k);
bool makecolorwheel();
void computeColor(float fx, float fy, uchar *pix);


#endif  //_COLOR_CODE_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

