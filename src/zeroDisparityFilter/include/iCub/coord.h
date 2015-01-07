// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco on Andrew Dankers ` code mainteined by Vadim Tikhanoff
 * email:   francesco.rea@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#ifndef __COORD_H__
#define __COORD_H__

#include <stdlib.h>

      
struct Coord{
	int x, y;
	
	Coord() {}
	Coord(int a, int b) { x = a; y = b; }
	
	Coord operator- ()        { return Coord(-x, -y); }
	Coord operator+ (Coord a) { return Coord(x + a.x, y + a.y); }
	Coord operator- (Coord a) { return Coord(x - a.x, y - a.y); }
	bool  operator< (Coord a) { return (x <  a.x) && (y <  a.y); }
	bool  operator<=(Coord a) { return (x <= a.x) && (y <= a.y); }
	bool  operator> (Coord a) { return (x >  a.x) && (y >  a.y); }
	bool  operator>=(Coord a) { return (x >= a.x) && (y >= a.y); }
	bool  operator==(Coord a) { return (x == a.x) && (y == a.y); }
	bool  operator!=(Coord a) { return (x != a.x) || (y != a.y); }
};

#endif
