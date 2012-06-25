// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Francesco Rea
 * email:  francesco.rea@iit.it
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

#ifndef __PREDICTOR_MODEL_H__
#define __PREDICTOR_MODEL_H__

#include <yarp/os/all.h>
#include <yarp/sig/Matrix.h>
#include <string>
#include <deque>

//namespace emorph
//{

//namespace ecodec
//{


// forward declaration
//class eEventQueue;


/**************************************************************************/
class predModel {
protected:
    bool valid;                 // defines the validity of the model
    std::string type;           // defines the typology of the model
    yarp::sig::Matrix A;        // matrices of the space state
    yarp::sig::Matrix B;        // matrices of the space state
public:
    predModel() : valid(false), type("") { }
    bool isValid() const        { return valid; }
    std::string getType() const { return type;  }

    yarp::sig::Matrix getA() const    {return A; };
    yarp::sig::Matrix getB() const    {return B; };

    //virtual yarp::sig::Matrix getA() const = 0;
    //virtual yarp::sig::Matrix getB() const = 0;
    virtual void setA(yarp::sig::Matrix mat) = 0;
    virtual void setB(yarp::sig::Matrix mat) = 0;
    
    virtual bool operator ==(const predModel &pModel) = 0;

};


/**************************************************************************/
class linVelModel : public predModel {
protected:
    int stamp;

public:
    linVelModel();    
    linVelModel(const linVelModel &model);
    

    linVelModel &operator = (const linVelModel &event);
    bool operator ==(const linVelModel &model);    

    //yarp::sig::Matrix getA() const    {return A; };
    //yarp::sig::Matrix getB() const    {return B; };
        
    void setA(const yarp::sig::Matrix mat) {A = mat; };
    void setB(const yarp::sig::Matrix mat) {B = mat; };

    int getLength() const { return 1; }
    bool operator ==(const predModel &model) { return operator==(dynamic_cast<const linVelModel&>(model)); }

};



//}

//}

#endif


