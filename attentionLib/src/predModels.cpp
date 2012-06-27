// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini
 * email:  ugo.pattacini@iit.it
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

#include <iCub/attention/predModels.h>

using namespace yarp::sig;


namespace attention
{

namespace predictor
{


genPredModel::genPredModel() {
    valid = true;
    type = "constVelocity";
    
}

genPredModel::genPredModel(const genPredModel &model) {
    valid = true;
    type = "constVelocity";
}

genPredModel &genPredModel::operator =(const genPredModel &model) {
    valid = model.valid;
    type  = model.valid;
    A = model.A;
    B = model.B;
    return *this;
}

bool genPredModel::operator ==(const genPredModel &model) {
    return ((valid==model.valid)&&(type==model.type)&&(A==model.A)&&(B==model.B)); 
}

void genPredModel::init(double param) {
    
    Matrix _A(3,3);
    Matrix _B(3,3);
    A = _A;
    B = _B;   
    A.zero();
    B.zero();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////

linVelModel::linVelModel() {
    valid = true;
    type = "constVelocity";
    
}

linVelModel::linVelModel(const linVelModel &model) {
    valid = true;
    type = "constVelocity";
}

linVelModel &linVelModel::operator =(const linVelModel &model) {
    valid = model.valid;
    type  = model.valid;
    A = model.A;
    B = model.B;
    return *this;
}

bool linVelModel::operator ==(const linVelModel &model) {
    return ((valid==model.valid)&&(type==model.type)&&(A==model.A)&&(B==model.B)); 
}

void linVelModel::init(double param) {

    Matrix _A(3,3);
    Matrix _B(3,3);
    A = _A;
    B = _B;
    
    A.zero();
    B.zero();
    B(0,0) = 1;

}
 
//////////////////////////////////////////////////////////////////////////////////////////////////////


linAccModel::linAccModel() {
    valid = true;
    type = "constAcceleration";
}

linAccModel::linAccModel(const linAccModel &model) {
    valid = true;
    type = "constAcceleration";
}

linAccModel &linAccModel::operator =(const linAccModel &model) {
    valid = model.valid;
    type  = model.valid;
    A = model.A;
    B = model.B;
    return *this;
}

bool linAccModel::operator ==(const linAccModel &model) {
    return ((valid == model.valid) && (type == model.type) && (A == model.A) && (B == model.B)); 
}

void linAccModel::init(double param) {
    Matrix _A(3,3);
    Matrix _B(3,3);
    A = _A;
    B = _B;

    A.zero();
    B.zero();
    B(1,1) = 1;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////


minJerkModel::minJerkModel() {
    valid = true;
    type = "minimumJerk";
}

minJerkModel::minJerkModel(const minJerkModel &model) {
    valid = true;
    type = "minimumJerk";
}

minJerkModel &minJerkModel::operator =(const minJerkModel &model) {
    valid = model.valid;
    type  = model.valid;
    A = model.A;
    B = model.B;
    return *this;
}

bool minJerkModel::operator ==(const minJerkModel &model) {
    return ((valid == model.valid) && (type == model.type) 
            && (A == model.A)&&(B == model.B)); 
}

void minJerkModel::init(double param) {
    T = param;
    double T2 = T * T;
    double T3 = T2 * T;

    a = -150.765868956161/T3;
    b = -84.9812819469538/T2;
    c = -15.9669610709384/T;
    
    Matrix _A(3,3);
    Matrix _B(3,3);
    A = _A;
    B = _B;

    A.zero();
    B.zero();
    
    A(0,1) = 1;
    A(1,2) = 1;
    A(2,0) = a / (T3);
    A(2,1) = b / (T2);
    A(2,2) = c / (T2);

    B(2,2) = a / (T3);
}


}
}
