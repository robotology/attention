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

namespace attention
{

namespace predictor
{


// forward declaration
class modelQueue;


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
    void setA(const yarp::sig::Matrix mat) {A = mat;};
    void setB(const yarp::sig::Matrix mat) {B = mat;};

    //virtual yarp::sig::Matrix getA() const = 0;
    //virtual yarp::sig::Matrix getB() const = 0;
    virtual void init(double param) = 0;
    //virtual void setB(yarp::sig::Matrix mat) = 0;
    
    virtual bool operator ==(const predModel &pModel) = 0;

};

/**************************************************************************/
class genPredModel : public predModel {
protected:
   

public:
    genPredModel();    
    genPredModel(const genPredModel &model);
    

    genPredModel &operator = (const genPredModel &model);
    bool operator ==(const genPredModel &model);    

    int getLength() const  { return 1; }
    bool operator ==(const predModel &model) { return operator==(dynamic_cast<const genPredModel&>(model)); }
    
    /**
    * initialisation of the matrices typical
    * @param parameter of the parametric initialisation
    */
    void init(double param);

};

/**************************************************************************/
class modelQueue : public std::deque<predModel*>
{
private:
    modelQueue(const modelQueue&);
    modelQueue &operator=(const modelQueue&);

protected:
    bool owner;

public:
    modelQueue()                    { owner = true;        }
    modelQueue(const bool _owner)   { owner = _owner;      }
    void setOwner(const bool owner) { this->owner = owner; }
    bool getOwner()                 { return owner;      }
    ~modelQueue() {
        if (owner)
            for (size_t i=0; i<size(); i++)
                if ((*this)[i]!=NULL)
                    delete (*this)[i];
        
        clear();
    }
};


/**************************************************************************/
class linVelModel : public genPredModel {
protected:
   

public:
    linVelModel();    
    linVelModel(const linVelModel &model);
    

    linVelModel &operator = (const linVelModel &model);
    bool operator ==(const linVelModel &model);    

    int getLength() const  { return 1; }
    bool operator ==(const predModel &model) { return operator==(dynamic_cast<const linVelModel&>(model)); }
    
    /**
    * initialisation of the matrices typical
    * @param parameter of the parametric initialisation
    */
    void init(double param);

};

/**************************************************************************/
class linAccModel : public genPredModel {
protected:
    

public:
    linAccModel();    
    linAccModel(const linAccModel &model);
    

    linAccModel &operator = (const linAccModel &model);
    bool operator ==(const linAccModel &model);    
        
    //void setA(const yarp::sig::Matrix mat) {A = mat; };
    //void setB(const yarp::sig::Matrix mat) {B = mat; };

    int getLength() const { return 1; }
    bool operator ==(const predModel &model) { return operator==(dynamic_cast<const linAccModel&>(model)); }

    /**
    * initialisation of the matrices typical
    * @param parameter of the parametric initialisation
    */
    void init(double param);

};

/**************************************************************************/
class minJerkModel : public genPredModel {
protected:
    double a ;    // parameters of the model
    double b ;    // parameters of the model
    double c ;    // parameters of the model
    double T;     // period of the motion 

public:
    minJerkModel();    
    minJerkModel(const minJerkModel &model);
    

    minJerkModel &operator = (const minJerkModel &model);
    bool operator ==(const minJerkModel &model);    
        
    //void setA(const yarp::sig::Matrix mat) {A = mat; };
    //void setB(const yarp::sig::Matrix mat) {B = mat; };

    int getLength() const { return 1; }
    bool operator ==(const predModel &model) { return operator==(dynamic_cast<const minJerkModel&>(model)); }

    /**
    * initialisation of the matrices typical
    * @param parameter of the parametric initialisation
    */
    void init(double param);

};


}

}

#endif


