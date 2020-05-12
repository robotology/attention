/*
 * Copyright (C) 2014 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Authors: Francesco Rea 
 * email: francesco.rea@iit.it
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */


#define DEBUG

// global includes
#include <iostream> 
#include <stdio.h>
#include <mex.h>

// include yarp 
#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Sound.h>

// Local includes
//#include <modelcomponent.h>
//#include <componentmanager.h>

// Namespaces
using namespace yarp::os;
using namespace yarp::sig;
//using namespace mexWBIComponent;

//Global variables
//static ComponentManager *componentManager = NULL;

//=========================================================================================================================
// Entry point function to library
void mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
  
   
#ifdef DEBUG
  mexPrintf("starting the mex function\n");
#endif

  yarp::os::Network yarp;

  // Initialisation of the component, i.e first call after a 'close all' or matlab start
  if (nrhs < 1) {
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Initialisation has not been performed correctly.");
  }
  
  
  // Check to be sure input is of type char 
  if (!(mxIsChar(prhs[0])))
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Initialisation must include component.\n.");
    }
  
  if(nrhs ==2)
    {
      if (!(mxIsChar(prhs[1])))
	{
	  mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Initialisation must include component and a robot name.\n.");
	}
      
    }
  else
    {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Initialisation must include component and a robot name.\n.");
    }
  
  
#ifdef DEBUG
  mexPrintf("starting to process function\n");
#endif
  
  if (nrhs < 1) {
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Required Component must be named.");
  }
  
  
  // Check to be sure input is of type char 
  if (!(mxIsChar(prhs[0]))){
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Input must be of type string.\n.");
  }

  BufferedPort<Sound> bufferPort;
  bufferPort.open("/receiver");
  mexPrintf("making the connection \n"); 
  Network::connect("/sender", "/receiver");
  //yarp.connect("/sender","/receiver");
  mexPrintf("connection success \n");
  
  //Sound *s; 
  /*while (true) {
    s = bufferPort.read(true);
    if (s!=NULL){
      mexPrintf("received \n");
      //put->renderSound(*s);
    }
    } */
  

  
}


