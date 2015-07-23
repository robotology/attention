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
  mexPrintf("happily starting the procedure \n");

  // initialization
  yarp::os::Network yarp;
  const mxArray *a_in_m, *b_in_m, *c_out_m, *d_out_m;
  double *a, *b, *c, *d;
  const mwSize *dims;

  //figure out dimensions
  //dims = mxGetDimensions(prhs[0]);
  //numdims = mxGetNumberOfDimensions(prhs[0]);
  //dimy = (int)dims[0]; dimx = (int)dims[1];

  // Initialisation of the component, i.e first call after a 'close all' or matlab start
  if (nrhs < 1) {
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Initialisation has not been performed correctly.");
  }
  
  
  // Check to be sure input is of type char 
  if (!(mxIsChar(prhs[0]))) {
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Initialisation must include component.\n.");
  }
  if(nrhs ==2) {
    if (!(mxIsChar(prhs[1]))) {
      mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Initialisation must include component and a robot name.\n.");
    }
      
  }
  else {
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Initialisation must include component and a robot name.\n.");
  }
   
  //mexPrintf("starting to process function\n");
  
  if (nrhs < 1) {
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:invalidNumInputs","Required Component must be named.");
  }  
  
  // Check to be sure input is of type char 
  if (!(mxIsChar(prhs[0]))){
    mexErrMsgIdAndTxt( "MATLAB:mexatexit:inputNotString","Input must be of type string.\n.");
  }

  mexPrintf("correctly parsed all the inputs \n");

  BufferedPort<Sound> bufferPort;
  bufferPort.open("/receiver");
  mexPrintf("making the connection \n"); 
  Network::connect("/sender", "/receiver");
  //yarp.connect("/sender","/receiver");
  mexPrintf("connection success \n");
  
  Sound* s;
  Bottle* bottReceived;
  s = bufferPort.read(true);
  int nchannels = 2;
  int nsamples  = 48000;
  mexPrintf("received Sound  \n");

  //associate outputs
  c_out_m = plhs[0] = mxCreateDoubleMatrix(1,48000 * 2,mxREAL);
  d_out_m = plhs[1] = mxCreateDoubleMatrix(2,2,mxREAL);

  //associate pointers
  c = mxGetPr(plhs[0]);
  d = mxGetPr(plhs[1]);
  
  if(s!=NULL) {
    unsigned char* soundData = s->getRawData();
  }
  else{
    mexPrintf("Cycle skipped because of the null value of the sound pointer \n");
  }
  
  unsigned char* dataSound;

  if(s!=NULL) {
    dataSound = s->getRawData(); 
  }
  else {
    mexPrintf("NULL DATA");
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 48000; j++) {
      c[i * 48000 + j] = dataSound[i * 48000 + j];
    }
  }
  

  d[0] = 3.0;
  d[1] = 4.0;
  d[2] = 1.0;
  d[3] = 3.0;

  bufferPort.interrupt();
  bufferPort.close();

  mexPrintf("end. \n");

  /*while (true) {
    s = bufferPort.read(true);
    if (s!=NULL){
      mexPrintf("received \n");
      //put->renderSound(*s);
    }
    } */
  

  return;
}


