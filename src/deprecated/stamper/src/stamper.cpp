//Copyright: (C) 2012,2013 RobotCub Consortium Authors: Konstantinos Theofilis, Katrin Solveig Lohan
#include <yarp/os/all.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string.h>
#include <math.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/ResourceFinder.h>

using namespace yarp::os;
using namespace std;

int main(int argc, char *argv[]) {
  Network yarp;
  
  ResourceFinder rf;
  rf.setVerbose();
  rf.configure(argc, argv);
  
  // take reference coordinates
  double refX=rf.find("x").asFloat32();
  double refY=rf.find("y").asFloat32();
  double refZ=rf.find("z").asFloat32();
  
  // open ports
  BufferedPort<Bottle> inPort;
  bool stampIn = inPort.open("/stamper/in");
  BufferedPort<Bottle> iKinInPort;
  bool iKinIn = iKinInPort.open("/stamper/iKinIn");
  BufferedPort<Bottle> outPort;
  bool stampOut = outPort.open("/stamper/out");
  BufferedPort<Bottle> refPort;
  bool ref = refPort.open("/stamper/ref");

  if (!stampIn || !stampOut || !iKinIn) {
    fprintf(stderr, "Failed to create ports.\n");
    fprintf(stderr, "Maybe you need to start a nameserver (run 'yarpserver' \n");
    return 1;
  }
  
  // connect ports
  yarp.connect("/icub/camcalib/left/out",inPort.getName());
  yarp.connect("/iKinGazeCtrl/x:o",iKinInPort.getName());
  yarp.connect("/stamper/ref","/iKinGazeCtrl/xd:i");
  yarp.connect("/stamper/out","/logger");
  
  // check if logger port is connected
  bool loggerCheck = yarp.isConnected("/stamper/out","/logger",true);
  
  if (!loggerCheck){
	  fprintf(stderr, "Start the logger port!.\n");
	  return 1;
  } 
  
  while(true) {
	int count = 0;
	int frameSeq;
	int iSeq;
	double vTime;
	double iTime;
	Stamp vStamp;
	Stamp iStamp;

	// get incoming bottles
	cout << "waiting for input" << endl;
	Bottle *in = inPort.read();
	
	Bottle *iKinIn = iKinInPort.read();
	Bottle &out	= outPort.prepare();
	
	if (in!=NULL && iKinIn!=NULL) {
	  
	  if (count==0) {
	    Bottle &refOut	= refPort.prepare();
	    refOut.clear();
	    
	    refOut.addFloat32(refX);
	    refOut.addFloat32(refY);
	    refOut.addFloat32(refZ);
	    refPort.writeStrict();
	  }	
	  
	  // get sequence and timestamps
	  // get information from the image
	  inPort.getEnvelope(vStamp);
	  frameSeq = vStamp.getCount();
	  vTime = vStamp.getTime();
	  // get information from iKin
	  iKinInPort.getEnvelope(iStamp);
	  iSeq = iStamp.getCount();
	  iTime = iStamp.getTime();
	  
	  
	  //get the iKin input
	  
	  // double x = iKinIn->get(0).asFloat32();
	  // double y = iKinIn->get(1).asFloat32();
	  // double z = iKinIn->get(2).asFloat32();
	  
	  
	  cout << "receiving input" << endl;
	  out.clear();
	  
	  //copy the iKinGazeCtrl input
	  out.copy(*iKinIn,0,-1);
	  
	}
	
	//	out.addFloat32(x);
	//	out.addFloat32(y);
	//	out.addFloat32(z);
	
	// add sequence numbers and timestamps
	out.addInt16(frameSeq);
	out.addInt16(iSeq);
	out.addFloat32(vTime);
	out.addFloat32(iTime);
	outPort.writeStrict();
	count++;
	
  }
}
