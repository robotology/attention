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
  rf.configure("ICUB_ROOT", argc, argv);
  
  // take reference coordinates
  double refX=rf.find("x").asDouble();
  double refY=rf.find("y").asDouble();
  double refZ=rf.find("z").asDouble();
  
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
  yarp.connect("/iKinGazeCtrl/head/x:o",iKinInPort.getName());
  yarp.connect("/stamper/ref","/iKinGazeCtrl/head/xd:i");
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
	    
	    refOut.addDouble(refX);
	    refOut.addDouble(refY);
	    refOut.addDouble(refZ);
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
	  
	  // double x = iKinIn->get(0).asDouble();
	  // double y = iKinIn->get(1).asDouble();
	  // double z = iKinIn->get(2).asDouble(); 
	  
	  
	  cout << "receiving input" << endl;
	  out.clear();
	  
	  //copy the iKinGazeCtrl input
	  out.copy(*iKinIn,0,-1);
	  
	}
	
	//	out.addDouble(x);
	//	out.addDouble(y);
	//	out.addDouble(z);
	
	// add sequence numbers and timestamps
	out.addInt(frameSeq);
	out.addInt(iSeq);
	out.addDouble(vTime);
	out.addDouble(iTime);
	outPort.writeStrict();
	count++;
	
  }
}
