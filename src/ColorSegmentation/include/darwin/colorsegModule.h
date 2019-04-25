#ifndef COLORSEG_MODULE_H
#define COLORSEG_MODULE_H

#include "darwin/colorsegThread.h"

#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

namespace Darwin{
// change this namespace name according to your module name:
namespace colorseg {

class colorsegModule: public RFModule
{

private:
	static const string CMD_LIST[];
	static const string CMD_DESC[];
	static const unsigned int CMD_SIZE;
	typedef enum {help,start,stop,exit} colorsegCommands;

	// tThread is the main thread (i.e. a vision recog application)
	// rpcPort is used to communicate with the module using a rpc protocol
	colorsegThread* tThread;
	Port			rpcPort;

	string moduleName;
	string threadName;
	int threadPeriod;

	bool identifyCmd(Bottle cmdBot, colorsegCommands &cmd);

public:
	// Configure module parameters coming either from console or configuration file (.ini)
	bool configure(ResourceFinder &rf);

	// set Module periodicity in seconds
	double getPeriod();
	
	// Main function, called with period getPeriod()
	bool updateModule();

	bool respond(const Bottle &command, Bottle &reply);

	bool interruptModule();

	// always called if interruptModule() returns true
	bool close();
};

} // end namespace colorseg
} // end namespace Darwin

#endif
