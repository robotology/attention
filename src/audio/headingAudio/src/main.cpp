// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <iCub/headingControlModule.h>

//YARP_DECLARE_DEVICES(icubmod)

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;


int main(int argc, char *argv[])
{
    Network yarp;
    if (!yarp.checkNetwork())
    {
        fprintf(stdout,"YARP server not available!\n");
        return -1;
    }

    //YARP_REGISTER_DEVICES(icubmod)
    
    //myReport rep;

    ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setMonitor(&rep);
    rf.setDefaultContext("audioAttention");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    managerModule mod;
    mod.setName("/headingAudio");

    return mod.runModule(rf);
}
