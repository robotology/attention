/*
 * Copyright: (C) 2010 RobotCub Consortium
 * Authors: Paul Fitzpatrick, Francesco Nori
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/AudioGrabberInterfaces.h>
#include <yarp/os/Property.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/BufferedPort.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

const int rec_seconds = 1;

int main(int argc, char *argv[]) {

    // Open the network
    Network yarp;
    BufferedPort<Sound> p;
    p.open("/sender");

    // Get a portaudio read device.
    Property conf;
    conf.put("device","portaudio");
    conf.put("read", "");
    conf.put("samples", 48000*rec_seconds);
    //conf.put("rate", 16000);
    PolyDriver poly(conf);
    IAudioGrabberSound *get;

    // Make sure we can read sound
    poly.view(get);
    if (get==NULL) {
        printf("cannot open interface\n");
        return 1;
    }

    //Grab and send
    Sound s;
    Bottle b;
    b.addString("hello");

    get->startRecording(); //this is optional, the first get->getsound() will do this anyway.
    while (true)
    {
        double t1=yarp::os::Time::now();
	s = p.prepare();
        get->getSound(s);
        double t2=yarp::os::Time::now();
        printf("acquired %f seconds\n", t2-t1);
	//p.write(b);
        p.write();
    }
    get->stopRecording();  //stops recording.

    return 0;
}

