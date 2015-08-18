/*
 * Copyright: (C) 2010 RobotCub Consortium
 * Authors: Paul Fitzpatrick, Francesco Nori
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#include <stdio.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/AudioGrabberInterfaces.h>
#include <yarp/os/Property.h>
#include <yarp/os/NetInt16.h>

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/Time.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/BufferedPort.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

const double rec_seconds = 0.1;
const int rate = 48000;
const int fixedNSample = 4096;

int main(int argc, char *argv[]) {
    // initialization

    // Open the network
    Network yarp;
    //BufferedPort<Sound> p;
    Port p;
    p.open("/sender");

    // Get a portaudio read device.
    Property conf;
    conf.put("device","portaudio");
    conf.put("read", "");
    // conf.put("samples", rate * rec_seconds);
    conf.put("samples", fixedNSample);
    conf.put("rate", rate);
    PolyDriver poly(conf);
    IAudioGrabberSound *get;

    // Make sure we can read sound
    poly.view(get);
    if (get==NULL) {
        printf("cannot open interface");
        return 1;
    }
    else{
        printf("correctly opened the interface rate: %d, number of samples: %d, number of channels %d \n",rate, rate*rec_seconds, 2);
    }

    //Grab and send
    Sound s;
    //Bottle b;
    //b.addString("hello");

    //unsigned char* dataSound;
    //short* dataAnalysis;
    //int v1, v2;
    //int i = 0, j = 0;
    //NetInt16 v;
    // i = sample amd j = channels;

    get->startRecording(); //this is optional, the first get->getsound() will do this anyway.
    Stamp ts;
    while (true)
    {
      double t1=yarp::os::Time::now();
      ts.update();  
      //s = p.prepare();           
      
      get->getSound(s);        
      
      //v1 = s.get(i,j);
      //v = (NetInt16) v1;
      //v2 = s.get(i+1,j+1); 
      //dataAnalysis = (short*) dataSound;        
      
      p.setEnvelope(ts);
      p.write(s);

      double t2=yarp::os::Time::now();
      printf("acquired %f seconds \n", t2-t1);
    }
    get->stopRecording();  //stops recording.

    return 0;
}

