// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2012  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.rea@iit.it
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

/**
 * @file attPrioritiserThread.cpp
 * @brief Implementation of the gaze arbiter thread(see header attPrioritiserThread.h)
 */

#include <yarp/dev/AudioGrabberInterfaces.h>
#include <yarp/sig/Sound.h>
#include <yarp/dev/PolyDriver.h>
#include <stdio.h>
// #include <math.h>
// #include <yarp/os/all.h>
// #include <yarp/os/Network.h>
// #include <yarp/os/Port.h>

// using namespace yarp::os;
// using namespace yarp::sig;
// using namespace yarp::dev;

// int main(int argc, char *argv[]) {

//     // Open the network
//     Network yarp;
//     BufferedPort<Sound> pReceiver;
//     pReceiver.open("/receiver");
//     //Network::connect("/sender", "/receiver");

//     Port pSender;
//     pSender.open("/sender");

//     // Get an audio write device.
    Property conf;
    conf.put("device","portaudio");
    conf.put("samples", "4096");
    conf.put("write", "1");
    PolyDriver polyRender(conf);
    if(!polyRender.isValid()) {
        printf("cannot open interface \n");
        return 1;
    }
    IAudioRender *put;


    // Get a portaudio read device.
    //Property conf;
    conf.put("device","portaudio");
    conf.put("read", "");
    //conf.put("samples", 4096);
    //conf.put("rate", 16000);
    PolyDriver polyGrabber(conf);
    if(!polyRender.isValid()) {
        printf("cannot open interface \n");
        return 1;
    }
    IAudioGrabberSound *get;

    
    /*
    // Make sure we can write sound
    polyRender.view(put);
    if (put==NULL) {
        printf("cannot open interface\n");
        return 1;
    }
    //Receive and render
    Sound *s;
    while (true)
      {
        s = p.read(false);
        if (s!=NULL)
            put->renderSound(*s);
      }
    return 0;
    */

 

    
    /*
    // Make sure we can read sound
    polyGrabber.view(get);
    if (get==NULL) {
        printf("cannot open interface\n");
        return 1;
    }

    //Grab and send
    Sound s;
    while (true)
      {
        get->getSound(s);
        p.write(s);
      }
    return 0;
    */

    // echo from microphone to headphones, superimposing an annoying tone   

    double vv=0;
    while(true){   
        Sound s;   
        get->getSound(s);   
        for (int i=0; i<s.getSamples(); i++) {   
            double now = Time::now();   
            static double first = now;   
            now -= first;   
            if ((long int) (now*2) % 2 == 0) {   
                vv += 0.08;   
            } else {   
                vv += 0.04;   
            }   
            double dv = 500*sin(vv);   
            for (int j=0; j<s.getChannels(); j++) {   
                int v =s.get(i,j);   
                s.set((int)(v+dv+0.5),i,j);   
            }   
        }   
        put->renderSound(s);   
    }   
    
    Network::fini();


    

}
