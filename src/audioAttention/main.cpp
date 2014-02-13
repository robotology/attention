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
#include <math.h>
#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

#include <iostream>
#include <fstream>
using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;

 int main(int argc, char *argv[]) {

     // Open the network
     Network yarp;

     ofstream myfile;
     myfile.open ("./example.bin", ios::out | ios::app | ios::binary);
     if (myfile.is_open()) { 
         cout<<"file correctly opened"<<endl;
        
     }
     else{
         cout<<"file is not opened"<<endl;
         return 0;
     }


     BufferedPort<Sound> pReceiver;
     pReceiver.open("/receiver");
     //Network::connect("/sender", "/receiver");

     Port pSender;
     pSender.open("/sender");
     
     // Get an audio write device.
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

    
    
    // Make sure we can write sound
    polyRender.view(put);
    if (put==NULL) {
        printf("cannot open interface\n");
        return 1;
    }
    //Receive and render
    Sound *sRead;
    /*
      while (true)
      {
        sRead = pReceiver.read(false);
        if (sRead!=NULL)
            put->renderSound(*sRead);
      }
    return 0; 
    */
    
    // Make sure we can read sound
    polyGrabber.view(get);
    if (get==NULL) {
        printf("cannot open interface\n");
        return 1;
    }

    //Grab and send
    Sound *sWrite;
    /*
      while (true)
      {
        get->getSound(*sWrite);
        //pSender.write(sWrite);
      }
    return 0;
    */
    
    float sample = 3.325e37;
    int sample_conv = (int) sample;
    

    printf("sample %f sample_conv %d \n",sample, sample_conv);

    unsigned char s_a; 
    unsigned char s_b; 
    unsigned char s_c; 
    unsigned char s_d; 

    //01000000 10100110 01100110 01100110
    s_a = 0x66;
    s_b = 0x66;
    s_c = 0xA6;
    s_d = 0x40;


    //s_a = (unsigned char)(sample_conv & 0x000000FF);          //LS
    //s_b = (unsigned char)(sample_conv >> 8) & 0x000000FF;
    //s_c = (unsigned char)(sample_conv >> 16) & 0x000000FF;
    //s_d = (unsigned char)((int)sample >> 24) & 0x000000FF;    //MS

    printf("%02x %02x %02x %02x \n",s_a, s_b, s_c, s_d);

    int check_int = 0;
    //check = s_a | (s_b << 8) | (s_b << 16) | (s_b << 24);
    check_int = s_a | (s_b << 8) | (s_c << 16) | (s_d << 24);
    printf("check_int %08x \n", check_int);
    
    //check = check + 0.25;
    float check = (float) check_int;

    printf("check %f \n", check);

    int j = 1024;
    float j_float = 1.0;
    int i = 0;
    
    for (int k =0; k< 1024; k++){ 
        myfile<<j;
        myfile<<i;
        j -= 1;
        i += 1;
        Time::delay(1.0);
        printf("*\n");
    }



    /*
    while(i<1){
        //cout<<i<<" "<<j<<endl;
        
        myfile<<i;
        //myfile<<j;
        
        j += 1;
        i += 1;
        
    }
    */
    

    /*
    // echo from microphone to headphones, superimposing an annoying tone   
    double vv=0;
    while(true){   
        Sound s;   
        get->getSound(*sRead);   
        for (int i=0; i<sRead->getSamples(); i++) {   
            double now = Time::now();   
            static double first = now;   
            now -= first;   
            if ((long int) (now*2) % 2 == 0) {   
                vv += 0.08;   
            } else {   
                vv += 0.04;   
            }   
            double dv = 500*sin(vv);   
            for (int j=0; j<sRead->getChannels(); j++) {   
                int v =sRead->get(i,j);  
                s.set((int)(v+dv+0.5),i,j);   
            }   
        }   
        put->renderSound(s);   
    }
    */
    
    myfile.close();
    cout<<"myFile correctly closed"<<endl;
    pReceiver.close();
    pSender.close();

    cout<<"closing the network"<<endl;
    Network::fini();
    
    return 1;

}
