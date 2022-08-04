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


#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/sig/SoundFile.h>

#include <deque>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <math.h>

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::sig::file;

#define INTERFACE

int padding = 0;

class Echo : public TypedReaderCallback<Sound> {
private:
    PolyDriver poly;
    IAudioRender *put;
    BufferedPort<Sound> port;
    BufferedPort<Bottle> outport;
    Semaphore mutex;
    bool muted;
    bool saving;
    std::deque<Sound> sounds;
    int samples;
    int channels;

public:
    Echo() : mutex(1) {
        put = NULL;
        port.useCallback(*this);
        port.setStrict();
        muted = false;
        saving = true;
        samples = 0;
        channels = 0;
        put = NULL;
    }

    bool open(Searchable& p) {
        bool dev = true;
        if (p.check("nodevice")) {
            dev = false;
            printf("no device! \n");
        }
        if (dev) {
            poly.open(p);
            if (!poly.isValid()) {
                printf("cannot open driver\n");
                return false;
            }
            
            if (!p.check("mute")) {
                // Make sure we can write sound
                poly.view(put);
                if (put==NULL) {
                    printf("cannot open interface\n");
                    return false;
                }
            }
        }
            
        port.setStrict(true);
        if (!port.open(p.check("name",Value("/audioAttention/yarphear")).asString())) {
            printf("Communication problem\n");
            return false;
        }
        else{
            printf("success in opening yarphear \n");
        }        
        if (!outport.open(p.check("outportname",Value("/audioAttention/left:o")).asString())) {
            printf("Communication problem\n");
            return false;
        }
        else{
            printf("success in opening yarphear \n");
        }
        
        if (p.check("remote")) {
            Network::connect(p.check("remote",Value("/grabber")).asString(),
                             port.getName());
            printf("successfully connected to the /grabber \n");
        }
        else {
            printf("success in connecting no remote \n");
        }

        return true;
    }

    void onRead(Sound& sound)
     {
        #ifdef TEST
        //this block can be used to measure time elapsed between two sound packets
        static double t1= yarp::os::Time::now();
        static double t2= yarp::os::Time::now();
        t1= yarp::os::Time::now();
        printf("onread %f\n", t2-t1);
        t2 = yarp::os::Time::now();
        #endif
        printf("onread \n");
        int ct = port.getPendingReads();
        //printf("pending reads %d\n", ct);
        while (ct>padding) {
            ct = port.getPendingReads();
            printf("Dropping sound packet -- %d packet(s) behind\n", ct);
            port.read();
        }
        mutex.wait();
        /*
          if (muted) {
          for (int i=0; i<sound.getChannels(); i++) {
          for (int j=0; j<sound.getSamples(); j++) {
          sound.put(0,j,i);
          }
          }
          }
        */
        if (!muted) {
            if (put!=NULL) {
                put->renderSound(sound);
            }
        }
        if (true) {
            saveFrame(sound);
        }

        mutex.post();
        Time::yield();
    }

    void mute(bool muteFlag=true) {
        mutex.wait();
        muted = muteFlag;
        mutex.post();
    }

    void save(bool saveFlag=true) {
        mutex.wait();
        saving = saveFlag;
        mutex.post();
    }

    void saveFrame(Sound& sound) {
        if(sounds.size() < 10){
        printf("saving the frame \n");
        sounds.push_back(sound);
        samples += sound.getSamples();
        channels = sound.getChannels();
        printf("  %ld sound frames buffered in memory (%ld samples)\n", 
               (long int) sounds.size(),
               (long int) samples);
        }
        else {
            printf("saving limit reached \n");
        }
    }

    bool saveFile(const char *name) {

        

        mutex.wait();
        //saving = false;

        Sound total;
        total.resize(samples,channels);
        long int at = 0;
        while (!sounds.empty()) {
            Sound& tmp = sounds.front();
            for (int i=0; i<channels; i++) {
                for (int j=0; j<tmp.getSamples(); j++) {
                    total.set(tmp.get(j,i),at+j,i);
                }
            }
            total.setFrequency(tmp.getFrequency());
            at += tmp.getSamples();
            sounds.pop_front();
        }
        mutex.post();
        bool ok = write(total,name);
        if (ok) {
            printf("Wrote audio to %s\n", name);
        }
        samples = 0;
        channels = 0;
        return ok;
    }

     bool sendFrame() {
         //printf("sending the frame %d %d \n", samples, channels);
        bool ok = true;

        //-------------------------------------------------------
        mutex.wait();
        //saving = false;

        Sound total;
        total.resize(samples,channels);
        long int at = 0;
        while (!sounds.empty()) {
            Sound& tmp = sounds.front();
            for (int i=0; i<channels; i++) {
                for (int j=0; j<tmp.getSamples(); j++) {
                    total.set(tmp.get(j,i),at+j,i);
                    //printf("%f", tmp.get(j,i));
                }
            }
            total.setFrequency(tmp.getFrequency());
            at += tmp.getSamples();
            sounds.pop_front();
        }
        mutex.post();
        //------------------------------------------------------
        
        
        at = 0;
        if (outport.getOutputCount()) {
            //Sound& sport =  outport.prepare();
            //sport.resize(samples, channels);
           
            //for (int i=0; i<channels; i++) {
            int i = 0;
            unsigned int result = 0;
            string str;
            unsigned char *pSound = total.getRawData();
            int bytesPerSample = total.getBytesPerSample();
            int rawDataSize    = total.getRawDataSize();
            
            //printf("bytesPerSample %d rawDataSize %d \n",bytesPerSample,rawDataSize  );
            
            for (int j=0; j<total.getSamples(); j++) {
                //sport.set(total.get(j,i),at+j,i);
                
                result = 0;
                Bottle& data = outport.prepare();
                data.clear();
                result = *pSound;
                //printf(" %d ", *pSound);
                pSound++;
                if(*pSound != 255) {
                    //printf(" %d ", *pSound);
                    result += *pSound << 8;
                    //printf(" %d", result);
                    //int t = total.get(j,i);
                    //str.append((const char *) pSound);
                    data.addInt16(result);
                    outport.writeStrict();
                }
                //printf("\n");
                pSound++;
            }
            //} 
            //sport.setFrequency(total.getFrequency());
            //outport.write();
        }
        
        
        //bool ok = write(total,name);
        //if (ok) {
        //    printf("Wrote audio to %s\n", name);
        //}

        samples = 0;
        channels = 0;
        return ok;
    }

    bool close() {
        port.close();
        mutex.wait(); // onRead never gets called again once it finishes
        return true;
    }
};

 int main(int argc, char *argv[]) {

     // Open the network
     Network yarp;

     BufferedPort<Bottle> outputPort1;
     BufferedPort<Bottle> outputPort2;   
     outputPort1.open("/audioGrabber/audio1:o");
     outputPort2.open("/audioGrabber/audio2:o");

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

    
    //opening the polyRender
    /*
    PolyDriver polyRender(conf);
    if(!polyRender.isValid()) {
        printf("cannot open write-interface \n");
        return 1;
    }
    IAudioRender *put;
    */


    // Get a portaudio read device.
    Property conf2;
    conf2.put("device","portaudio");
    conf2.put("read", "1");
    conf2.put("samples", 4096);
    conf2.put("rate", 16000);
    
    //opening the polyGrabber
    /*
    PolyDriver polyGrabber(conf2);
    if(!polyGrabber.isValid()) {
        printf("cannot open read-interface \n");
        return 1;
    }
    IAudioGrabberSound *get;
    */
    
    // see if user has supplied audio device
    Property p;
    if (argc>1) {
        p.fromCommand(argc,argv);
    }

    // otherwise default device is "portaudio"
    if (!p.check("device")) {
        p.put("nodevice",1);
        p.put("remote", "/grabber");
        //p.put("device","portaudio");
        //p.put("write",1);
        //p.put("delay",1);
    }
    Echo echo;
    echo.open(p);

    
    /*
    // Make sure we can write sound
    polyRender.view(put);
    if (put==NULL) {
        printf("cannot open interface\n");
        return 1;
    }
    */

    //Receive and render
    Sound *sRead = new Sound();
    
    //while (true){
    //    sRead = pReceiver.read(false);
    //    if (sRead!=NULL)
    //        put->renderSound(*sRead);
    //  }
    //return 0; 
    
    
    // Make sure we can read sound
    /*
    polyGrabber.view(get);
    if (get==NULL) {
        printf("cannot open interface\n");
        return 1;
    }
    else{
        printf("able to open interface \n");
    }
    */


    //**********************************************************************************//
    //Grab and send
    //Sound *sWrite;
    /*
      while (true)
      {
        get->getSound(*sWrite);
        //pSender.write(sWrite);
      }
    return 0;
    */

    //printf("starting the send Frame mechanism \n");
    while(true) {
        echo.sendFrame();
        //Time::delay(0.9);
    }
    

    //*************************************************************************************//
    /*
    printf("data dumping \n");

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
    
    for (int k =0; k< 10; k++){ 
        myfile<<j;
        myfile<<i;
        j -= 1;
        i += 1;
        Time::delay(0.1);
    }
    */


    /*
    while(i<1){
        //cout<<i<<" "<<j<<endl;
        
        myfile<<i;
        //myfile<<j;
        
        j += 1;
        i += 1;
        
    }
    */
    
    //*****************************************************************//

    
    /******************************************************************/

#ifndef INTERFACE 
    // echo from microphone to headphones, superimposing an annoying tone   
    double vv=0;
    int left = 0;
    int right = 0;
    int timestamp = 0;
    while(false){

        if(get->getSound(*sRead)) {
            for (int i=0; i<sRead->getSamples(); i++) {   
                //double now = Time::now();   
                //static double first = now;   
                //now -= first;   
                //if ((long int) (now*2) % 2 == 0) {   
                //    vv += 0.08;   
                //} else {   
                //    vv += 0.04;   
                //}   
                //double dv = 500*sin(vv);   
                //for (int j=0; j<sRead->getChannels(); j++) {   
                //int v =sRead->get(i,j);  
                //s.set((int)(v+dv+0.5),i,j);   
                //}   
                
            
            
                if(sRead->getChannels() == 2) {
                    left  = sRead->get(i,0);
                    //right = sRead->get(i,1);    
                    //timestamp++; // = Stamp::getTime();
                }
    
            
                
                if ((outputPort1.getOutputCount()) && (true) ){
                    if (true) {            
                        Bottle& toSend = outputPort1.prepare();
                        toSend.clear();
                        toSend.addFloat32(left);
                        //toSend.addInt16(right);
                        //toSend.addInt16(timestamp);
                        outputPort1.write(); 
                    }
                    else {
                        Bottle& toSend = outputPort2.prepare();
                        toSend.clear();
                        toSend.addInt16(left++);
                        toSend.addInt16(right++);
                        toSend.addInt16(timestamp++);
                        outputPort2.write();
                    }
                }
                
    
            }   
            // put->renderSound(s);   
       
        }
        else {
            printf("getSound error \n");
        }
        
    }
    
#endif

    
    myfile.close();
    cout<<"myFile correctly closed"<<endl;

    pReceiver.close();
    pSender.close();

    
    outputPort1.close();
    outputPort2.close();
    
    cout<<"closing the network"<<endl;
    Network::fini();
    
    return 1;

}
