// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
  * Copyright (C)2014  Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
  * Author:Francesco Rea
  * email: francesco.reak@iit.it
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
 * @file audioInterfaceRatethread.cpp
 * @brief Implementation of the eventDriven thread (see audioInterfaceRatethread.h)
 */

#include <iCub/audioInterfaceRatethread.h>
#include <cstring>

using namespace yarp::dev;
using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100 //ms

/*
std::size_t getSizeOfFile(std::ofstream *fs) {
        if (!fs) {
            return 0;
        }
        std::streampos currPos = fs->tellp();
        fs->seekp (0, fs->end);
        std::size_t size = static_cast<std::size_t>(fs->tellp());
        fs->seekp (currPos);
        return size;
    }
*/

std::size_t getSizeOfFile(std::fstream *fs) {
        if (!fs) {
            return 0;
        }
        std::streampos currPos = fs->tellg();
        fs->seekg (0, fs->end);
        std::size_t size = static_cast<std::size_t>(fs->tellg());
        fs->seekg (currPos);
        return size;
    }



audioInterfaceRatethread::audioInterfaceRatethread() {
    robot = "icub";        
}

audioInterfaceRatethread::audioInterfaceRatethread(string _robot, string _configFile){
    robot = _robot;
    configFile = _configFile;
}

audioInterfaceRatethread::~audioInterfaceRatethread() {
    // do nothing
}

bool audioInterfaceRatethread::threadInit() {

    string str = getName("dumpFile.txt");
    printf("\n opening the file name %s \n", str.c_str());
    dumpFile = fopen("dumpFile.txt","wb");
    //ofstream myfile;
    myfile.open ("./example.bin", ios::out | ios::app);
    if (myfile.is_open()) { 
         cout<<"file correctly opened"<<endl;
        
     }
     else{
         cout<<"file is not opened"<<endl;
         return 0;
     }
    if (dumpFile!=NULL){
        printf("Opened \n");
    }
    else {
        printf("Error in opening \n");
    }
    
    int numIter = 3;
    char defaultValue = 0;
    printf("int size %d ", sizeof(int));
    for(int i = 0; i < 7000 * 4096 * 3 * 4; i++) {
        //myfile<<defaultValue;
        fprintf(dumpFile,"%c",0);
    }
    myfile.close();
    //fclose(dumpFile);

    myInputFile.open ("./example.bin", ios::in | ios::binary);
    int sizeOfFile = getSizeOfFile(&myInputFile);
    myInputFile.seekg (0, myInputFile.beg);
    int length = 4 * numIter;
    char * buffer = new char [4];
    myInputFile.read(buffer, 4);
    
    for(int i = 0; i < numIter; i++){
        printf("%d ", buffer[i]);
    }
    printf("\n Done with the initialization %d \n", sizeOfFile );


    if (!inputPort.open(getName("/audio:i").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }
   
    if (!outputPort.open(getName("/audio:o").c_str())) {
        cout << ": unable to open port to send unmasked events "  << endl;
        return false;  // unable to open; let RFModule know so that it won't run
    }    

    return true;
    

}

void audioInterfaceRatethread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}


std::string audioInterfaceRatethread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void audioInterfaceRatethread::setInputPortName(string InpPort) {
    
}

void audioInterfaceRatethread::run() {    
    int a, b, c;
    int counter = 0;

    unsigned char s_a; 
    unsigned char s_b; 
    unsigned char s_c; 
    unsigned char s_d; 

    int mask_a = 0x000000FF;
    int mask_b = 0x0000FF00;
    int mask_c = 0x00FF0000;
    int mask_d = 0xFF000000;

    fseek(dumpFile, 1, SEEK_SET);

    while(!isStopping()){

        //code here ....
        if(inputPort.getInputCount()){
            
            counter++;

            //fseek(dumpFile, 10, SEEK_SET);
            
            Bottle* read = inputPort.read(true);    

            printf("bottleSize %d \n", read->size());
            for(int i= 0; i < 4096 * 3; i++){
                a = read->get(i).asInt16();
                s_a = a & mask_a;
                s_b = (a & mask_b) >> 8;
                s_c = (a & mask_c) >> 16;
                s_d = (a & mask_d) >> 24;
                fprintf(dumpFile,"%c%c%c%c",s_a,s_b,s_c,s_d);
            }

            /*
            b = read->get(1).asInt16();
            c = read->get(2).asInt16();

            s_a = b & mask_a;
            s_b = b & mask_b >> 8;
            s_c = b & mask_c >> 16;
            s_d = b & mask_d >> 24;
            fprintf(dumpFile,"%c%c%c%c",s_a,s_b,s_c,s_d);
            
            s_a = c & mask_a;
            s_b = c & mask_b >> 8;
            s_c = c & mask_c >> 16;
            s_d = c & mask_d >> 24;
            fprintf(dumpFile,"%c%c%c%c",s_a,s_b,s_c,s_d);
            */
            
            printf("received %d bytes \n", counter * 4096 * 4);
            //printf("%s \n", read->toString().c_str());
            
        }
        /*
        if (outputPort.getOutputCount()) {
            outputPort.prepare() = *inputImage;
            outputPort.write();  
        }
        */
    }
    
    printf("received %d bytes \n", counter * 3 * 4);
                  
}

void audioInterfaceRatethread::onStop() {
    printf("closing the files \n");
    //myfile.close();
    fclose(dumpFile);

    printf("closing the ports \n");
    //outputPort.interrupt();   
    //inputPort.interrupt();
    //outputPort.close();
    //inputPort.close();
}

void audioInterfaceRatethread::threadRelease() {
    inputPort.close();
    outputPort.close();
}


