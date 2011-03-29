// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
 * website: www.robotcub.org 
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
 * Public License fo
 r more details
 */

/**
 * @file populatorThread.cpp
 * @brief Implementation of the thread (see header populatorThread.h)
 */

#include <iCub/populatorThread.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100
#define OBLIVIONFACTOR 10

populatorThread::populatorThread() : RateThread(THRATE) {
    count = 0;
    
}

populatorThread::~populatorThread() {
}

bool populatorThread::threadInit() {
    databasePort.open(getName("/database").c_str());
    guiPort.open(getName("/gui:o").c_str());
    return true;
}

void populatorThread::interrupt() {
    databasePort.interrupt();
    guiPort.interrupt();
}

void populatorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string populatorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void populatorThread::run() {
    count ++;
    
    if(guiPort.getOutputCount()) {
        Bottle writer, reader;
        Bottle writer2, reader2;
        writer.clear(); reader.clear();
        Bottle list;
        list.clear();
        //cleaning the iCubGui
        Bottle& obj = guiPort.prepare();
        obj.clear();
        obj.addString("reset");
        guiPort.write();
        
        //asking for the list of all the object in the objectsPropertiesCollector
        writer.addVocab(VOCAB3('a','s','k'));
        Bottle& listAttr=writer.addList();
        listAttr.addString("all");
        //writer.append(listAttr);
        databasePort.write(writer,reader);
        //int v = reader.pop().asVocab();
        cout<<"reader:"<<reader.toString()<<endl;
        if(reader.get(0).asVocab()==VOCAB3('a','c','k')) {
            cout<<"reader:"<<reader.toString()<<" size:"<<reader.size()<<endl;
            if(reader.size()<=1) {
                printf("No Objects Found \n");
                return;
            }
            Bottle* completeList=reader.get(1).asList();
            Bottle* list = completeList->get(1).asList();
            int size = list->size();
            cout<<"list size:"<<size<<endl;
            // list of ids
            int j = 0;
            double posX, posY, posZ, lifeTimer;
            int r, g, b;
            //starting for the list of ids, extract properties of all the object
            while( j < size) {
                int id = list->get(j++).asInt();
                cout<<id<<", ";
                writer2.clear(); reader2.clear();
                writer2.addVocab(VOCAB3('g','e','t'));
                Bottle& listAttr=writer2.addList();
                //listAttr.addList(listAttr);
                Bottle& listId=listAttr.addList();
                listId.addString("id");
                listId.addInt(id);
                cout<<writer2.toString()<<endl;
                databasePort.write(writer2,reader2);
                if(reader2.get(0).asVocab()==VOCAB3('a','c','k')) {
                    //cout << "reader:" << reader2.toString() << endl;
                    Bottle* list = reader2.get(1).asList();
                    //cout << "list:" << list->toString() << endl;
                    posX = list->find("x").asDouble();
                    posY = list->find("y").asDouble();
                    posZ = list->find("z").asDouble();
                    //printf("position: %f,%f,%f \n", posX,posY,posZ);
                    
                    r = list->find("r").asInt();
                    g = list->find("g").asInt();
                    b = list->find("b").asInt();
                    //printf("colour: %d,%d,%d \n", r,g,b);

                    lifeTimer = list->find("lifeTimer").asDouble();
                    //printf("lifeTimer %f \n",lifeTimer);
                    
                    Bottle& obj = guiPort.prepare();
                    obj.clear();
                    string name("obj");
                    sprintf((char*)name.c_str(),"Object%d",id);
                    //printf("name of the object:%s \n",name.c_str());
                    obj.addString("object"); // comando
                    obj.addString(name.c_str()); // nome dell'oggetto
                    
                    // object dimension in millimeters 
                    // it draws an ellips with a the name close by
                    // pay attention to the order!!!!!!!
                    obj.addDouble(55.0); 
                    obj.addDouble(55.0); 
                    obj.addDouble(55.0);
                    // position of the objects in millimeters!
                    // (pay attention to the order!!!!!!)
                    // frame of reference locate with the Z axis toward the ceiling, the X axis pointing into the heaps,
                    // and the Y axis directed to the right hand side of the robot 
 
                    obj.addDouble(posZ);
                    obj.addDouble(posX);
                    obj.addDouble(posY);
                    // orientation of the object (roll, pitch,yaw) 
                    // in gradi 
                    obj.addDouble(45.0);
                    obj.addDouble(0.0);
                    obj.addDouble(45.0);
                    // colour of the object (0-255)
                    obj.addInt(r);
                    obj.addInt(g);
                    obj.addInt(b);
                    // trasparency of the object (0.0=invisible 1.0=solid) 
                    if(lifeTimer == 0)
                        obj.addDouble(1.0);
                    else
                        obj.addDouble((lifeTimer / OBLIVIONFACTOR) + 0.05);
                    guiPort.write();
                }
            }
            cout<<endl;
        }
    }
}

void populatorThread::threadRelease() {
    //closing ports
    databasePort.close();
    guiPort.close();
}

