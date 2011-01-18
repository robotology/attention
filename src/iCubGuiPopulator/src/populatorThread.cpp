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

#define THRATE 10
#define OBLIVIONFACTOR 100

populatorThread::populatorThread() : RateThread(THRATE) {
    
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
            cout<<"reader:"<<reader.toString()<<endl;
            Bottle* completeList=reader.get(1).asList();
            Bottle* list = completeList->get(1).asList();
            cout<<"list:"<<list->toString()<<endl;
            int size = list->size();
            cout<<"list size:"<<size<<endl;
            // list of ids
            int j = 0;
            double posX, posY, posZ, r, g, b, lifeTimer;
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
                    cout << "reader:" << reader2.toString() << endl;
                    Bottle* list = reader2.get(1).asList();
                    cout << "list:" << list->toString() << endl;
                    //TODO list->find
                    posX = list->find("x").asDouble();
                    posY = list->find("y").asDouble();
                    posZ = list->find("z").asDouble();
                    printf("position: %f,%f,%f \n", posX,posY,posZ);
                    
                    r = list->find("r").asDouble();
                    g = list->find("g").asDouble();
                    b = list->find("b").asDouble();
                    printf("colour: %f,%f,%f \n", r,g,b);

                    lifeTimer = list->find("lifeTimer").asDouble();
                    printf("lifeTimer %f \n",lifeTimer);
                    
                    Bottle& obj = guiPort.prepare();
                    obj.clear();
                    string name("object");
                    sprintf((char*)name.c_str(),"Object%d",id);
                    printf("name of the object:%s \n",name.c_str());
                    obj.addString("object"); // comando
                    obj.addString(name.c_str()); // nome dell'oggetto
                    
                    // dimensioni dell'oggetto (viene visualizzato come un ellissoide con il nome accanto) 
                    // in millimetri 
                    obj.addDouble(55.0); 
                    obj.addDouble(55.0); 
                    obj.addDouble(55.0);
                    // posizione dell'oggetto
                    // in millimetri
                    // il sistema di riferimento ha l'asse Z verso l'alto, l'asse X in avanti e l'asse Y verso sinistra 
                    obj.addDouble(posX);
                    obj.addDouble(posY);
                    obj.addDouble(posZ);
                    // orientazione dell'oggetto (roll, pitch,yaw) 
                    // in gradi 
                    obj.addDouble(45.0);
                    obj.addDouble(0.0);
                    obj.addDouble(45.0);
                    // colore dell'oggetto (0-255)
                    obj.addInt(r);
                    obj.addInt(g);
                    obj.addInt(b);
                    // trasparenza (0.0=invisibile 1.0=solido) 
                    if(lifeTimer == 0)
                        obj.addDouble(1.0);
                    else
                        obj.addDouble(lifeTimer / OBLIVIONFACTOR);
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

