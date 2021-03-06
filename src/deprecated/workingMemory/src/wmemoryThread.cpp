// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2014 RBCS Robotics Brain and Cognitive Science
 * Authors: Rea Francesco
 * email:   francesco.rea@iit.it
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
 * @brief Implementation of the thread (see header wmemoryThread.h)
 */

#include <iCub/wmemoryThread.h>
#include <cstring>
#include <cassert>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 100
#define OBLIVIONFACTOR 10
#define INCREMENT 10
#define DECREMENT 2

wmemoryThread::wmemoryThread() : RateThread(THRATE) {
    targetReady = false;
    count    = 0; 
    numNames = 0;
    for (int i=0; i< MAXBUFFERDIMENSION; i++) {
        cName[i]=0;
    }
}

wmemoryThread::~wmemoryThread() {

}

bool wmemoryThread::threadInit() {
    databasePort.open(getName("/database:o").c_str());
    guiPort.open(getName("/gui:o").c_str());
    texPort.open(getName("/textures:o").c_str());    
    return true;
}

void wmemoryThread::interrupt() {
    databasePort.interrupt();
    guiPort.interrupt();
    texPort.interrupt();
}

void wmemoryThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string wmemoryThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

const void wmemoryThread::setTarget(const Bottle& _target){
    targetMutex.wait();
    target = _target;
    targetReady = true;
    targetMutex.post();
}

const void wmemoryThread::parseTarget(Bottle& list){
    //Bottle* list = reader2.get(1).asList();
    cout << "list:" << list.toString() << endl;
    posX = list.find("x").asDouble();
    posY = list.find("y").asDouble();
    posZ = list.find("z").asDouble();
    printf("position: %f,%f,%f \n", posX,posY,posZ);
    
    r = list.find("r").asDouble();
    g = list.find("g").asDouble();
    b = list.find("b").asDouble();
    printf("colour: %f,%f,%f \n", r,g,b);
    
    lifeTimer = list.find("lifeTimer").asDouble();
    printf("lifeTimer %f \n",lifeTimer);   
}

bool wmemoryThread::checkTarget(const yarp::os::Bottle& target){
    printf("checking the target %s\n", target.toString().c_str());
    //extracting target characteristics
    Bottle* xList    = target.get(0).asList();
    Bottle* yList    = target.get(1).asList();
    Bottle* zList    = target.get(2).asList();
    Bottle* rList    = target.get(3).asList();
    Bottle* gList    = target.get(4).asList();
    Bottle* bList    = target.get(5).asList();
    Bottle* lifeList = target.get(6).asList();

    double targetX = xList->get(1).asDouble();
    double targetY = yList->get(1).asDouble();
    double targetZ = zList->get(1).asDouble();
    double targetR = rList->get(1).asDouble();
    double targetG = gList->get(1).asDouble();
    double targetB = bList->get(1).asDouble();
    double lifeTim = lifeList->get(1).asDouble();
    
    printf("targetX %f \n", targetX);
    printf("targetY %f \n", targetY);
    printf("targetZ %f \n", targetZ);
    printf("targetR %f \n", targetR);
    printf("targetG %f \n", targetG);
    printf("targetB %f \n", targetB);
    printf("lifeTim %f \n", lifeTim);

    targetReady = false;
    Bottle writer, reader;
    Bottle writer2, reader2;
    writer.clear(); reader.clear();
    Bottle list;
    list.clear();
    
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
            return true;
        }
        Bottle* completeList = reader.get(1).asList();
        Bottle* list = completeList->get(1).asList();
        int size = list->size();
        cout<<"list size:"<<size<<endl;
        if (size == 0){
            //empty list obvioulsly novel
            return true;
        }

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
                printf("position: %f,%f,%f \n", posX,posY,posZ);
                
                r = list->find("r").asDouble();
                g = list->find("g").asDouble();
                b = list->find("b").asDouble();
                printf("colour: %d,%d,%d \n", r,g,b);
                
                lifeTimer = list->find("lifeTimer").asDouble();
                printf("lifeTimer %f \n",lifeTimer);   
                
                
            }
        }
    }
    printf("checking the target \n \n");
    return true;
}

const void wmemoryThread::memorizeTarget(){
    if(databasePort.getOutputCount()){
        printf("updating the database \n");
        //adding novel position to the 
        Bottle request, reply;
        request.clear(); reply.clear();
        request.addVocab(VOCAB3('a','d','d'));
        Bottle& listAttr=request.addList();
        
        Bottle& sublistX = listAttr.addList();
        
        sublistX.addString("x");
        sublistX.addDouble(posX);    
        listAttr.append(sublistX);
        
        Bottle& sublistY = listAttr.addList();
        sublistY.addString("y");
        sublistY.addDouble(posY);      
        listAttr.append(sublistY);
        
        Bottle& sublistZ = listAttr.addList();            
        sublistZ.addString("z");
        sublistZ.addDouble(posZ);   
        listAttr.append(sublistZ);
        
        Bottle& sublistR = listAttr.addList();
        sublistR.addString("r");
        sublistR.addDouble(r);
        listAttr.append(sublistR);
        
        Bottle& sublistG = listAttr.addList();
        sublistG.addString("g");
        sublistG.addDouble(g);
        listAttr.append(sublistG);
        
        Bottle& sublistB = listAttr.addList();
        sublistB.addString("b");
        sublistB.addDouble(b);
        listAttr.append(sublistB);
        
        Bottle& sublistLife = listAttr.addList();
        sublistLife.addString("lifeTimer");
        sublistLife.addDouble(lifeTimer);
        listAttr.append(sublistLife);        
  
        databasePort.write(request, reply);

        printf("updating the database reply: %s \n", reply.toString().c_str());
    }
}

void wmemoryThread::run() {
    count ++;
    bool novel;
    bool readyTarget;

    targetMutex.wait();
    readyTarget = targetReady;
    targetMutex.post();
    
    if(readyTarget){
        targetMutex.wait();
        novel = checkTarget(target);
        targetMutex.post();

        if(novel){
            // getting the data out of the target
            parseTarget(target);
            memorizeTarget();
            
        }
        printf("----------------------------------------------------\n");
    }

    

    /*
    if(guiPort.getOutputCount()) {
        Bottle writer, reader;
        Bottle writer2, reader2;
        writer.clear(); reader.clear();
        Bottle list;
        list.clear();
        
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
                    printf("position: %f,%f,%f \n", posX,posY,posZ);
                    
                    r = list->find("r").asInt();
                    g = list->find("g").asInt();
                    b = list->find("b").asInt();
                    printf("colour: %d,%d,%d \n", r,g,b);

                    lifeTimer = list->find("lifeTimer").asDouble();
                    printf("lifeTimer %f \n",lifeTimer);   
                           
                    if(guiPort.getOutputCount()) {

                        Bottle& obj = guiPort.prepare();
                        obj.clear();
                        string name("");
                        sprintf((char*)name.c_str(),"Object%d",id);
                    
                        int len;
                        if (id>1000) {
                            len = 6 + 4 ;
                        }
                        else if( id > 100) {
                            len = 6 + 3;
                        }
                        else if( id > 10) {
                            len = 6 + 2;
                        }
                        else if( id < 10) {
                            len = 6 + 1;
                        }
                    
                        bool found = checkNames(id);
                        //bool found = false;
                        if(!found) {                                            
                            //adding the object to the GUI
                            printf("!found numName=%d \n", numNames);
                            printf("dimension :%d \n",len);
                            obj.addString("object"); // command
                            obj.addString(name.c_str()); // object name
                            
                            // object dimension in millimeters 
                            // it draws an ellips with a the name close by
                            // pay attention to the order!!!!!!!
                            obj.addDouble(5.0); 
                            obj.addDouble(155.0); 
                            obj.addDouble(155.0);
                            // position of the objects in millimeters!
                            // (pay attention to the order!!!!!!)
                            // frame of reference locate with the Z axis toward the ceiling, the X axis pointing into the hip,
                            // and the Y axis directed to the right hand side of the robot  
                            obj.addDouble(posX);
                            obj.addDouble(posY);
                            obj.addDouble(posZ);
                            // orientation of the object (roll, pitch,yaw) 
                            // in gradi 
                            obj.addDouble(0.0);
                            obj.addDouble(0.0);
                            obj.addDouble(0.0);
                            // colour of the object (0-255)
                            obj.addInt(r);
                            obj.addInt(g);
                            obj.addInt(b);
                            // trasparency of the object (0.0=invisible 1.0=solid) 
                            //if(lifeTimer == 0)
                            obj.addDouble(1.0);
                            //else
                            //obj.addDouble((lifeTimer / OBLIVIONFACTOR) + 0.05);
                            guiPort.writeStrict();
                        }
                        printf("object written on the guiPort \n");
                      
                        
                        Bottle& texture = list->findGroup("texture");                    
                        if ((!texture.isNull()) && (texPort.getOutputCount())) {
                            printf("looking for the texture \n");
                            Bottle* templateBottle = texture.get(1).asList();
                            printf("dimension of the template %d \n", templateBottle->size());
                            int dimTemplate = templateBottle->size();
                            //sending information on the texture
                            yarp::sig::VectorOf<unsigned char>& tex=texPort.prepare();
                            
                            
                            //unsigned char garbage[53];
                            //fread(garbage,1,53,img); // throws pgm header
                            
                            int dimHeader = 11;
                            unsigned char buffer[9529]; 
                            unsigned char* pbuffer = &buffer[0];
                            int pwidth =  templateBottle->get(0).asInt();
                            int pheight =  templateBottle->get(1).asInt();
                            
                            
                            
                            buffer[0]=pwidth;   // width
                            buffer[1]=pheight;  // height
                            
                            buffer[2]='O';
                            buffer[3]='b';
                            buffer[4]='j';
                            buffer[5]='e';
                            buffer[6]='c';
                            buffer[7]='t';
                            pbuffer += 8;
                            char* pointerName = (char*) name.c_str();
                            pointerName += 6;
                            printf("name size %d", len);
                            for (int i = 6; i < len; i++) {
                                *pbuffer = *pointerName;
                                printf("[%c,%d]  ", *pointerName,*pointerName);
                                pointerName++; pbuffer++;
                            }
                            printf("\n");
                            *pbuffer++ = 0;
                        
                            //printf("chars \n");
                            for (int i = 2; i < dimTemplate; i++) {
                                int value = templateBottle->get(i).asInt();
                                unsigned char c = (unsigned char) value;
                                *pbuffer++ = c;
                                //printf("%d ",c);
                            }
                            
                            //printf("sent \n");
                            tex.clear();
                            for (int i=0; i< dimTemplate + dimHeader; ++i) {
                                tex.push_back(buffer[i]);
                                //printf("%d ",buffer[i]);
                            }                 
                            texPort.write();
                        } //endif texture

                        //adding the object to the list                         
                        printf("adding the object ot the list... \n");
                        listNames[numNames] =  id;
                        cName[numNames] += INCREMENT;
                        printf("added the new name : %d ", listNames[numNames]);
                        numNames++;
                    } // endif found                                      
                }
            }
            cout<<endl;
            //cleaning the iCubGui
            cleanNames();
            //Bottle& obj = guiPort.prepare();
            //obj.clear();
            //obj.addString("reset");
            //guiPort.write();
        }
    }
    */
}

void wmemoryThread::cleanNames() {
    int position;
    string name("");
    bool removed = false;
    position =  numNames;
    for (int i = 0; i< numNames; i++) {
        cName[i] -= DECREMENT;
        printf("cName[%d]=%d \n", i, cName[i]);
        if(cName[i] <= 0) {
            Bottle& obj = guiPort.prepare();
            obj.clear();
            obj.addString("delete");
            sprintf((char*)name.c_str(),"Object%d",listNames[i]);
            printf("deleting the object %s \n ", name.c_str());
            obj.addString(name.c_str());
            guiPort.write();
            position = i;
            removed = true;
            break;
        }        
    }
    if (removed) {
        for (int i= position; i < numNames - 1; i++) { 
            listNames[i] = listNames[i+1];
            cName[i] = cName[i+1];
        }
        numNames --;
    }
}

bool wmemoryThread::checkNames(short str) {
    printf("checking name %d in list dim %d \n", str, numNames);    
    for (int i = 0; i< numNames; i++) {
        printf("checking against %d \n",listNames[i]); 
        if(listNames[i] == str) {
            cName[i] += INCREMENT; 
            return true;
        }
    }
    return false;
}

void wmemoryThread::threadRelease() {
    //closing ports
    databasePort.close();
    guiPort.close();
    texPort.close();
}

