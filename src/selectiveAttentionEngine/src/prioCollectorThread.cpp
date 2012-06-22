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
 * @file prioCollectorThread.cpp
 * @brief Implementation of the thread of prioritiser Collector(see header prioCollectorThread.h)
 */

#include <iCub/prioCollectorThread.h>
#include <cstring>

using namespace yarp::os;
using namespace yarp::sig;
using namespace std;

#define THRATE 10

inline void copy_8u_C1R(ImageOf<PixelMono>* src, ImageOf<PixelMono>* dest) {
    int padding = src->getPadding();
    int channels = src->getPixelCode();
    int width = src->width();
    int height = src->height();
    unsigned char* psrc = src->getRawImage();
    unsigned char* pdest = dest->getRawImage();
    for (int r=0; r < height; r++) {
        for (int c=0; c < width; c++) {
            *pdest++ = (unsigned char) *psrc++;
        }
        pdest += padding;
        psrc += padding;
    }
}

inline void crossAssign(unsigned char* p,int value, int rowsize) {
    
    // in the conversion between cartesian and logpolar the single maxresponse pixel can go lost
    // enhancing the response of the neightbourhood 
    *p = value;
    p++;              *p = value;
    p -= 2;           *p = value;
    p += 1 + rowsize; *p = value;
    p -= 2 * rowsize; *p = value;
    p += rowsize;
 
} 

prioCollectorThread::prioCollectorThread() {
    k1 = 0.5; 
    k2 = 0.5;
}

prioCollectorThread::~prioCollectorThread() {
    
}

bool prioCollectorThread::threadInit() {
    printf("starting the thread.... \n");
    /* open ports */
    string rootName("");
    rootName.append(getName("/cmd:i"));
    printf("opening ports with rootname %s .... \n", rootName.c_str());
    inCommandPort.open(rootName.c_str());

    string testName("");
    testName.append(getName("/test2:o"));
    printf("opening ports with rootname %s .... \n", testName.c_str());
    testPort.open(testName.c_str());
    
    return true;
}

void prioCollectorThread::interrupt() {
    inCommandPort.interrupt();
}

void prioCollectorThread::setName(string str) {
    this->name=str;
    printf("name: %s", name.c_str());
}

std::string prioCollectorThread::getName(const char* p) {
    string str(name);
    str.append(p);
    return str;
}

void prioCollectorThread::reinitialise(int width, int height){
    this->width  = width;
    this->height = height;
    
    if(0 != map1_yarp) {
        map1_yarp->resize(width,height);
        map1_yarp->zero();
    }
    
    if(0 != map2_yarp) {
        map2_yarp->resize(width,height);
        map2_yarp->zero();    
    }
}

bool prioCollectorThread::earlyFilter(ImageOf<PixelMono>* map1_yarp, ImageOf<PixelMono>* map2_yarp, ImageOf<PixelMono> linearCombinationImage, double& xm, double&ym) {
    
    unsigned char* pmap1Left   = map1_yarp->getRawImage();
    unsigned char* pmap1Right  = map1_yarp->getRawImage(); 
    unsigned char* pmap2Left   = map2_yarp->getRawImage();  
    unsigned char* pmap2Right  = map2_yarp->getRawImage();
    unsigned char* plinearLeft = linearCombinationImage.getRawImage();
    unsigned char* plinearRight= linearCombinationImage.getRawImage();
    int rowSize    = map1_yarp->getRowSize();
    int halfwidth  = width  >> 1;
    
    ImageOf<PixelMono>& tmpImage = testPort.prepare();
    tmpImage.resize(width,height);
    tmpImage.zero();
    
    unsigned char* ptmp        = tmpImage.getRawImage();
    ptmp         += halfwidth - 1;            
    pmap1Left    += halfwidth - 1;                   
    pmap1Right   += halfwidth;                  
    pmap2Left    += halfwidth - 1;                  
    pmap2Right   += halfwidth;
    plinearLeft  += halfwidth - 1;
    plinearRight += halfwidth;
            
    // exploring the image from rho=0 and from theta = 0
    double value;
    double threshold = 255;
    double thresholdInt = 225;
    
    bool ret = false;
    //printf("before the loop %d %d %d  \n", height, width, rowSize); 
    for(int y = 0 ; y < height ; y++){
        for(int x = 0 ; x < halfwidth ; x++){
            
            //------------- motion ----------------------
            
            if(true) {
                
                //value = (k2/sumK) * (double) *pmap2Left;
                value = *pmap2Left;
                if(*pmap2Left >= threshold){
                    printf("max in motion Left %d %f \n",*pmap2Left, value); 
                }
                
                *ptmp = *pmap2Left;
                *plinearLeft = value;
                
                if (value >= threshold) {
                    printf("max in motion Left    \n", (unsigned char) *pmap2Left); 
                    
                    *plinearLeft = 255;
                    crossAssign(plinearLeft, 255, rowSize);
                    xm = halfwidth - x;
                    ym = y;
                    timing = 0.1;
                    //printf("Jumping to cartSpace \n");
                    //goto cartSpace;
                    ret = true;
                    return ret;
                    //y = height;// for jumping out of the outer loop
                    //idle = true;                        
                    //break;
                }
                
                pmap2Left--;
                ptmp--;
                
                //value = (k2/sumK) *  (double) *pmap2Right ;                   
                value = *pmap2Right;
                if(*pmap2Right >= threshold){
                    printf("max in motion Right %d %f \n",*pmap2Right, value); 
                }
                *plinearRight = value;

                
                if (value >= threshold) {
                    printf("max in motion Right \n");                    
                    
                    *plinearRight = 255;
                    crossAssign(plinearRight, 255, rowSize);
                    xm = halfwidth + x;
                    ym = y;
                    timing = 0.1;
                    //printf("Jumping to cartSpace \n");
                    //goto cartSpace;
                    ret = true;
                    return ret;
                    //y = height;// for jumping out of the outer loop
                    //idle = true;                        
                    //break;
                }
                
                pmap2Right++;                           
            }
            
            
            // ----------- intensity ---------------------
            //value = (k1/sumK) * (double) *pmap1Right;
            value = 0.80 * (double) *pmap1Right;
            //*plinearRight = value;
            if (value >= thresholdInt){
                printf("max in intesity Right %f \n",value);                    
                //*plinearRight = 255;
                //crossAssign(plinearRight, 255, rowSize);
                xm = halfwidth + x;
                ym = y;
                timing = 0.1;
                //printf("Jumping to cartSpace \n");
                //goto cartSpace;
                ret = false;
                return ret;
                //y = height;// for jumping out of the outer loop
                //idle =  true;                        
                //break;
            }
            pmap1Right++;
            
            
            //value = (k1/sumK) * (double) *pmap1Left;
            value = 0.85 * (double)*pmap1Left;
            //*plinearLeft = value;
            if (value >= thresholdInt){
                printf("max in intensity Left %f \n",value);
                //*plinearRight = 255;
                //crossAssign(plinearLeft, 255, rowSize);                   
                xm = halfwidth - x;
                ym = y;
                timing = 0.1;
                //printf("Jumping to cartSpace \n");
                //goto cartSpace;
                ret = true;
                return ret;
                //y = height;// for jumping out of the outer loop
                //idle =  true;                        
                //break;
            }
            pmap1Left--;
            
       
            // moving pointer of the plinear
            // in this version only the motion map is saved in the plinear
            plinearRight++;
            plinearLeft--;
            
            
        }
        pmap1Right   += rowSize - halfwidth;
        pmap1Left    += rowSize + halfwidth;
        pmap2Right   += rowSize - halfwidth;
        pmap2Left    += rowSize + halfwidth;        
        ptmp         += rowSize + halfwidth;
        plinearRight += rowSize - halfwidth;
        plinearLeft  += rowSize + halfwidth;
    }

    //tmpImage = *(map2_yarp);
    testPort.write();

    return ret;
}

void prioCollectorThread::run() {
    printf("prioCollectorThread::run \n");
    //ImageOf<PixelMono>* tmp = new ImageOf<PixelMono>;
    //tmp->resize(320,240);

    while(isStopping() != true){
                

        if(0 == map1_yarp) {
            continue;            
        }
        if(0 == map2_yarp) {
            continue;
        }
        if(0 == linearCombinationImage) {
            continue;
        }

      

        //if((tmp == 0)&&(!reinit_flag)){
        //    continue;
        //}                

        /*
        if(!reinit_flag){
            printf("reinit \n");
            reinitialise(tmp->width(), tmp->height());
            reinit_flag = true;
        }
        */

        /*
        
        idle = false;
        //reading contrast map 
        printf("copying the contrast map \n");
        if((map1Port->getInputCount())&&(k1!=0)) {
            if(tmp != 0) {                
                copy_8u_C1R(tmp,map1_yarp);
                //idle=false;
            }
        }

        //reading motion map       
        if((map2Port->getInputCount())&&(k2!=0)) { 
            tmp = map2Port->read(false);
            printf("Copying motion map \n");
            if(tmp != 0) {
                copy_8u_C1R(tmp,map2_yarp);
                //idle=false;
            }
        }
        
        double xm, ym;
        

        //ImageOf<PixelMono>& linearCombinationImage = linearCombinationPort.prepare();
        //linearCombinationImage.resize(width,height);
        
        if(false){
            
        }
        
        */
        
        double xm,ym;
        bool res = earlyFilter(map1_yarp, map2_yarp, *linearCombinationImage, xm, ym);
        if(res) {
            //printf("max in contrast or motion \n");
            Bottle b;
            b.addString("MOT");
            b.addInt(xm);
            b.addInt(ym);
            setChanged();
            notifyObservers(&b);
        }

        Time::delay(0.033);

    }
}

void prioCollectorThread::onStop() {
    inCommandPort.interrupt();
    testPort.interrupt();
    inCommandPort.close();
    testPort.close();
}

void prioCollectorThread::threadRelease() {

}
