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
 * Public License for more details
 */

/**
 * @file trackerThread.h
 * @brief RateThread which collects gaze request from the lower level as commands and foward those to the arbiter
 * 
 */

#ifndef _TRACKER_THREAD_H_
#define _TRACKER_THREAD_H_

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Image.h>

#include <cv.h>
#include <highgui.h>

#include <stdio.h>
#include <string>

using namespace std;
using namespace yarp;
using namespace yarp::os;
using namespace yarp::sig;

class trackerThread : public yarp::os::Thread {
protected:
    ResourceFinder &rf;

    string name;
    int template_size;
    int search_size;
    float lastMinCumul;                         // last min cumulative value extracted by template matching                             
 
    CvRect  search_roi;
    CvRect  template_roi;
    CvPoint point;

    bool firstConsistencyCheck;
    bool running;
    bool init_success;                           //flag that check whether initialisation was successful

    ImageOf<PixelMono> imgMonoIn;
    ImageOf<PixelMono> imgMonoPrev;
    
    BufferedPort<ImageOf<PixelBgr> > inPort;     // current image 
    BufferedPort<ImageOf<PixelBgr> > outPort;    // output image extracted from the current
    BufferedPort<ImageOf<PixelMono> > tmplPort;  // template image where the template is extracted

    yarp::os::Semaphore mutex;                   // semaphore for the min cumulative value

public:
    /************************************************************************/
    trackerThread(ResourceFinder &_rf) : rf(_rf) { }

    /************************************************************************/
    virtual bool threadInit()
    {
        //name = "matchTracker"; //rf.check("name",Value("matchTracker")).asString().c_str();
        template_size = 20; //rf.check("template_size",Value(20)).asInt();
        search_size   = 40;  //rf.check("search_size",Value(100)).asInt();

        //inPort.open(("/"+name+"/img:i").c_str());
        //outPort.open(("/"+name+"/img:o").c_str());
        //tmplPort.open(("/"+name+"/tmpl:o").c_str());

        inPort.open  ((name+"/img:i").c_str());
        outPort.open ((name+"/img:o").c_str());
        tmplPort.open((name+"/tmpl:o").c_str());

        firstConsistencyCheck = true;
        running = false;
        init_success = false;
        point.x = 0;
        point.y = 0;

        imgMonoPrev.resize(320,240);

        return true;
    }

    /************************************************************************/
    
    void setName(string str) {
        name=str;
        printf("name: %s \n", name.c_str());
    }

    /************************************************************************/
    
    std::string getName(const char* p) {
        string str(name);
        str.append(p);
        return str;
    }
    

    /************************************************************************/

    int getInputCount() {
        return inPort.getInputCount();
    }

    /************************************************************************/

    int getLastMinCumul() {
        float tmp;
        mutex.wait();
        tmp = lastMinCumul;
        mutex.post();
        return tmp;
    }

    /************************************************************************/
    virtual void run()
    {
        int count = 0;
        while (!isStopping())
        {
            // acquire new image
            ImageOf<PixelBgr> *pImgBgrIn=inPort.read(true);

            if (isStopping() || (pImgBgrIn==NULL))
                break;

            // consistency check
            if (firstConsistencyCheck)
            {
                imgMonoIn.resize(*pImgBgrIn);
                firstConsistencyCheck=false;
            }

            // convert the input-image to gray-scale
            cvCvtColor(pImgBgrIn->getIplImage(),imgMonoIn.getIplImage(),CV_BGR2GRAY);

            // copy input-image into output-image
            ImageOf<PixelBgr>  &imgBgrOut   = outPort.prepare();
            ImageOf<PixelMono> &imgTemplate = tmplPort.prepare();
            imgBgrOut   = *pImgBgrIn;
            
            imgTemplate = imgMonoPrev;

            if (running)
            {
                ImageOf<PixelMono> &img = imgMonoIn;      // image where to seek for the template in
                ImageOf<PixelMono> &tmp = imgMonoPrev;    // image containing the template

                // specify the searching area
                search_roi.x=(std::max)(0,(std::min)(img.width()-search_roi.width,point.x-(search_roi.width>>1)));
                search_roi.y=(std::max)(0,(std::min)(img.height()-search_roi.height,point.y-(search_roi.height>>1)));

                // specify the template area
                template_roi.x=(std::max)(0,(std::min)(tmp.width()-template_roi.width,point.x-(template_roi.width>>1)));
                template_roi.y=(std::max)(0,(std::min)(tmp.height()-template_roi.height,point.y-(template_roi.height>>1)));

                // perform tracking with template matching
                float ftmp;
                CvPoint minLoc=sqDiff(img,search_roi,tmp,template_roi, ftmp);
                mutex.wait();
                lastMinCumul = ftmp;
                mutex.post();

                // update point coordinates
                point.x=search_roi.x+minLoc.x+(template_roi.width>>1);
                point.y=search_roi.y+minLoc.y+(template_roi.height>>1);

                if(count % 5 == 0) {
                    // draw results on the output-image
                    CvPoint p0, p1;
                    p0.x=point.x-(template_roi.width>>1);
                    p0.y=point.y-(template_roi.height>>1);
                    p1.x=p0.x+template_roi.width;
                    p1.y=p0.y+template_roi.height;
                    cvRectangle(imgBgrOut.getIplImage(),p0,p1,cvScalar(0,0,255),1);
                    
                    cvRectangle(imgBgrOut.getIplImage(),cvPoint(search_roi.x,search_roi.y),
                                cvPoint(search_roi.x+search_roi.width,search_roi.y+search_roi.height),
                                cvScalar(255,0,0),2);
                    
                    cvRectangle(imgBgrOut.getIplImage(),cvPoint(point.x - 1 ,point.y - 1),
                                cvPoint(point.x + 1, point.y + 1),
                                cvScalar(0,0,255),2);
                    
                    cvRectangle(imgBgrOut.getIplImage(),cvPoint((img.width()>>1)-1,(img.height()>>1)-1),
                                cvPoint((img.width()>>1)+1,(img.height()>>1)+1),
                                cvScalar(0,255,0),2);
                }
                
                init_success = true; // considering init success at the end of the first loop
                count++;
            }

            // send out output-image
            printf("writing output port \n");
            outPort.write();
            tmplPort.write();
            // save data for next cycle
            imgMonoPrev = imgMonoIn;            
        }
    }

    /************************************************************************/
    virtual void onStop()
    {
        inPort.interrupt();
        outPort.interrupt();
    }

    /************************************************************************/
    virtual void threadRelease()
    {
        inPort.close();
        outPort.close();
    }

    /************************************************************************/
    string getName()
    {
        return name;
    }

    /************************************************************************/
    void init(const int x, const int y){
        init_success = false;
        point.x = x;
        point.y = y;

        search_roi.width = search_roi.height=search_size;
        template_roi.width = template_roi.height=template_size;

        running = true;
    }
    /*****************************************************************************/
    
    void waitInitTracker() {
        while (!init_success) {
            Time::delay(0.005);
        }
    }


    /*****************************************************************************/

    void getPoint(CvPoint& p) {
        p = point;
    }

    /************************************************************************/
    CvPoint sqDiff(const ImageOf<PixelMono> &img, const CvRect searchRoi,
                   const ImageOf<PixelMono> &tmp, const CvRect tmpRoi)
    {
        bool firstCheck=true;
        float minCumul=0.0;
        CvPoint minLoc;

        for (int y=0; y<searchRoi.height-tmpRoi.height+1; y++)
        {
            for (int x=0; x<searchRoi.width-tmpRoi.width+1; x++)
            {
                float curCumul=0.0;
            
                for (int y1=0; y1<tmpRoi.height-1; y1++)
                {
                    for (int x1=0; x1<tmpRoi.width-1; x1++)
                    {
                        int diff=tmp(tmpRoi.x+x1,tmpRoi.y+y1)-img(searchRoi.x+x+x1,searchRoi.y+y+y1);
                        curCumul+=diff*diff;
                    }
                }
            
                if ((curCumul<minCumul) || firstCheck)
                {
                    minLoc.x=x;
                    minLoc.y=y;
            
                    minCumul=curCumul;
                    firstCheck=false;
                }
            }
        }

        return minLoc;
    }
    
    /************************************************************************/
    CvPoint sqDiff(const ImageOf<PixelMono> &img, const CvRect searchRoi,
                   const ImageOf<PixelMono> &tmp, const CvRect tmpRoi, float& minCumul)
    {
        bool firstCheck=true;
        minCumul = 0.0;
        CvPoint minLoc;

        for (int y=0; y<searchRoi.height-tmpRoi.height+1; y++)
        {
            for (int x=0; x<searchRoi.width-tmpRoi.width+1; x++)
            {
                float curCumul=0.0;
            
                for (int y1=0; y1<tmpRoi.height-1; y1++)
                {
                    for (int x1=0; x1<tmpRoi.width-1; x1++)
                    {
                        int diff=tmp(tmpRoi.x+x1,tmpRoi.y+y1)-img(searchRoi.x+x+x1,searchRoi.y+y+y1);
                        curCumul+=diff*diff;
                    }
                }
            
                if ((curCumul<minCumul) || firstCheck)
                {
                    minLoc.x=x;
                    minLoc.y=y;
            
                    minCumul=curCumul;
                    firstCheck=false;
                }
            }
        }

        return minLoc;
    }

    /************************************************************************/
    bool execReq(const Bottle &req, Bottle &reply)
    {
        if (req.size())
        {
            string cmd=req.get(0).asString().c_str();

            if (cmd=="start")
            {
                if (req.size()<3)
                    return false;

                init(req.get(1).asInt(),req.get(2).asInt());
                reply.addString("ack");
            }
            else if (cmd=="stop")
            {
                running=false;
                reply.addString("ack");
            }
            else if (cmd=="set")
            {
                if (running || (req.size()<3))
                    return false;

                string subcmd=req.get(1).asString().c_str();

                if (subcmd=="template_size")
                {
                    template_size=req.get(2).asInt();
                    reply.addString("ack");
                }
                else if (subcmd=="search_size")
                {
                    search_size=req.get(2).asInt();
                    reply.addString("ack");
                }
                else
                    return false;
            }
            else if (cmd=="get")
            {
                if (req.size()<2)
                    return false;

                string subcmd=req.get(1).asString().c_str();

                if (subcmd=="template_size")
                    reply.addInt(template_size);
                else if (subcmd=="search_size")
                    reply.addInt(search_size);
                else if (subcmd=="status")
                    reply.addString(running?"running":"paused");
                else if (subcmd=="point")
                {
                    reply.addInt(point.x);
                    reply.addInt(point.y);
                }
                else
                    return false;
            }
            else
                return false;

            return true;
        }
        else
            return false;
    }
};

#endif  //_TRACKER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

