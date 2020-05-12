// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* 
 * Copyright (C) 2012 RobotCub Consortium, European Commission FP6 Project IST-004370
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

#ifndef _PER_TRACKER_THREAD_H_
#define _PER_TRACKER_THREAD_H_

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

/**
 * @brief : tracker that updates its tracked image only when required from the cognitive agent with a top-down command
 */

class periodicTrackerThread : public yarp::os::Thread {
protected:
    ResourceFinder &rf;

    string name;
    int template_size;
    int search_size;
    float lastMinCumul;                           // last min cumulative value extracted by template matching                             
 
    CvRect  search_roi;                           // region of interest of the search
    CvRect  template_roi;                         // region of interest of the template
    CvPoint point;                                // selected point

    bool firstConsistencyCheck;   
    bool running;                                 // flag tha indicates when the main cycle has to be performed
    bool init_success;                            // flag that check whether initialisation was successful
    bool check;                                   // flag that allows external user to enable updating in the class
    bool update;                                  // flag that allows for updating the template image
    bool computation_done;                        // flag that indicates when the computation has been performed

    ImageOf<PixelMono> imgMonoIn;                 // input image for the comparison
    ImageOf<PixelMono> imgMonoPrev;               // updated image result of the previous steps
    
    BufferedPort<ImageOf<PixelBgr> >  inPort;     // current image 
    BufferedPort<ImageOf<PixelBgr> >  outPort;    // output image extracted from the current
    BufferedPort<ImageOf<PixelMono> > tmplPort;   // image where the template is extracted
    BufferedPort<ImageOf<PixelMono> > masterPort; // template

    yarp::os::Semaphore mutex;                    // semaphore for the min cumulative value
    yarp::os::Semaphore mutexCheck;               // semaphore for the mutexCheck
    yarp::os::Semaphore mutexUpdate;              // semaphore for the mutexUpdate
    yarp::os::Semaphore mutexComput;              // semaphore for the computation done

public:
    /************************************************************************/
    periodicTrackerThread(ResourceFinder &_rf) : rf(_rf) { }

    /************************************************************************/
    /**
     * @brief initialisation of the class
     */
    virtual bool threadInit()
    {
        //name = "matchTracker"; //rf.check("name",Value("matchTracker")).asString().c_str();
        template_size = 50; //rf.check("template_size",Value(20)).asInt();
        search_size = 220;  //rf.check("search_size",Value(100)).asInt();

        //inPort.open(("/"+name+"/img:i").c_str());
        //outPort.open(("/"+name+"/img:o").c_str());
        //tmplPort.open(("/"+name+"/tmpl:o").c_str());

        inPort.open    ((name+"/img:i")   .c_str());
        outPort.open   ((name+"/img:o")   .c_str());
        tmplPort.open  ((name+"/tmpl:o")  .c_str());
        masterPort.open((name+"/master:o").c_str());

        // setting flags
        firstConsistencyCheck = true;
        running               = false;
        init_success          = false;
        computation_done      = false;
        
        point.x = 0;
        point.y = 0;

        return true;
    }

    /************************************************************************/
    /**
     * @brief function that sets the name of the object
     */
    void setName(string str) {
        name=str;
        printf("name: %s \n", name.c_str());
    }

    /************************************************************************/
    /**
     * @brief function that set the check flag. Set from the higher cognitive level allows for comparison of images
     */
    void setCheck(bool value) {
        mutexCheck.wait();
        check = value;
        mutexCheck.post();
    }

    /************************************************************************/
    /**
     * @brief function that set the flage that forces for an update of the template image right before tracker init.
     */
    void setUpdate(bool value) {
        mutexUpdate.wait();
        update = value;
        mutexUpdate.post();
    }

    /************************************************************************/
    
    /**
     * @brief function that returns the name of the object
     */
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
    
    /**
     * @brief function that returns the correlation measure of the last comparison
     */
    int getLastMinCumul() {
        float tmp;
        mutex.wait();
        tmp = lastMinCumul;
        mutex.post();
        return tmp;
    }

    /************************************************************************/
    /**
     * @brief main function where the active algorithm is defined
     */
    virtual void run()
    {
        int count = 0;
        while (!isStopping())
        {
            // acquire new image
            ImageOf<PixelBgr> *pImgBgrIn=inPort.read(true);

            if (isStopping() || (pImgBgrIn==NULL)) {
                //printf("breaking off because the input image is null \n");
                break;
            }
                    

            // consistency check
            if (firstConsistencyCheck)
            {
                imgMonoIn.resize(*pImgBgrIn);
                firstConsistencyCheck=false;
                // save data for the first cycle
                imgMonoPrev = imgMonoIn;            
            }


            // convert the input-image to gray-scale
            cvCvtColor(pImgBgrIn->getIplImage(),imgMonoIn.getIplImage(),CV_BGR2GRAY);

            // copy input-image into output-image
            ImageOf<PixelBgr>  &imgBgrOut   = outPort.prepare();
            ImageOf<PixelMono> &imgTemplate = tmplPort.prepare();
            ImageOf<PixelMono> &master      = masterPort.prepare();
            master.resize(template_roi.width, template_roi.height);
            master.zero();
            int padding = master.getPadding();
            imgBgrOut   = *pImgBgrIn;
            imgTemplate = imgMonoPrev;
            
            bool check_copy;
            mutexCheck.wait();
            check_copy = check;
            mutexCheck.post();
            
            // ****************************************************************************************
            if (running) {
                
                // the episodic tracker thread performs a comparison only when the check flag is set
                // the check flag is set to false immidiately
                
                
                ImageOf<PixelMono> &img = imgMonoIn;      // image where to seek for the template in
                ImageOf<PixelMono> &tmp = imgMonoPrev;    // image containing the template
                
                // specify the searching area
                search_roi.x = (std::max)(0, (std::min) ((int) (img.width()  - search_roi.width),  point.x - (search_roi.width  >> 1)));
                search_roi.y = (std::max)(0, (std::min) ((int) (img.height() - search_roi.height), point.y - (search_roi.height >> 1)));
                
                // specify the template area
                template_roi.x = (std::max) (0, (std::min) ((int) (tmp.width()  - template_roi.width),  point.x - (template_roi.width  >> 1)));
                template_roi.y = (std::max) (0, (std::min) ((int) (tmp.height() - template_roi.height), point.y - (template_roi.height >> 1)));
                
                // updating the template image right after the selected position
                bool update_tmp;
                mutexUpdate.wait();
                update_tmp = update;
                mutexUpdate.post();                
                if (update_tmp) {
                     imgMonoPrev = imgMonoIn; 
                     mutexUpdate.wait();
                     update = false;
                     mutexUpdate.post(); 
                }

                // representing the template in the master image
                for (int y1 = 0; y1 < template_roi.height; y1++) {
                    for (int x1 = 0; x1 < template_roi.width; x1++) {
                        unsigned int value  = tmp( template_roi.x+ x1, template_roi.y+ y1);
                        master(x1,y1) = value;
                    }
                }                       
                
                // perform tracking with template matching
                float ftmp;
                CvPoint minLoc;
                if (check_copy) {
                    
                    minLoc=sqDiff(img,search_roi,tmp,template_roi, ftmp);

                    // update point coordinates (ONLY IF REQUIRED BY SUPERUSER)
                    point.x = search_roi.x + minLoc.x + (template_roi.width  >> 1);
                    point.y = search_roi.y + minLoc.y + (template_roi.height >> 1);
                    
                    mutexComput.wait();
                    computation_done = true;
                    mutexComput.post();
                    printf("minima %f in point(%d,%d) \n", ftmp, point.x, point.y);
               
                    // updating the correlation measure 
                    mutex.wait();
                    lastMinCumul = ftmp;
                    mutex.post();

                    
                }
                
                
                    
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
                
                
                init_success = true; // considering init success at the end of the first loop
                count++;
            }// end if running
            
            // *****************************************************
            // send out output-image
            outPort   .write();
            tmplPort  .write();
            if (init_success) {
                masterPort.write();
            }
            if (check_copy) {
                // save data for next cycle
                imgMonoPrev = imgMonoIn;            
                mutexCheck.wait();
                check = false;
                mutexCheck.post();
                
            }
            
        } //end of the while
    }

    /************************************************************************/
    virtual void onStop()
    {
        inPort    .interrupt();
        outPort   .interrupt();
        tmplPort  .interrupt();
        masterPort.interrupt();
    }

    /************************************************************************/
    virtual void threadRelease()
    {
        inPort    .close();
        outPort   .close();
        tmplPort  .close();
        masterPort.close();
    }

    /************************************************************************/
    string getName()
    {
        return name;
    }

    /************************************************************************/

    /**
     * @brief function that initialises correctly the tracker on the pattern surrounding the position x y
     */
    void init(const int x, const int y){
        printf("initialisation of the episodic tracker in %d %d \n", x, y);
        init_success = false;
        point.x = x;
        point.y = y;

        search_roi.width   = search_roi.height   = search_size;
        template_roi.width = template_roi.height = template_size;

        running = true;
        
    }
    /*****************************************************************************/
    /**
     * @brief waiting for the success in the initialisation of the tracker
     */
    void waitInitTracker() {
        while (!init_success) {
            Time::delay(0.005);
        }
    }

    /*****************************************************************************/
    /**
     * @brief waiting for the success in the initialisation of the tracker
     */
    void waitCorrComputation() {
        mutexComput.wait();
        bool comp_done = computation_done;
        mutexComput.post();
        while (!comp_done) {
            Time::delay(0.005);
            mutexComput.wait();
            comp_done = computation_done;
            mutexComput.post();
        }
        mutexComput.wait();
        computation_done = false;
        mutexComput.post();
    }

    

    /*****************************************************************************/
    /**
     * @brief returs the point where the max correlation measure is localised
     */
    void getPoint(CvPoint& p) {
        p = point;
    }

    /************************************************************************/
    /**
     * @brief function that calculates the pixel-wise correlation measure
     */
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
    /**
     * @brief function that calculates the pixel-wise correlation measure
     */
    CvPoint sqDiff(const ImageOf<PixelMono> &img, const CvRect searchRoi,
                   const ImageOf<PixelMono> &tmp, const CvRect tmpRoi, float& minCumul)
    {
        bool firstCheck=true;
        minCumul = 0.0;
        CvPoint minLoc;

        for (int y = 0; y < searchRoi.height - tmpRoi.height + 1; y++)
        {
            for (int x = 0; x < searchRoi.width - tmpRoi.width + 1; x++)
            {
                float curCumul=0.0;
            
                for (int y1 = 0; y1 < tmpRoi.height - 1; y1++)
                {
                    for (int x1 = 0; x1 < tmpRoi.width - 1; x1++)
                    {
                        int diff  = tmp( tmpRoi.x        + x1, tmpRoi.y        + y1) 
                                  - img( searchRoi.x + x + x1, searchRoi.y + y + y1);
                        curCumul += diff * diff;
                    }
                }
               
                if ((curCumul<minCumul) || firstCheck)
                {
                    minLoc.x   = x;
                    minLoc.y   = y;
            
                    minCumul   = curCumul;
                    firstCheck = false;
                }
            }
        }

        return minLoc;
    }

    /************************************************************************/
    /**
     * @brief command interface with the rest of the system
     */
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

#endif  //_PER_TRACKER_THREAD_H_

//----- end-of-file --- ( next line intentionally left blank ) ------------------

