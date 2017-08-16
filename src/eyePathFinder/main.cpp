/* 
 * Copyright (C) 2010 RobotCub Consortium, European Commission FP6 Project IST-004370
 * Authors: Francesco Rea
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
* @ingroup icub_contrib_modules
* \defgroup eyePathFinder eyePathFinder

* Tracks the fixation point in the image plane
* that resorts to the sum of squared differences. 
 
* Basically the module opens up three ports: 
 
- /matchTracker/img:i, where to send the incoming images 
- /matchTracker/img:o, streaming out the output images 
- /matchTracker/rpc , for remote commands 
 

\author Rea Francesco
*/ 

/**
* 16/01/11 : added the cross in the middle of the remapping of fovea in cartesian image                         @author Rea
* 16/01/11 : representing the trajectory of the projection of fixation point  in the image                      @author Rea

*/


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

#define crossSize 12
#define collectionDim 100


/************************************************************************/
class ProcessThread : public Thread
{
protected:
    ResourceFinder &rf;

    string name;
    int template_size;
    int search_size;
    int positionCounter;

    CvRect  search_roi;
    CvRect  template_roi;
    CvPoint point;
    CvPoint positions[collectionDim];

    bool firstConsistencyCheck;
    bool running;

    ImageOf<PixelMono> imgMonoIn;
    ImageOf<PixelMono> imgMonoPrev;

    BufferedPort<ImageOf<PixelBgr> > inPort;
    BufferedPort<ImageOf<PixelBgr> > outPort;

public:
    /************************************************************************/
    ProcessThread(ResourceFinder &_rf) : rf(_rf) { }

    /************************************************************************/
    virtual bool threadInit()
    {
        name=rf.check("name",Value("eyePathFinder")).asString().c_str();
        template_size=rf.check("template_size",Value(20)).asInt();
        search_size=rf.check("search_size",Value(100)).asInt();

        inPort.open(("/"+name+"/img:i").c_str());
        outPort.open(("/"+name+"/img:o").c_str());

        firstConsistencyCheck=true;
        running=false;

        positionCounter = 0;
        for (int i = 0 ; i< collectionDim ; i ++) {
            positions[i].x = -1;
            positions[i].y = -1;
        }

        return true;
    }

    /************************************************************************/
    virtual void run() {
        while (!isStopping()) {
            // acquire new image
            ImageOf<PixelBgr> *pImgBgrIn=inPort.read(true);

            if (isStopping() || (pImgBgrIn==NULL))
                break;
            //initialising the process
            int xFix = (int) imgMonoIn.width() / 2;
            int yFix = (int) imgMonoIn.height() / 2;
            init( xFix, yFix);
            // consistency check
            if (firstConsistencyCheck) {
                imgMonoIn.resize(*pImgBgrIn);
                firstConsistencyCheck=false;
                running = false;
            }

            // convert the input-image to gray-scale
            cvCvtColor(pImgBgrIn->getIplImage(),imgMonoIn.getIplImage(),CV_BGR2GRAY);
            
            
            // copy input-image into output-image
            ImageOf<PixelBgr> &imgBgrOut=outPort.prepare();
            imgBgrOut=*pImgBgrIn;
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
                CvPoint minLoc=sqDiff(img,search_roi,tmp,template_roi);

                // update point coordinates
                point.x=search_roi.x + minLoc.x+(template_roi.width>>1);
                point.y=search_roi.y + minLoc.y+(template_roi.height>>1);

                // draw results on the output-image
                CvPoint p0, p1;
                p0.x = point.x - (template_roi.width>>1);
                p0.y = point.y - (template_roi.height>>1);
                p1.x = p0.x + template_roi.width;
                p1.y = p0.y + template_roi.height;
                cvRectangle(imgBgrOut.getIplImage(),p0,p1,cvScalar(0,0,255),1);

                cvRectangle(imgBgrOut.getIplImage(),cvPoint(search_roi.x,search_roi.y),
                            cvPoint(search_roi.x+search_roi.width,search_roi.y+search_roi.height),
                            cvScalar(255,0,0),2);

                unsigned char* pout = (unsigned char *) imgBgrOut.getRawImage();
                int rowsize = imgBgrOut.getRowSize();

                
                //updating the position vector & representing the trajectory
                int errorx = xFix - point.x;
                int errory = yFix - point.y;
                
                positions[positionCounter].x = point.x;
                positions[positionCounter].y = point.y;
                
                
                for ( int i = 0; i < collectionDim ; i++) {
                    if( (positions[i].x != -1) && (positions[i].y != -1) && (i != positionCounter)) {
                        positions[i].x = positions[i].x - errorx;
                        positions[i].y = positions[i].y - errory;
                        pout = (unsigned char *) imgBgrOut.getRawImage();
                        if( (positions[i].x >= 0) &&(positions[i].y >= 0)) {
                            pout += rowsize * positions[i].y + positions[i].x * 3 + 2; // trajectory composed of blue dots (just red)
                            *pout = 255;        
                        }
                    }
                }
                positionCounter ++;
                if (positionCounter == collectionDim) {
                    positionCounter = 0;
                } 
                
                
                //representing the cross in the fovea
                int padding = imgBgrOut.getPadding();
                
                pout = (unsigned char *) imgBgrOut.getRawImage();
                pout += (yFix - crossSize / 2) * rowsize + xFix * 3;
                for (int r = 0; r < crossSize + 1  ; r++) {
                    *pout ++ = 255;
                    *pout ++ = 255;
                    *pout ++ = 255;
                    pout += rowsize - 3;
                }
                pout = (unsigned char *) imgBgrOut.getRawImage();
                pout += yFix * rowsize + (xFix - crossSize / 2) * 3;
                for (int c = 0; c < crossSize + 1; c++) {
                        *pout++ = 255;
                        *pout++ = 255;
                        *pout++ = 255;
                }
                
            }

            // send out output-image
            outPort.write();

            // save data for next cycle
            imgMonoPrev=imgMonoIn;            
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
    void init(const int x, const int y)
    {
        point.x = x;
        point.y = y;

        search_roi.width=search_roi.height=search_size;
        template_roi.width=template_roi.height=template_size;

        running=true;
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
                        curCumul += diff * diff;
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


/************************************************************************/
class ProcessModule: public RFModule
{
private:
    ProcessThread *thr;
    Port           rpcPort;

public:
    /************************************************************************/
    ProcessModule() : thr(NULL) { }

    /************************************************************************/
    virtual bool configure(ResourceFinder &rf)
    {
        Time::turboBoost();

        thr=new ProcessThread(rf);
        if (!thr->start())
        {
            delete thr;    
            return false;
        }

        rpcPort.open(("/"+thr->getName()+"/rpc").c_str());
        attach(rpcPort);

        return true;
    }

    /************************************************************************/
    virtual bool respond(const Bottle &command, Bottle &reply)
    {
        if (thr->execReq(command,reply))
            return true;
        else
            return RFModule::respond(command,reply);
    }

    /************************************************************************/
    virtual bool close()
    {
        if (thr)
        {
            thr->stop();
            delete thr;
        }

        rpcPort.interrupt();
        rpcPort.close();

        return true;
    }

    /************************************************************************/
    virtual double getPeriod()
    {
        return 1.0;
    }

    /************************************************************************/
    virtual bool updateModule()
    {
        return true;
    }
};


/************************************************************************/
int main(int argc, char *argv[])
{
    ResourceFinder rf;
    rf.setVerbose(true);
    rf.configure(argc,argv);

    Network yarp;
    if (!yarp.checkNetwork())
        return -1;

    ProcessModule mod;

    return mod.runModule(rf);
}


