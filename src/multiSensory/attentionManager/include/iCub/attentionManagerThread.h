//
// Created by omar on 14/04/20.
//

#ifndef ATTENTION_ATTENTIONMANAGERTHREAD_H
#define ATTENTION_ATTENTIONMANAGERTHREAD_H

#include <stdio.h>
#include <cmath>
#include <string>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/Time.h>
#include <yarp/sig/all.h>
#include <yarp/os/Thread.h>
#include <yarp/os/Vocab.h>
#include <yarp/os/Network.h>
#include <yarp/os/all.h>
#include <yarp/cv/Cv.h>
#include <vector>
#include <numeric>
#include <iCub/attention/commandDictionary.h>

using namespace yarp::os;
using namespace std;


typedef yarp::sig::ImageOf<yarp::sig::PixelRgb>  yImgPixelRGB;

class attentionManagerThread : public PeriodicThread {

public:
    attentionManagerThread(string moduleName = "attentionManager");
    ~attentionManagerThread();
    bool configure(yarp::os::ResourceFinder &rf);
    bool threadInit();
    void run();
    void threadRelease();
    string getName(const char* p) const;
    bool resetAttentionState();
    bool suspendAttentionState();
    void setThreshold(int val);
    int getThreshold();

private:
    Network yarp;

    //Names
    string moduleName;
    string combinedImagePortName;
    string hotPointPortName;
    string engineControlPortName;
    string gazeArbiterControlPortName;



    //Input Ports
    BufferedPort<yImgPixelRGB> combinedImagePort;

    //output Ports
    BufferedPort<Bottle> hotPointPort;

    //rpcPorts
    RpcClient engineControlPort;
    RpcClient gazeArbiterControlPort;


    //Data
    yImgPixelRGB *combinedImage;



    //Parameters
    int thresholdVal;


    //Processing Variables
    unsigned char maxValue;
    cv::Point idxOfMax;



    //processing functions
    bool sendMaxPointToLinker(cv::Point maxPoint, int val);
    bool suspendEngine();
    bool resumeEngine();
    bool suspendArbiter();
    bool resumeArbiter();


};
#endif //ATTENTION_ATTENTIONMANAGERTHREAD_H
