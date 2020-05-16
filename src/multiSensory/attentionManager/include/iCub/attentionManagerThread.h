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

#include <iCub/attention/commandDictionary.h>

using namespace yarp::os;
using namespace std;


typedef yarp::sig::ImageOf<yarp::sig::PixelMono>  yImgPixelMono;

class attentionManagerThread : public PeriodicThread {

public:
    attentionManagerThread(string moduleName = "attentionManager");
    ~attentionManagerThread();
    bool configure(yarp::os::ResourceFinder &rf);
    bool threadInit();
    void run();
    void threadRelease();
    string getName(const char* p) const;
    void resetAttentionState();
    void suspendAttentionState();
    void setThreshold(int val);
    int getThreshold();

private:
    Network yarp;

    //Names
    string moduleName;
    string combinedImagePortName;
    string hotPointPortName;
    string engineControlPortName;



    //Input Ports
    BufferedPort<yImgPixelMono> combinedImagePort;

    //output Ports
    BufferedPort<Bottle> hotPointPort;

    //rpcPorts
    RpcClient engineControlPort;


    //Data
    yImgPixelMono *combinedImage;
    cv::Mat combinedImageMat;


    //Parameters
    int thresholdVal;


    //Processing Variables
    double maxValue;
    double minValue;
    cv::Point idxOfMax;
    cv::Point idxOfMin;



    //processing functions
    bool sendMaxPointToLinker(cv::Point maxPoint, int val);
    bool suspendEngine();
    bool resumeEngine();


};
#endif //ATTENTION_ATTENTIONMANAGERTHREAD_H
