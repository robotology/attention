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
    bool resetAttentionState(int statType);
    bool suspendAttentionState(int statType);
    void setThreshold(const int32_t mode,const int32_t type,float val);
    bool resetThreshold(const int32_t type);
    float getThreshold(const int32_t type);

private:
    Network yarp;

    //Names
    string moduleName;
    string combinedImagePortName;
    string hotPointPortName;
    string engineControlPortName;
    string gazeArbiterControlPortName;
    string sceneAnalysisPortName;



    //Input Ports
    BufferedPort<yImgPixelRGB> combinedImagePort;

    //output Ports
    BufferedPort<Bottle> hotPointPort;
    BufferedPort<Bottle> sceneAnalysisPort;

    //rpcPorts
    RpcClient engineControlPort;
    RpcClient gazeArbiterControlPort;


    //Data
    yImgPixelRGB *combinedImage;



    //Parameters
    int max_thresholdVal;
    float mean_thresholdVal;
    float std_thresholdVal;
    float threeSigma_thresholdVal;

    int init_max_thresholdVal;
    float init_mean_thresholdVal;
    float init_std_thresholdVal;
    float init_threeSigma_thresholdVal;


    //Processing Variables
    unsigned char maxValue;
    float meanVal;
    float stdVal;
    float threeSigmaVal;
    cv::Point idxOfMax;



    //processing functions
    bool sendMaxPointToLinker(cv::Point maxPoint, int val,float imgMean,float imgStd);
    bool suspendEngine();
    bool resumeEngine();
    bool suspendArbiter();
    bool resumeArbiter();
    bool publishAnalysis();

};
#endif //ATTENTION_ATTENTIONMANAGERTHREAD_H
