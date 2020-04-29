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


using namespace yarp::os;
using namespace std;


enum class ATTENTION_PROCESS_STATE{PROCESSING,SUSPENDED};
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

private:
    Network yarp;

    //Names
    string moduleName;
    string combinedImagePortName;
    string hotPointPortName;



    //Input Ports
    BufferedPort<yImgPixelMono> combinedImagePort;
    BufferedPort<Bottle> hotPointPort;




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


    ATTENTION_PROCESS_STATE attentionProcessState;



    //processing functions
    bool sendMaxPointToLinker(cv::Point maxPoint);


};
#endif //ATTENTION_ATTENTIONMANAGERTHREAD_H
