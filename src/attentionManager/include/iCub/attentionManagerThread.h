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

//To Bo Transferred to library
#define ACK                         yarp::os::createVocab('a','c','k')
#define NACK                        yarp::os::createVocab('n','a','c','k')
#define CMD_POINT                   yarp::os::createVocab('p','o','i','n')
#define GET_S2C                     yarp::os::createVocab('s','2','c')
#define CMD_GET                     yarp::os::createVocab('g','e','t')






using namespace yarp::os;
using namespace std;


enum class ATTENTION_PROCESS_STATE{PROCESSING,STOPPING_GAZE,STARTING_ACTION,EXECUTING_ACTION,COMPUTING_RESULTS,RESETTING};
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

private:
    Network yarp;

    //Names
    string moduleName;
    string combinedImagePortName;
    string getCartesianCoordinatesPortName;
    string pointActionPortName;


    //Input Ports
    BufferedPort<yImgPixelMono> combinedImagePort;
    RpcClient getCartesianCoordinatesPort;
    RpcClient pointActionPort;


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

    float xCartOfMax;
    float yCartOfMax;
    float zCartOfMax;

    ATTENTION_PROCESS_STATE attentionProcessState;



    //processing functions
    bool expectCartesian3dLocation(int u,int v,float &x,float &y, float &z);
    bool isInsideTheBoard(float x,float y,float z);
    bool refineLocation(float &x,float &y, float &z);
    bool pointToCartesian3dLocation(float x,float y,float z);



};
#endif //ATTENTION_ATTENTIONMANAGERTHREAD_H
