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
using namespace yarp::os;
using namespace std;


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
    RpcClient port;
    string moduleName;

};
#endif //ATTENTION_ATTENTIONMANAGERTHREAD_H
