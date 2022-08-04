#include "darwin/colorsegModule.h"

using namespace std;

using namespace yarp::os;
using namespace yarp::sig;

// change namespace name according to your module name:
// ex: using namespace Darwin::pmp (used for PMPmodule)
using namespace Darwin::colorseg;


const string colorsegModule::CMD_LIST[] = {"help", "start", "stop", "sus", "res", "exit"};

const string colorsegModule::CMD_DESC[] = {"Gives the available command list and description",
                                           "Start colorsegThread",
                                           "Stop colorsegThread",
                                           "Suspend colorsegThread",
                                           "Resume colorsegThread",
                                           "Quit module"};
const unsigned int colorsegModule::CMD_SIZE = 6;


bool colorsegModule::identifyCmd(Bottle cmdBot, colorsegCommands &cmd) {
    unsigned int i = 0;
    string word = cmdBot.get(0).asString().c_str();

    for (unsigned int i = 0; i < CMD_SIZE; i++) {
        if (word == CMD_LIST[i]) {
            cmd = (colorsegCommands) i;
            return true;
        }
    }

    return false;
}

bool colorsegModule::configure(ResourceFinder &rf) {
    threadPeriod = 0;

    // set module name: if no name exists, set PMPmodule as the module name
    moduleName = rf.check("name",
                          Value("colorSegmentation"),
                          "module name (string)").asString();

    // set thread name: if no name exists, set PMPthread as the module name
    // set thread period: if no period specified, set 20 ms;
    threadName = rf.check("threadName",
                          Value("colorSegmentation"),
                          "module name (string)").asString();
    threadPeriod = rf.check("threadPeriod", Value(20), "Thread period in ms").asInt16();


    if (moduleName.empty() || threadName.empty() || threadPeriod == 0)
        return false;

    setName(moduleName.c_str());
    printf("--> Module name set as: %s\n", moduleName.c_str());
    printf("--> Thread name set as: %s\n", threadName.c_str());
    printf("--> Thread period set as: %d ms\n", threadPeriod);

    // instantiate and start the thread
    tThread = new colorsegThread(rf, threadName, threadPeriod);
    if (!tThread->start())
        return false;

    // open rpc Port called /moduleName/rpc. moduleName is read from a config file
    // and connect it to the respond method
    rpcPort.open(("/" + moduleName + "/rpc").c_str());
    if (!Network::exists(rpcPort.getName()))
        return false;
    printf("--> Rpc port name set as: %s\n", rpcPort.getName().c_str());
    attach(rpcPort);

    return true;
}

double colorsegModule::getPeriod() {
    return 0.1;
}

bool colorsegModule::updateModule() {
    return true;
}

bool colorsegModule::respond(const Bottle &command, Bottle &reply) {
    reply.clear();
    colorsegCommands cmd;

    //identifyCmd reads only the first keyword
    if (!identifyCmd(command, cmd)) {
        reply.addString("Unknown command. Input 'help' to get a list of the available commands.");
        return true;
    }

    switch (cmd) {
        case help:
            for (int i = 0; i < CMD_SIZE; i++) {
                reply.addString(CMD_LIST[i].c_str());
                reply.addString(CMD_DESC[i].c_str());
            }
            break;

        case start:
            if (!tThread->isRunning()) {
                tThread->start();
                reply.addString("--> Thread started");
            } else if (tThread->isSuspended()) {
                tThread->resume();
                reply.addString("--> Thread resumed");
            } else
                reply.addString("--> Thread was already running");
            break;

        case stop:
            tThread->stop();
            this->stopModule();
            reply.addString("--> Thread stopped");
            break;

        case sus:
            tThread->suspend();
            reply.addString("--> Thread suspended");
            break;

        case res:
            tThread->resume();
            reply.addString("--> Thread resumed");
            break;

        case exit:
            reply.addString("--> Quitting");
            interruptModule();
            break;

        default:
            return false;
    }

    return true;
}

bool colorsegModule::interruptModule() {
    tThread->stop();
    //rpcPort.interrupt();
    return true;
}

bool colorsegModule::close() {
    //tThread->threadRelease();
    //delete tThread;
    rpcPort.interrupt();

    rpcPort.close();
    return true;
}
