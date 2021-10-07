#include <yarp/os/all.h>
#include <Demo.h>
#include <iostream>

class DemoServer : public Demo
{
  /* Class members declaration */
  int32_t answer;
  bool isRunning;
public:
  DemoServer(){};
  /* function declarations, copied from Demo.h */
  int32_t get_answer();
  bool set_answer(int32_t rightAnswer){};
  int32_t add_one(const int32_t x){};
  bool start(){};
  bool stop(){};
  bool is_running(){};
  bool read(yarp::os::ConnectionReader& connection){};
  std::vector<std::string> help(const std::string& functionName="--all"){};
};

int32_t DemoServer::get_answer()
{
  std::cout << "The answer is "<< answer << std::endl;
  return answer;
}

int main(int argc, char *argv[]) {
    yarp::os::Network yarp;
    DemoServer demoServer;
    yarp::os::Port port;
    demoServer.yarp().attachAsServer(port);
    if (!port.open("/demoServer")) { return 1; }
    while (true) {
        printf("Server running happily\n");
        yarp::os::Time::delay(10);
    }
    port.close();
    return 0;
}
