// This is an automatically-generated file.
// It could get re-generated if the ALLOW_IDL_GENERATION flag is on.

#ifndef YARP_THRIFT_GENERATOR_Demo
#define YARP_THRIFT_GENERATOR_Demo

#include <yarp/os/Wire.h>
#include <yarp/os/idl/WireTypes.h>
#include <iostream>
class Demo;


class Demo : public yarp::os::Wire {
public:
  Demo();
  virtual int32_t get_answer();
  virtual bool set_answer(const int32_t rightAnswer);
  virtual int32_t add_one(const int32_t x);
  virtual bool start();
  virtual bool stop();
  virtual bool is_running();
  virtual bool read(yarp::os::ConnectionReader& connection);
  virtual std::vector<std::string> help(const std::string& functionName="--all");
};

#endif
