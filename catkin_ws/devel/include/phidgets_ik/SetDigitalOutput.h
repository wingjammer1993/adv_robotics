// Generated by gencpp from file phidgets_ik/SetDigitalOutput.msg
// DO NOT EDIT!


#ifndef PHIDGETS_IK_MESSAGE_SETDIGITALOUTPUT_H
#define PHIDGETS_IK_MESSAGE_SETDIGITALOUTPUT_H

#include <ros/service_traits.h>


#include <phidgets_ik/SetDigitalOutputRequest.h>
#include <phidgets_ik/SetDigitalOutputResponse.h>


namespace phidgets_ik
{

struct SetDigitalOutput
{

typedef SetDigitalOutputRequest Request;
typedef SetDigitalOutputResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct SetDigitalOutput
} // namespace phidgets_ik


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::phidgets_ik::SetDigitalOutput > {
  static const char* value()
  {
    return "8496af00b3dd95e3884fd81d8e38f019";
  }

  static const char* value(const ::phidgets_ik::SetDigitalOutput&) { return value(); }
};

template<>
struct DataType< ::phidgets_ik::SetDigitalOutput > {
  static const char* value()
  {
    return "phidgets_ik/SetDigitalOutput";
  }

  static const char* value(const ::phidgets_ik::SetDigitalOutput&) { return value(); }
};


// service_traits::MD5Sum< ::phidgets_ik::SetDigitalOutputRequest> should match 
// service_traits::MD5Sum< ::phidgets_ik::SetDigitalOutput > 
template<>
struct MD5Sum< ::phidgets_ik::SetDigitalOutputRequest>
{
  static const char* value()
  {
    return MD5Sum< ::phidgets_ik::SetDigitalOutput >::value();
  }
  static const char* value(const ::phidgets_ik::SetDigitalOutputRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::phidgets_ik::SetDigitalOutputRequest> should match 
// service_traits::DataType< ::phidgets_ik::SetDigitalOutput > 
template<>
struct DataType< ::phidgets_ik::SetDigitalOutputRequest>
{
  static const char* value()
  {
    return DataType< ::phidgets_ik::SetDigitalOutput >::value();
  }
  static const char* value(const ::phidgets_ik::SetDigitalOutputRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::phidgets_ik::SetDigitalOutputResponse> should match 
// service_traits::MD5Sum< ::phidgets_ik::SetDigitalOutput > 
template<>
struct MD5Sum< ::phidgets_ik::SetDigitalOutputResponse>
{
  static const char* value()
  {
    return MD5Sum< ::phidgets_ik::SetDigitalOutput >::value();
  }
  static const char* value(const ::phidgets_ik::SetDigitalOutputResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::phidgets_ik::SetDigitalOutputResponse> should match 
// service_traits::DataType< ::phidgets_ik::SetDigitalOutput > 
template<>
struct DataType< ::phidgets_ik::SetDigitalOutputResponse>
{
  static const char* value()
  {
    return DataType< ::phidgets_ik::SetDigitalOutput >::value();
  }
  static const char* value(const ::phidgets_ik::SetDigitalOutputResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // PHIDGETS_IK_MESSAGE_SETDIGITALOUTPUT_H
