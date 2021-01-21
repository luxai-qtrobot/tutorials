// Generated by gencpp from file qt_gspeech_interface/QTrobotGspeech.msg
// DO NOT EDIT!


#ifndef QT_GSPEECH_INTERFACE_MESSAGE_QTROBOTGSPEECH_H
#define QT_GSPEECH_INTERFACE_MESSAGE_QTROBOTGSPEECH_H

#include <ros/service_traits.h>


#include <qt_gspeech_interface/QTrobotGspeechRequest.h>
#include <qt_gspeech_interface/QTrobotGspeechResponse.h>


namespace qt_gspeech_interface
{

struct QTrobotGspeech
{

typedef QTrobotGspeechRequest Request;
typedef QTrobotGspeechResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct QTrobotGspeech
} // namespace qt_gspeech_interface


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::qt_gspeech_interface::QTrobotGspeech > {
  static const char* value()
  {
    return "2e9160b2c93872c949e46fee310a6d5c";
  }

  static const char* value(const ::qt_gspeech_interface::QTrobotGspeech&) { return value(); }
};

template<>
struct DataType< ::qt_gspeech_interface::QTrobotGspeech > {
  static const char* value()
  {
    return "qt_gspeech_interface/QTrobotGspeech";
  }

  static const char* value(const ::qt_gspeech_interface::QTrobotGspeech&) { return value(); }
};


// service_traits::MD5Sum< ::qt_gspeech_interface::QTrobotGspeechRequest> should match 
// service_traits::MD5Sum< ::qt_gspeech_interface::QTrobotGspeech > 
template<>
struct MD5Sum< ::qt_gspeech_interface::QTrobotGspeechRequest>
{
  static const char* value()
  {
    return MD5Sum< ::qt_gspeech_interface::QTrobotGspeech >::value();
  }
  static const char* value(const ::qt_gspeech_interface::QTrobotGspeechRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_gspeech_interface::QTrobotGspeechRequest> should match 
// service_traits::DataType< ::qt_gspeech_interface::QTrobotGspeech > 
template<>
struct DataType< ::qt_gspeech_interface::QTrobotGspeechRequest>
{
  static const char* value()
  {
    return DataType< ::qt_gspeech_interface::QTrobotGspeech >::value();
  }
  static const char* value(const ::qt_gspeech_interface::QTrobotGspeechRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::qt_gspeech_interface::QTrobotGspeechResponse> should match 
// service_traits::MD5Sum< ::qt_gspeech_interface::QTrobotGspeech > 
template<>
struct MD5Sum< ::qt_gspeech_interface::QTrobotGspeechResponse>
{
  static const char* value()
  {
    return MD5Sum< ::qt_gspeech_interface::QTrobotGspeech >::value();
  }
  static const char* value(const ::qt_gspeech_interface::QTrobotGspeechResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::qt_gspeech_interface::QTrobotGspeechResponse> should match 
// service_traits::DataType< ::qt_gspeech_interface::QTrobotGspeech > 
template<>
struct DataType< ::qt_gspeech_interface::QTrobotGspeechResponse>
{
  static const char* value()
  {
    return DataType< ::qt_gspeech_interface::QTrobotGspeech >::value();
  }
  static const char* value(const ::qt_gspeech_interface::QTrobotGspeechResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // QT_GSPEECH_INTERFACE_MESSAGE_QTROBOTGSPEECH_H