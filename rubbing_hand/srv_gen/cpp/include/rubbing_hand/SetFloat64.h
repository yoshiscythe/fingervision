/* Auto-generated by genmsg_cpp for file /home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/srv/SetFloat64.srv */
#ifndef RUBBING_HAND_SERVICE_SETFLOAT64_H
#define RUBBING_HAND_SERVICE_SETFLOAT64_H
#include <string>
#include <vector>
#include <map>
#include <ostream>
#include "ros/serialization.h"
#include "ros/builtin_message_traits.h"
#include "ros/message_operations.h"
#include "ros/time.h"

#include "ros/macros.h"

#include "ros/assert.h"

#include "ros/service_traits.h"




namespace rubbing_hand
{
template <class ContainerAllocator>
struct SetFloat64Request_ {
  typedef SetFloat64Request_<ContainerAllocator> Type;

  SetFloat64Request_()
  : data(0.0)
  {
  }

  SetFloat64Request_(const ContainerAllocator& _alloc)
  : data(0.0)
  {
  }

  typedef double _data_type;
  double data;


  typedef boost::shared_ptr< ::rubbing_hand::SetFloat64Request_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rubbing_hand::SetFloat64Request_<ContainerAllocator>  const> ConstPtr;
}; // struct SetFloat64Request
typedef  ::rubbing_hand::SetFloat64Request_<std::allocator<void> > SetFloat64Request;

typedef boost::shared_ptr< ::rubbing_hand::SetFloat64Request> SetFloat64RequestPtr;
typedef boost::shared_ptr< ::rubbing_hand::SetFloat64Request const> SetFloat64RequestConstPtr;



template <class ContainerAllocator>
struct SetFloat64Response_ {
  typedef SetFloat64Response_<ContainerAllocator> Type;

  SetFloat64Response_()
  : result(false)
  {
  }

  SetFloat64Response_(const ContainerAllocator& _alloc)
  : result(false)
  {
  }

  typedef uint8_t _result_type;
  uint8_t result;


  typedef boost::shared_ptr< ::rubbing_hand::SetFloat64Response_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rubbing_hand::SetFloat64Response_<ContainerAllocator>  const> ConstPtr;
}; // struct SetFloat64Response
typedef  ::rubbing_hand::SetFloat64Response_<std::allocator<void> > SetFloat64Response;

typedef boost::shared_ptr< ::rubbing_hand::SetFloat64Response> SetFloat64ResponsePtr;
typedef boost::shared_ptr< ::rubbing_hand::SetFloat64Response const> SetFloat64ResponseConstPtr;


struct SetFloat64
{

typedef SetFloat64Request Request;
typedef SetFloat64Response Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;
}; // struct SetFloat64
} // namespace rubbing_hand

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rubbing_hand::SetFloat64Request_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rubbing_hand::SetFloat64Request_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rubbing_hand::SetFloat64Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "fdb28210bfa9d7c91146260178d9a584";
  }

  static const char* value(const  ::rubbing_hand::SetFloat64Request_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xfdb28210bfa9d7c9ULL;
  static const uint64_t static_value2 = 0x1146260178d9a584ULL;
};

template<class ContainerAllocator>
struct DataType< ::rubbing_hand::SetFloat64Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rubbing_hand/SetFloat64Request";
  }

  static const char* value(const  ::rubbing_hand::SetFloat64Request_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rubbing_hand::SetFloat64Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64 data\n\
\n\
";
  }

  static const char* value(const  ::rubbing_hand::SetFloat64Request_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rubbing_hand::SetFloat64Request_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros


namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rubbing_hand::SetFloat64Response_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rubbing_hand::SetFloat64Response_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rubbing_hand::SetFloat64Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "eb13ac1f1354ccecb7941ee8fa2192e8";
  }

  static const char* value(const  ::rubbing_hand::SetFloat64Response_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0xeb13ac1f1354ccecULL;
  static const uint64_t static_value2 = 0xb7941ee8fa2192e8ULL;
};

template<class ContainerAllocator>
struct DataType< ::rubbing_hand::SetFloat64Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rubbing_hand/SetFloat64Response";
  }

  static const char* value(const  ::rubbing_hand::SetFloat64Response_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rubbing_hand::SetFloat64Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "bool result\n\
\n\
\n\
";
  }

  static const char* value(const  ::rubbing_hand::SetFloat64Response_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct IsFixedSize< ::rubbing_hand::SetFloat64Response_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rubbing_hand::SetFloat64Request_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct SetFloat64Request_
} // namespace serialization
} // namespace ros


namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rubbing_hand::SetFloat64Response_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.result);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct SetFloat64Response_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace service_traits
{
template<>
struct MD5Sum<rubbing_hand::SetFloat64> {
  static const char* value() 
  {
    return "e914a2ad5395f6f66880d5aed68f5700";
  }

  static const char* value(const rubbing_hand::SetFloat64&) { return value(); } 
};

template<>
struct DataType<rubbing_hand::SetFloat64> {
  static const char* value() 
  {
    return "rubbing_hand/SetFloat64";
  }

  static const char* value(const rubbing_hand::SetFloat64&) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rubbing_hand::SetFloat64Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e914a2ad5395f6f66880d5aed68f5700";
  }

  static const char* value(const rubbing_hand::SetFloat64Request_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rubbing_hand::SetFloat64Request_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rubbing_hand/SetFloat64";
  }

  static const char* value(const rubbing_hand::SetFloat64Request_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct MD5Sum<rubbing_hand::SetFloat64Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "e914a2ad5395f6f66880d5aed68f5700";
  }

  static const char* value(const rubbing_hand::SetFloat64Response_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct DataType<rubbing_hand::SetFloat64Response_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rubbing_hand/SetFloat64";
  }

  static const char* value(const rubbing_hand::SetFloat64Response_<ContainerAllocator> &) { return value(); } 
};

} // namespace service_traits
} // namespace ros

#endif // RUBBING_HAND_SERVICE_SETFLOAT64_H

