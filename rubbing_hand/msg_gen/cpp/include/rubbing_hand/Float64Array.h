/* Auto-generated by genmsg_cpp for file /home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/msg/Float64Array.msg */
#ifndef RUBBING_HAND_MESSAGE_FLOAT64ARRAY_H
#define RUBBING_HAND_MESSAGE_FLOAT64ARRAY_H
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


namespace rubbing_hand
{
template <class ContainerAllocator>
struct Float64Array_ {
  typedef Float64Array_<ContainerAllocator> Type;

  Float64Array_()
  : data()
  {
  }

  Float64Array_(const ContainerAllocator& _alloc)
  : data(_alloc)
  {
  }

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _data_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  data;


  typedef boost::shared_ptr< ::rubbing_hand::Float64Array_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rubbing_hand::Float64Array_<ContainerAllocator>  const> ConstPtr;
}; // struct Float64Array
typedef  ::rubbing_hand::Float64Array_<std::allocator<void> > Float64Array;

typedef boost::shared_ptr< ::rubbing_hand::Float64Array> Float64ArrayPtr;
typedef boost::shared_ptr< ::rubbing_hand::Float64Array const> Float64ArrayConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::rubbing_hand::Float64Array_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::rubbing_hand::Float64Array_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace rubbing_hand

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rubbing_hand::Float64Array_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rubbing_hand::Float64Array_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rubbing_hand::Float64Array_<ContainerAllocator> > {
  static const char* value() 
  {
    return "788898178a3da2c3718461eecda8f714";
  }

  static const char* value(const  ::rubbing_hand::Float64Array_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x788898178a3da2c3ULL;
  static const uint64_t static_value2 = 0x718461eecda8f714ULL;
};

template<class ContainerAllocator>
struct DataType< ::rubbing_hand::Float64Array_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rubbing_hand/Float64Array";
  }

  static const char* value(const  ::rubbing_hand::Float64Array_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rubbing_hand::Float64Array_<ContainerAllocator> > {
  static const char* value() 
  {
    return "float64[] data\n\
\n\
";
  }

  static const char* value(const  ::rubbing_hand::Float64Array_<ContainerAllocator> &) { return value(); } 
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rubbing_hand::Float64Array_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct Float64Array_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rubbing_hand::Float64Array_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::rubbing_hand::Float64Array_<ContainerAllocator> & v) 
  {
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.data[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // RUBBING_HAND_MESSAGE_FLOAT64ARRAY_H

