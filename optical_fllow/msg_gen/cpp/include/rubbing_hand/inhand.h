/* Auto-generated by genmsg_cpp for file /home/suzuki/ros_ws/ay_tools/fingervision/suzuki/rubbing_hand/msg/inhand.msg */
#ifndef RUBBING_HAND_MESSAGE_INHAND_H
#define RUBBING_HAND_MESSAGE_INHAND_H
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

#include "std_msgs/Header.h"

namespace rubbing_hand
{
template <class ContainerAllocator>
struct inhand_ {
  typedef inhand_<ContainerAllocator> Type;

  inhand_()
  : header()
  , interval(0.0)
  , MV(0.0)
  , mv_s()
  , obj_orientation(0.0)
  , obj_orientation_filtered(0.0)
  , d_obj_orientation_filtered(0.0)
  , target_obj_orientation(0.0)
  , target_d_obj_orientation(0.0)
  , omega_d(0.0)
  , th_slip(0.0)
  , MV_i()
  , MV_o()
  , process_f(0)
  , debag()
  {
  }

  inhand_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , interval(0.0)
  , MV(0.0)
  , mv_s(_alloc)
  , obj_orientation(0.0)
  , obj_orientation_filtered(0.0)
  , d_obj_orientation_filtered(0.0)
  , target_obj_orientation(0.0)
  , target_d_obj_orientation(0.0)
  , omega_d(0.0)
  , th_slip(0.0)
  , MV_i(_alloc)
  , MV_o(_alloc)
  , process_f(0)
  , debag(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef double _interval_type;
  double interval;

  typedef double _MV_type;
  double MV;

  typedef std::vector<float, typename ContainerAllocator::template rebind<float>::other >  _mv_s_type;
  std::vector<float, typename ContainerAllocator::template rebind<float>::other >  mv_s;

  typedef double _obj_orientation_type;
  double obj_orientation;

  typedef double _obj_orientation_filtered_type;
  double obj_orientation_filtered;

  typedef double _d_obj_orientation_filtered_type;
  double d_obj_orientation_filtered;

  typedef double _target_obj_orientation_type;
  double target_obj_orientation;

  typedef double _target_d_obj_orientation_type;
  double target_d_obj_orientation;

  typedef double _omega_d_type;
  double omega_d;

  typedef double _th_slip_type;
  double th_slip;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _MV_i_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  MV_i;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _MV_o_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  MV_o;

  typedef int32_t _process_f_type;
  int32_t process_f;

  typedef std::vector<double, typename ContainerAllocator::template rebind<double>::other >  _debag_type;
  std::vector<double, typename ContainerAllocator::template rebind<double>::other >  debag;


  typedef boost::shared_ptr< ::rubbing_hand::inhand_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::rubbing_hand::inhand_<ContainerAllocator>  const> ConstPtr;
}; // struct inhand
typedef  ::rubbing_hand::inhand_<std::allocator<void> > inhand;

typedef boost::shared_ptr< ::rubbing_hand::inhand> inhandPtr;
typedef boost::shared_ptr< ::rubbing_hand::inhand const> inhandConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::rubbing_hand::inhand_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::rubbing_hand::inhand_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace rubbing_hand

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::rubbing_hand::inhand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::rubbing_hand::inhand_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::rubbing_hand::inhand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "266b6584c02ea6d87499d928f5c0eec9";
  }

  static const char* value(const  ::rubbing_hand::inhand_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x266b6584c02ea6d8ULL;
  static const uint64_t static_value2 = 0x7499d928f5c0eec9ULL;
};

template<class ContainerAllocator>
struct DataType< ::rubbing_hand::inhand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "rubbing_hand/inhand";
  }

  static const char* value(const  ::rubbing_hand::inhand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::rubbing_hand::inhand_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
\n\
# --------dynamixel--------\n\
\n\
# ダイナミクセルへ渡した目標指間距離\n\
# 取得した指間距離へ操作量を加えている\n\
# interval = got_interval + MV\n\
float64 interval\n\
\n\
# 指間距離へ加える操作量(Manipulated Variable)\n\
float64 MV\n\
\n\
# -------------------------\n\
\n\
# --------fingervision-----\n\
\n\
# Array of slips (slip distribution), which is a serialized list of 3x3 matrix.\n\
# Each cell in the 3x3 matrix is the sum of moving pixels in the cell.\n\
float32[] mv_s\n\
\n\
# objectの角度 [deg]\n\
float64 obj_orientation\n\
\n\
# objectの角度 [deg]  obj_orientationをsmaでフィルターしてる（filter_node参照）\n\
float64 obj_orientation_filtered\n\
\n\
# objectの角速度 [deg/s]　obj_orientation_filteredの差分をとったものをsmaでフィルターしてる（filter_node参照）\n\
float64 d_obj_orientation_filtered\n\
\n\
# -------------------------\n\
\n\
# ---------inhand----------\n\
\n\
# 目標角度\n\
float64 target_obj_orientation\n\
\n\
# 目標角速度\n\
float64 target_d_obj_orientation\n\
\n\
# 目標角速度と取得した角速度の差\n\
# d_obj_orientation_filtered - target_d_obj_orientation\n\
float64 omega_d\n\
\n\
# mv_sの和からすべり判定をする際のしきい値\n\
float64 th_slip\n\
\n\
# 取得した角速度d_obj_orientation_filteredと目標角速度target_d_obj_orientationとの差から操作量MVを決めるパラメータ\n\
# MV_input  = [neutral_min, neutral_max , drop]\n\
# MV_output = [open, close, quick_close]\n\
# drop < d_obj_orientation_filtered : quick_close\n\
# d_omega <= neutral_min : open\n\
# neutral_min < d_omega <= neutral_max : 0\n\
# neutral_max < d_omega : close\n\
float64[] MV_i\n\
float64[] MV_o\n\
\n\
# マニピュレーション実行区間を表すフラグ\n\
# 0: してない， 1:マニピュレーション終了後の数秒間， 2:マニピュレーション中\n\
int32 process_f\n\
\n\
# -------------------------\n\
\n\
# なんでも入れていいよ\n\
float64[] debag\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
string frame_id\n\
\n\
";
  }

  static const char* value(const  ::rubbing_hand::inhand_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::rubbing_hand::inhand_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::rubbing_hand::inhand_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::rubbing_hand::inhand_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.interval);
    stream.next(m.MV);
    stream.next(m.mv_s);
    stream.next(m.obj_orientation);
    stream.next(m.obj_orientation_filtered);
    stream.next(m.d_obj_orientation_filtered);
    stream.next(m.target_obj_orientation);
    stream.next(m.target_d_obj_orientation);
    stream.next(m.omega_d);
    stream.next(m.th_slip);
    stream.next(m.MV_i);
    stream.next(m.MV_o);
    stream.next(m.process_f);
    stream.next(m.debag);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct inhand_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::rubbing_hand::inhand_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::rubbing_hand::inhand_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "interval: ";
    Printer<double>::stream(s, indent + "  ", v.interval);
    s << indent << "MV: ";
    Printer<double>::stream(s, indent + "  ", v.MV);
    s << indent << "mv_s[]" << std::endl;
    for (size_t i = 0; i < v.mv_s.size(); ++i)
    {
      s << indent << "  mv_s[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.mv_s[i]);
    }
    s << indent << "obj_orientation: ";
    Printer<double>::stream(s, indent + "  ", v.obj_orientation);
    s << indent << "obj_orientation_filtered: ";
    Printer<double>::stream(s, indent + "  ", v.obj_orientation_filtered);
    s << indent << "d_obj_orientation_filtered: ";
    Printer<double>::stream(s, indent + "  ", v.d_obj_orientation_filtered);
    s << indent << "target_obj_orientation: ";
    Printer<double>::stream(s, indent + "  ", v.target_obj_orientation);
    s << indent << "target_d_obj_orientation: ";
    Printer<double>::stream(s, indent + "  ", v.target_d_obj_orientation);
    s << indent << "omega_d: ";
    Printer<double>::stream(s, indent + "  ", v.omega_d);
    s << indent << "th_slip: ";
    Printer<double>::stream(s, indent + "  ", v.th_slip);
    s << indent << "MV_i[]" << std::endl;
    for (size_t i = 0; i < v.MV_i.size(); ++i)
    {
      s << indent << "  MV_i[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.MV_i[i]);
    }
    s << indent << "MV_o[]" << std::endl;
    for (size_t i = 0; i < v.MV_o.size(); ++i)
    {
      s << indent << "  MV_o[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.MV_o[i]);
    }
    s << indent << "process_f: ";
    Printer<int32_t>::stream(s, indent + "  ", v.process_f);
    s << indent << "debag[]" << std::endl;
    for (size_t i = 0; i < v.debag.size(); ++i)
    {
      s << indent << "  debag[" << i << "]: ";
      Printer<double>::stream(s, indent + "  ", v.debag[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // RUBBING_HAND_MESSAGE_INHAND_H
