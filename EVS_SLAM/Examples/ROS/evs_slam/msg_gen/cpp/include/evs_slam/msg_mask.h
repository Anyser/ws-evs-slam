/* Auto-generated by genmsg_cpp for file /home/lapyr/catkin_ws_slam/src/EVS_SLAM/Examples/ROS/evs_slam/msg/msg_mask.msg */
#ifndef EVS_SLAM_MESSAGE_MSG_MASK_H
#define EVS_SLAM_MESSAGE_MSG_MASK_H
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

namespace evs_slam
{
template <class ContainerAllocator>
struct msg_mask_ {
  typedef msg_mask_<ContainerAllocator> Type;

  msg_mask_()
  : header()
  , data()
  {
  }

  msg_mask_(const ContainerAllocator& _alloc)
  : header(_alloc)
  , data(_alloc)
  {
  }

  typedef  ::std_msgs::Header_<ContainerAllocator>  _header_type;
   ::std_msgs::Header_<ContainerAllocator>  header;

  typedef std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  _data_type;
  std::vector<uint8_t, typename ContainerAllocator::template rebind<uint8_t>::other >  data;


  typedef boost::shared_ptr< ::evs_slam::msg_mask_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::evs_slam::msg_mask_<ContainerAllocator>  const> ConstPtr;
}; // struct msg_mask
typedef  ::evs_slam::msg_mask_<std::allocator<void> > msg_mask;

typedef boost::shared_ptr< ::evs_slam::msg_mask> msg_maskPtr;
typedef boost::shared_ptr< ::evs_slam::msg_mask const> msg_maskConstPtr;


template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const  ::evs_slam::msg_mask_<ContainerAllocator> & v)
{
  ros::message_operations::Printer< ::evs_slam::msg_mask_<ContainerAllocator> >::stream(s, "", v);
  return s;}

} // namespace evs_slam

namespace ros
{
namespace message_traits
{
template<class ContainerAllocator> struct IsMessage< ::evs_slam::msg_mask_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct IsMessage< ::evs_slam::msg_mask_<ContainerAllocator>  const> : public TrueType {};
template<class ContainerAllocator>
struct MD5Sum< ::evs_slam::msg_mask_<ContainerAllocator> > {
  static const char* value() 
  {
    return "8903b686ebe5db3477e83c6d0bb149f8";
  }

  static const char* value(const  ::evs_slam::msg_mask_<ContainerAllocator> &) { return value(); } 
  static const uint64_t static_value1 = 0x8903b686ebe5db34ULL;
  static const uint64_t static_value2 = 0x77e83c6d0bb149f8ULL;
};

template<class ContainerAllocator>
struct DataType< ::evs_slam::msg_mask_<ContainerAllocator> > {
  static const char* value() 
  {
    return "evs_slam/msg_mask";
  }

  static const char* value(const  ::evs_slam::msg_mask_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator>
struct Definition< ::evs_slam::msg_mask_<ContainerAllocator> > {
  static const char* value() 
  {
    return "Header header\n\
uint8[] data\n\
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

  static const char* value(const  ::evs_slam::msg_mask_<ContainerAllocator> &) { return value(); } 
};

template<class ContainerAllocator> struct HasHeader< ::evs_slam::msg_mask_<ContainerAllocator> > : public TrueType {};
template<class ContainerAllocator> struct HasHeader< const ::evs_slam::msg_mask_<ContainerAllocator> > : public TrueType {};
} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

template<class ContainerAllocator> struct Serializer< ::evs_slam::msg_mask_<ContainerAllocator> >
{
  template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
  {
    stream.next(m.header);
    stream.next(m.data);
  }

  ROS_DECLARE_ALLINONE_SERIALIZER
}; // struct msg_mask_
} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::evs_slam::msg_mask_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const  ::evs_slam::msg_mask_<ContainerAllocator> & v) 
  {
    s << indent << "header: ";
s << std::endl;
    Printer< ::std_msgs::Header_<ContainerAllocator> >::stream(s, indent + "  ", v.header);
    s << indent << "data[]" << std::endl;
    for (size_t i = 0; i < v.data.size(); ++i)
    {
      s << indent << "  data[" << i << "]: ";
      Printer<uint8_t>::stream(s, indent + "  ", v.data[i]);
    }
  }
};


} // namespace message_operations
} // namespace ros

#endif // EVS_SLAM_MESSAGE_MSG_MASK_H
