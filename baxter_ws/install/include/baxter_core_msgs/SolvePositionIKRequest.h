// Generated by gencpp from file baxter_core_msgs/SolvePositionIKRequest.msg
// DO NOT EDIT!


#ifndef BAXTER_CORE_MSGS_MESSAGE_SOLVEPOSITIONIKREQUEST_H
#define BAXTER_CORE_MSGS_MESSAGE_SOLVEPOSITIONIKREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

namespace baxter_core_msgs
{
template <class ContainerAllocator>
struct SolvePositionIKRequest_
{
  typedef SolvePositionIKRequest_<ContainerAllocator> Type;

  SolvePositionIKRequest_()
    : pose_stamp()
    , seed_angles()
    , seed_mode(0)  {
    }
  SolvePositionIKRequest_(const ContainerAllocator& _alloc)
    : pose_stamp(_alloc)
    , seed_angles(_alloc)
    , seed_mode(0)  {
  (void)_alloc;
    }



   typedef std::vector< ::geometry_msgs::PoseStamped_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::other >  _pose_stamp_type;
  _pose_stamp_type pose_stamp;

   typedef std::vector< ::sensor_msgs::JointState_<ContainerAllocator> , typename ContainerAllocator::template rebind< ::sensor_msgs::JointState_<ContainerAllocator> >::other >  _seed_angles_type;
  _seed_angles_type seed_angles;

   typedef uint8_t _seed_mode_type;
  _seed_mode_type seed_mode;



  enum {
    SEED_AUTO = 0u,
    SEED_USER = 1u,
    SEED_CURRENT = 2u,
    SEED_NS_MAP = 3u,
  };


  typedef boost::shared_ptr< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> const> ConstPtr;

}; // struct SolvePositionIKRequest_

typedef ::baxter_core_msgs::SolvePositionIKRequest_<std::allocator<void> > SolvePositionIKRequest;

typedef boost::shared_ptr< ::baxter_core_msgs::SolvePositionIKRequest > SolvePositionIKRequestPtr;
typedef boost::shared_ptr< ::baxter_core_msgs::SolvePositionIKRequest const> SolvePositionIKRequestConstPtr;

// constants requiring out of line definition

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace baxter_core_msgs

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'baxter_core_msgs': ['/scratch/shared/baxter_ws/src/baxter_common/baxter_core_msgs/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "2587e42983d0081d0a2288230991073b";
  }

  static const char* value(const ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x2587e42983d0081dULL;
  static const uint64_t static_value2 = 0x0a2288230991073bULL;
};

template<class ContainerAllocator>
struct DataType< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "baxter_core_msgs/SolvePositionIKRequest";
  }

  static const char* value(const ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n\
geometry_msgs/PoseStamped[] pose_stamp\n\
\n\
\n\
\n\
\n\
sensor_msgs/JointState[] seed_angles\n\
\n\
\n\
\n\
\n\
\n\
uint8 SEED_AUTO    = 0\n\
uint8 SEED_USER    = 1\n\
uint8 SEED_CURRENT = 2\n\
uint8 SEED_NS_MAP  = 3\n\
\n\
uint8 seed_mode\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
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
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: sensor_msgs/JointState\n\
# This is a message that holds data to describe the state of a set of torque controlled joints. \n\
#\n\
# The state of each joint (revolute or prismatic) is defined by:\n\
#  * the position of the joint (rad or m),\n\
#  * the velocity of the joint (rad/s or m/s) and \n\
#  * the effort that is applied in the joint (Nm or N).\n\
#\n\
# Each joint is uniquely identified by its name\n\
# The header specifies the time at which the joint states were recorded. All the joint states\n\
# in one message have to be recorded at the same time.\n\
#\n\
# This message consists of a multiple arrays, one for each part of the joint state. \n\
# The goal is to make each of the fields optional. When e.g. your joints have no\n\
# effort associated with them, you can leave the effort array empty. \n\
#\n\
# All arrays in this message should have the same size, or be empty.\n\
# This is the only way to uniquely associate the joint name with the correct\n\
# states.\n\
\n\
\n\
Header header\n\
\n\
string[] name\n\
float64[] position\n\
float64[] velocity\n\
float64[] effort\n\
";
  }

  static const char* value(const ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.pose_stamp);
      stream.next(m.seed_angles);
      stream.next(m.seed_mode);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct SolvePositionIKRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::baxter_core_msgs::SolvePositionIKRequest_<ContainerAllocator>& v)
  {
    s << indent << "pose_stamp[]" << std::endl;
    for (size_t i = 0; i < v.pose_stamp.size(); ++i)
    {
      s << indent << "  pose_stamp[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "    ", v.pose_stamp[i]);
    }
    s << indent << "seed_angles[]" << std::endl;
    for (size_t i = 0; i < v.seed_angles.size(); ++i)
    {
      s << indent << "  seed_angles[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::sensor_msgs::JointState_<ContainerAllocator> >::stream(s, indent + "    ", v.seed_angles[i]);
    }
    s << indent << "seed_mode: ";
    Printer<uint8_t>::stream(s, indent + "  ", v.seed_mode);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BAXTER_CORE_MSGS_MESSAGE_SOLVEPOSITIONIKREQUEST_H
