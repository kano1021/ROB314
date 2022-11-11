// Generated by gencpp from file manipulation_msgs/PlaceLocation.msg
// DO NOT EDIT!


#ifndef MANIPULATION_MSGS_MESSAGE_PLACELOCATION_H
#define MANIPULATION_MSGS_MESSAGE_PLACELOCATION_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <manipulation_msgs/GripperTranslation.h>
#include <manipulation_msgs/GripperTranslation.h>

namespace manipulation_msgs
{
template <class ContainerAllocator>
struct PlaceLocation_
{
  typedef PlaceLocation_<ContainerAllocator> Type;

  PlaceLocation_()
    : id()
    , post_place_posture()
    , place_pose()
    , approach()
    , retreat()
    , allowed_touch_objects()  {
    }
  PlaceLocation_(const ContainerAllocator& _alloc)
    : id(_alloc)
    , post_place_posture(_alloc)
    , place_pose(_alloc)
    , approach(_alloc)
    , retreat(_alloc)
    , allowed_touch_objects(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _id_type;
  _id_type id;

   typedef  ::sensor_msgs::JointState_<ContainerAllocator>  _post_place_posture_type;
  _post_place_posture_type post_place_posture;

   typedef  ::geometry_msgs::PoseStamped_<ContainerAllocator>  _place_pose_type;
  _place_pose_type place_pose;

   typedef  ::manipulation_msgs::GripperTranslation_<ContainerAllocator>  _approach_type;
  _approach_type approach;

   typedef  ::manipulation_msgs::GripperTranslation_<ContainerAllocator>  _retreat_type;
  _retreat_type retreat;

   typedef std::vector<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>> _allowed_touch_objects_type;
  _allowed_touch_objects_type allowed_touch_objects;





  typedef boost::shared_ptr< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> const> ConstPtr;

}; // struct PlaceLocation_

typedef ::manipulation_msgs::PlaceLocation_<std::allocator<void> > PlaceLocation;

typedef boost::shared_ptr< ::manipulation_msgs::PlaceLocation > PlaceLocationPtr;
typedef boost::shared_ptr< ::manipulation_msgs::PlaceLocation const> PlaceLocationConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::manipulation_msgs::PlaceLocation_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::manipulation_msgs::PlaceLocation_<ContainerAllocator1> & lhs, const ::manipulation_msgs::PlaceLocation_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id &&
    lhs.post_place_posture == rhs.post_place_posture &&
    lhs.place_pose == rhs.place_pose &&
    lhs.approach == rhs.approach &&
    lhs.retreat == rhs.retreat &&
    lhs.allowed_touch_objects == rhs.allowed_touch_objects;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::manipulation_msgs::PlaceLocation_<ContainerAllocator1> & lhs, const ::manipulation_msgs::PlaceLocation_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace manipulation_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsFixedSize< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0139dab9852add0e64233c5fb3b8a25a";
  }

  static const char* value(const ::manipulation_msgs::PlaceLocation_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0139dab9852add0eULL;
  static const uint64_t static_value2 = 0x64233c5fb3b8a25aULL;
};

template<class ContainerAllocator>
struct DataType< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "manipulation_msgs/PlaceLocation";
  }

  static const char* value(const ::manipulation_msgs::PlaceLocation_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# A name for this grasp\n"
"string id\n"
"\n"
"# The internal posture of the hand for the grasp\n"
"# positions and efforts are used\n"
"sensor_msgs/JointState post_place_posture\n"
"\n"
"# The position of the end-effector for the grasp relative to a reference frame \n"
"# (that is always specified elsewhere, not in this message)\n"
"geometry_msgs/PoseStamped place_pose\n"
"\n"
"# The approach motion\n"
"GripperTranslation approach\n"
"\n"
"# The retreat motion\n"
"GripperTranslation retreat\n"
"\n"
"# an optional list of obstacles that we have semantic information about\n"
"# and that can be touched/pushed/moved in the course of grasping\n"
"string[] allowed_touch_objects\n"
"\n"
"================================================================================\n"
"MSG: sensor_msgs/JointState\n"
"# This is a message that holds data to describe the state of a set of torque controlled joints. \n"
"#\n"
"# The state of each joint (revolute or prismatic) is defined by:\n"
"#  * the position of the joint (rad or m),\n"
"#  * the velocity of the joint (rad/s or m/s) and \n"
"#  * the effort that is applied in the joint (Nm or N).\n"
"#\n"
"# Each joint is uniquely identified by its name\n"
"# The header specifies the time at which the joint states were recorded. All the joint states\n"
"# in one message have to be recorded at the same time.\n"
"#\n"
"# This message consists of a multiple arrays, one for each part of the joint state. \n"
"# The goal is to make each of the fields optional. When e.g. your joints have no\n"
"# effort associated with them, you can leave the effort array empty. \n"
"#\n"
"# All arrays in this message should have the same size, or be empty.\n"
"# This is the only way to uniquely associate the joint name with the correct\n"
"# states.\n"
"\n"
"\n"
"Header header\n"
"\n"
"string[] name\n"
"float64[] position\n"
"float64[] velocity\n"
"float64[] effort\n"
"\n"
"================================================================================\n"
"MSG: std_msgs/Header\n"
"# Standard metadata for higher-level stamped data types.\n"
"# This is generally used to communicate timestamped data \n"
"# in a particular coordinate frame.\n"
"# \n"
"# sequence ID: consecutively increasing ID \n"
"uint32 seq\n"
"#Two-integer timestamp that is expressed as:\n"
"# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n"
"# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n"
"# time-handling sugar is provided by the client library\n"
"time stamp\n"
"#Frame this data is associated with\n"
"string frame_id\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/PoseStamped\n"
"# A Pose with reference coordinate frame and timestamp\n"
"Header header\n"
"Pose pose\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Pose\n"
"# A representation of pose in free space, composed of position and orientation. \n"
"Point position\n"
"Quaternion orientation\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Quaternion\n"
"# This represents an orientation in free space in quaternion form.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
"float64 w\n"
"\n"
"================================================================================\n"
"MSG: manipulation_msgs/GripperTranslation\n"
"# defines a translation for the gripper, used in pickup or place tasks\n"
"# for example for lifting an object off a table or approaching the table for placing\n"
"\n"
"# the direction of the translation\n"
"geometry_msgs/Vector3Stamped direction\n"
"\n"
"# the desired translation distance\n"
"float32 desired_distance\n"
"\n"
"# the min distance that must be considered feasible before the\n"
"# grasp is even attempted\n"
"float32 min_distance\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3Stamped\n"
"# This represents a Vector3 with reference coordinate frame and timestamp\n"
"Header header\n"
"Vector3 vector\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Vector3\n"
"# This represents a vector in free space. \n"
"# It is only meant to represent a direction. Therefore, it does not\n"
"# make sense to apply a translation to it (e.g., when applying a \n"
"# generic rigid transformation to a Vector3, tf2 will only apply the\n"
"# rotation). If you want your data to be translatable too, use the\n"
"# geometry_msgs/Point message instead.\n"
"\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::manipulation_msgs::PlaceLocation_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
      stream.next(m.post_place_posture);
      stream.next(m.place_pose);
      stream.next(m.approach);
      stream.next(m.retreat);
      stream.next(m.allowed_touch_objects);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct PlaceLocation_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::manipulation_msgs::PlaceLocation_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::manipulation_msgs::PlaceLocation_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.id);
    s << indent << "post_place_posture: ";
    s << std::endl;
    Printer< ::sensor_msgs::JointState_<ContainerAllocator> >::stream(s, indent + "  ", v.post_place_posture);
    s << indent << "place_pose: ";
    s << std::endl;
    Printer< ::geometry_msgs::PoseStamped_<ContainerAllocator> >::stream(s, indent + "  ", v.place_pose);
    s << indent << "approach: ";
    s << std::endl;
    Printer< ::manipulation_msgs::GripperTranslation_<ContainerAllocator> >::stream(s, indent + "  ", v.approach);
    s << indent << "retreat: ";
    s << std::endl;
    Printer< ::manipulation_msgs::GripperTranslation_<ContainerAllocator> >::stream(s, indent + "  ", v.retreat);
    s << indent << "allowed_touch_objects[]" << std::endl;
    for (size_t i = 0; i < v.allowed_touch_objects.size(); ++i)
    {
      s << indent << "  allowed_touch_objects[" << i << "]: ";
      Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.allowed_touch_objects[i]);
    }
  }
};

} // namespace message_operations
} // namespace ros

#endif // MANIPULATION_MSGS_MESSAGE_PLACELOCATION_H
