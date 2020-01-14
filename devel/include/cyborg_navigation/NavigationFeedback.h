// Generated by gencpp from file cyborg_navigation/NavigationFeedback.msg
// DO NOT EDIT!


#ifndef CYBORG_NAVIGATION_MESSAGE_NAVIGATIONFEEDBACK_H
#define CYBORG_NAVIGATION_MESSAGE_NAVIGATIONFEEDBACK_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace cyborg_navigation
{
template <class ContainerAllocator>
struct NavigationFeedback_
{
  typedef NavigationFeedback_<ContainerAllocator> Type;

  NavigationFeedback_()
    : status()  {
    }
  NavigationFeedback_(const ContainerAllocator& _alloc)
    : status(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other >  _status_type;
  _status_type status;





  typedef boost::shared_ptr< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> const> ConstPtr;

}; // struct NavigationFeedback_

typedef ::cyborg_navigation::NavigationFeedback_<std::allocator<void> > NavigationFeedback;

typedef boost::shared_ptr< ::cyborg_navigation::NavigationFeedback > NavigationFeedbackPtr;
typedef boost::shared_ptr< ::cyborg_navigation::NavigationFeedback const> NavigationFeedbackConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace cyborg_navigation

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg'], 'cyborg_navigation': ['/home/lassegoncz/catkin_ws/devel/share/cyborg_navigation/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4fe5af303955c287688e7347e9b00278";
  }

  static const char* value(const ::cyborg_navigation::NavigationFeedback_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4fe5af303955c287ULL;
  static const uint64_t static_value2 = 0x688e7347e9b00278ULL;
};

template<class ContainerAllocator>
struct DataType< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "cyborg_navigation/NavigationFeedback";
  }

  static const char* value(const ::cyborg_navigation::NavigationFeedback_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback\n\
string status\n\
";
  }

  static const char* value(const ::cyborg_navigation::NavigationFeedback_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.status);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct NavigationFeedback_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::cyborg_navigation::NavigationFeedback_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::cyborg_navigation::NavigationFeedback_<ContainerAllocator>& v)
  {
    s << indent << "status: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename ContainerAllocator::template rebind<char>::other > >::stream(s, indent + "  ", v.status);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CYBORG_NAVIGATION_MESSAGE_NAVIGATIONFEEDBACK_H
