// Generated by gencpp from file custom_msgs/Cone.msg
// DO NOT EDIT!


#ifndef CUSTOM_MSGS_MESSAGE_CONE_H
#define CUSTOM_MSGS_MESSAGE_CONE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace custom_msgs
{
template <class ContainerAllocator>
struct Cone_
{
  typedef Cone_<ContainerAllocator> Type;

  Cone_()
    : position()
    , color()
    , confidence(0.0)  {
    }
  Cone_(const ContainerAllocator& _alloc)
    : position(_alloc)
    , color(_alloc)
    , confidence(0.0)  {
  (void)_alloc;
    }



   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _position_type;
  _position_type position;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _color_type;
  _color_type color;

   typedef double _confidence_type;
  _confidence_type confidence;





  typedef boost::shared_ptr< ::custom_msgs::Cone_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::custom_msgs::Cone_<ContainerAllocator> const> ConstPtr;

}; // struct Cone_

typedef ::custom_msgs::Cone_<std::allocator<void> > Cone;

typedef boost::shared_ptr< ::custom_msgs::Cone > ConePtr;
typedef boost::shared_ptr< ::custom_msgs::Cone const> ConeConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::custom_msgs::Cone_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::custom_msgs::Cone_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::custom_msgs::Cone_<ContainerAllocator1> & lhs, const ::custom_msgs::Cone_<ContainerAllocator2> & rhs)
{
  return lhs.position == rhs.position &&
    lhs.color == rhs.color &&
    lhs.confidence == rhs.confidence;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::custom_msgs::Cone_<ContainerAllocator1> & lhs, const ::custom_msgs::Cone_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace custom_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::custom_msgs::Cone_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::custom_msgs::Cone_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msgs::Cone_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::custom_msgs::Cone_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msgs::Cone_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::custom_msgs::Cone_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::custom_msgs::Cone_<ContainerAllocator> >
{
  static const char* value()
  {
    return "43bb11ea9f3016ef6978a1dc3a27791d";
  }

  static const char* value(const ::custom_msgs::Cone_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x43bb11ea9f3016efULL;
  static const uint64_t static_value2 = 0x6978a1dc3a27791dULL;
};

template<class ContainerAllocator>
struct DataType< ::custom_msgs::Cone_<ContainerAllocator> >
{
  static const char* value()
  {
    return "custom_msgs/Cone";
  }

  static const char* value(const ::custom_msgs::Cone_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::custom_msgs::Cone_<ContainerAllocator> >
{
  static const char* value()
  {
    return "geometry_msgs/Point position\n"
"string color\n"
"float64 confidence\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::custom_msgs::Cone_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::custom_msgs::Cone_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.position);
      stream.next(m.color);
      stream.next(m.confidence);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct Cone_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::custom_msgs::Cone_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::custom_msgs::Cone_<ContainerAllocator>& v)
  {
    s << indent << "position: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.position);
    s << indent << "color: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.color);
    s << indent << "confidence: ";
    Printer<double>::stream(s, indent + "  ", v.confidence);
  }
};

} // namespace message_operations
} // namespace ros

#endif // CUSTOM_MSGS_MESSAGE_CONE_H
