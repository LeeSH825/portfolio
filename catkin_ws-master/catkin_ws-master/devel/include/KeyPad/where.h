// Generated by gencpp from file KeyPad/where.msg
// DO NOT EDIT!


#ifndef KEYPAD_MESSAGE_WHERE_H
#define KEYPAD_MESSAGE_WHERE_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace KeyPad
{
template <class ContainerAllocator>
struct where_
{
  typedef where_<ContainerAllocator> Type;

  where_()
    : data(0)  {
    }
  where_(const ContainerAllocator& _alloc)
    : data(0)  {
  (void)_alloc;
    }



   typedef int32_t _data_type;
  _data_type data;





  typedef boost::shared_ptr< ::KeyPad::where_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::KeyPad::where_<ContainerAllocator> const> ConstPtr;

}; // struct where_

typedef ::KeyPad::where_<std::allocator<void> > where;

typedef boost::shared_ptr< ::KeyPad::where > wherePtr;
typedef boost::shared_ptr< ::KeyPad::where const> whereConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::KeyPad::where_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::KeyPad::where_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace KeyPad

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'KeyPad': ['/home/cseecar/catkin_ws/src/KeyPad/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::KeyPad::where_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::KeyPad::where_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::KeyPad::where_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::KeyPad::where_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::KeyPad::where_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::KeyPad::where_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::KeyPad::where_<ContainerAllocator> >
{
  static const char* value()
  {
    return "da5909fbe378aeaf85e547e830cc1bb7";
  }

  static const char* value(const ::KeyPad::where_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xda5909fbe378aeafULL;
  static const uint64_t static_value2 = 0x85e547e830cc1bb7ULL;
};

template<class ContainerAllocator>
struct DataType< ::KeyPad::where_<ContainerAllocator> >
{
  static const char* value()
  {
    return "KeyPad/where";
  }

  static const char* value(const ::KeyPad::where_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::KeyPad::where_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 data\n\
";
  }

  static const char* value(const ::KeyPad::where_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::KeyPad::where_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.data);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct where_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::KeyPad::where_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::KeyPad::where_<ContainerAllocator>& v)
  {
    s << indent << "data: ";
    Printer<int32_t>::stream(s, indent + "  ", v.data);
  }
};

} // namespace message_operations
} // namespace ros

#endif // KEYPAD_MESSAGE_WHERE_H
