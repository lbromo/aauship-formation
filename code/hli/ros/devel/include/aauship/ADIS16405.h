// Generated by gencpp from file aauship/ADIS16405.msg
// DO NOT EDIT!


#ifndef AAUSHIP_MESSAGE_ADIS16405_H
#define AAUSHIP_MESSAGE_ADIS16405_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace aauship
{
template <class ContainerAllocator>
struct ADIS16405_
{
  typedef ADIS16405_<ContainerAllocator> Type;

  ADIS16405_()
    : supply(0.0)
    , xgyro(0.0)
    , ygyro(0.0)
    , zgyro(0.0)
    , xaccl(0.0)
    , yaccl(0.0)
    , zaccl(0.0)
    , xmagn(0.0)
    , ymagn(0.0)
    , zmagn(0.0)
    , temp(0.0)
    , adc(0.0)  {
    }
  ADIS16405_(const ContainerAllocator& _alloc)
    : supply(0.0)
    , xgyro(0.0)
    , ygyro(0.0)
    , zgyro(0.0)
    , xaccl(0.0)
    , yaccl(0.0)
    , zaccl(0.0)
    , xmagn(0.0)
    , ymagn(0.0)
    , zmagn(0.0)
    , temp(0.0)
    , adc(0.0)  {
    }



   typedef float _supply_type;
  _supply_type supply;

   typedef float _xgyro_type;
  _xgyro_type xgyro;

   typedef float _ygyro_type;
  _ygyro_type ygyro;

   typedef float _zgyro_type;
  _zgyro_type zgyro;

   typedef float _xaccl_type;
  _xaccl_type xaccl;

   typedef float _yaccl_type;
  _yaccl_type yaccl;

   typedef float _zaccl_type;
  _zaccl_type zaccl;

   typedef float _xmagn_type;
  _xmagn_type xmagn;

   typedef float _ymagn_type;
  _ymagn_type ymagn;

   typedef float _zmagn_type;
  _zmagn_type zmagn;

   typedef float _temp_type;
  _temp_type temp;

   typedef float _adc_type;
  _adc_type adc;




  typedef boost::shared_ptr< ::aauship::ADIS16405_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::aauship::ADIS16405_<ContainerAllocator> const> ConstPtr;

}; // struct ADIS16405_

typedef ::aauship::ADIS16405_<std::allocator<void> > ADIS16405;

typedef boost::shared_ptr< ::aauship::ADIS16405 > ADIS16405Ptr;
typedef boost::shared_ptr< ::aauship::ADIS16405 const> ADIS16405ConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::aauship::ADIS16405_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::aauship::ADIS16405_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace aauship

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'sensor_msgs': ['/opt/ros/indigo/share/sensor_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/indigo/share/std_msgs/cmake/../msg'], 'geometry_msgs': ['/opt/ros/indigo/share/geometry_msgs/cmake/../msg'], 'aauship': ['/home/kenny/Documents/aauship-formation/code/hli/ros/src/aauship/msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::aauship::ADIS16405_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::aauship::ADIS16405_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aauship::ADIS16405_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::aauship::ADIS16405_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aauship::ADIS16405_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::aauship::ADIS16405_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::aauship::ADIS16405_<ContainerAllocator> >
{
  static const char* value()
  {
    return "7cf3439b3e98d50b8f75a089f6f143fa";
  }

  static const char* value(const ::aauship::ADIS16405_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x7cf3439b3e98d50bULL;
  static const uint64_t static_value2 = 0x8f75a089f6f143faULL;
};

template<class ContainerAllocator>
struct DataType< ::aauship::ADIS16405_<ContainerAllocator> >
{
  static const char* value()
  {
    return "aauship/ADIS16405";
  }

  static const char* value(const ::aauship::ADIS16405_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::aauship::ADIS16405_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# Format for the ADIS13205 IMU from the LLI decoded data\n\
float32 supply\n\
float32 xgyro\n\
float32 ygyro\n\
float32 zgyro\n\
float32 xaccl\n\
float32 yaccl\n\
float32 zaccl\n\
float32 xmagn\n\
float32 ymagn\n\
float32 zmagn\n\
float32 temp\n\
float32 adc\n\
";
  }

  static const char* value(const ::aauship::ADIS16405_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::aauship::ADIS16405_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.supply);
      stream.next(m.xgyro);
      stream.next(m.ygyro);
      stream.next(m.zgyro);
      stream.next(m.xaccl);
      stream.next(m.yaccl);
      stream.next(m.zaccl);
      stream.next(m.xmagn);
      stream.next(m.ymagn);
      stream.next(m.zmagn);
      stream.next(m.temp);
      stream.next(m.adc);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER;
  }; // struct ADIS16405_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::aauship::ADIS16405_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::aauship::ADIS16405_<ContainerAllocator>& v)
  {
    s << indent << "supply: ";
    Printer<float>::stream(s, indent + "  ", v.supply);
    s << indent << "xgyro: ";
    Printer<float>::stream(s, indent + "  ", v.xgyro);
    s << indent << "ygyro: ";
    Printer<float>::stream(s, indent + "  ", v.ygyro);
    s << indent << "zgyro: ";
    Printer<float>::stream(s, indent + "  ", v.zgyro);
    s << indent << "xaccl: ";
    Printer<float>::stream(s, indent + "  ", v.xaccl);
    s << indent << "yaccl: ";
    Printer<float>::stream(s, indent + "  ", v.yaccl);
    s << indent << "zaccl: ";
    Printer<float>::stream(s, indent + "  ", v.zaccl);
    s << indent << "xmagn: ";
    Printer<float>::stream(s, indent + "  ", v.xmagn);
    s << indent << "ymagn: ";
    Printer<float>::stream(s, indent + "  ", v.ymagn);
    s << indent << "zmagn: ";
    Printer<float>::stream(s, indent + "  ", v.zmagn);
    s << indent << "temp: ";
    Printer<float>::stream(s, indent + "  ", v.temp);
    s << indent << "adc: ";
    Printer<float>::stream(s, indent + "  ", v.adc);
  }
};

} // namespace message_operations
} // namespace ros

#endif // AAUSHIP_MESSAGE_ADIS16405_H
