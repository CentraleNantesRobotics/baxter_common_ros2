// Generated by gencpp from file baxter_core_msgs/BridgePublishersAuthRequest.msg
// DO NOT EDIT!


#ifndef BAXTER_CORE_MSGS_MESSAGE_BRIDGEPUBLISHERSAUTHREQUEST_H
#define BAXTER_CORE_MSGS_MESSAGE_BRIDGEPUBLISHERSAUTHREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace baxter_core_msgs
{
template <class ContainerAllocator>
struct BridgePublishersAuthRequest_
{
  typedef BridgePublishersAuthRequest_<ContainerAllocator> Type;

  BridgePublishersAuthRequest_()
    : topic()
    , user()  {
    }
  BridgePublishersAuthRequest_(const ContainerAllocator& _alloc)
    : topic(_alloc)
    , user(_alloc)  {
  (void)_alloc;
    }



   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _topic_type;
  _topic_type topic;

   typedef std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> _user_type;
  _user_type user;





  typedef boost::shared_ptr< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> const> ConstPtr;

}; // struct BridgePublishersAuthRequest_

typedef ::baxter_core_msgs::BridgePublishersAuthRequest_<std::allocator<void> > BridgePublishersAuthRequest;

typedef boost::shared_ptr< ::baxter_core_msgs::BridgePublishersAuthRequest > BridgePublishersAuthRequestPtr;
typedef boost::shared_ptr< ::baxter_core_msgs::BridgePublishersAuthRequest const> BridgePublishersAuthRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator1> & lhs, const ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator2> & rhs)
{
  return lhs.topic == rhs.topic &&
    lhs.user == rhs.user;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator1> & lhs, const ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace baxter_core_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "b16cb0c81cf9122d3d29f22bf77acc6e";
  }

  static const char* value(const ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xb16cb0c81cf9122dULL;
  static const uint64_t static_value2 = 0x3d29f22bf77acc6eULL;
};

template<class ContainerAllocator>
struct DataType< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "baxter_core_msgs/BridgePublishersAuthRequest";
  }

  static const char* value(const ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "string topic\n"
"string user\n"
;
  }

  static const char* value(const ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.topic);
      stream.next(m.user);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct BridgePublishersAuthRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::baxter_core_msgs::BridgePublishersAuthRequest_<ContainerAllocator>& v)
  {
    s << indent << "topic: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.topic);
    s << indent << "user: ";
    Printer<std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>>::stream(s, indent + "  ", v.user);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BAXTER_CORE_MSGS_MESSAGE_BRIDGEPUBLISHERSAUTHREQUEST_H
