#ifndef BAXTER_BRIDGE_1TO2_H
#define BAXTER_BRIDGE_1TO2_H

#include <baxter_bridge/bridge.h>

namespace baxter_bridge
{

using Node2 = rclcpp::Node;

template <class Msg1>
auto convertROS2(const Msg1 &)
{
  return;
}

template<class Msg1, class Msg2>
struct Bridge_1to2 : public Bridge
{
  Bridge_1to2(std::string topic)
  {  
    pub = ros2()->create_publisher<Msg2>(topic, 10);
    sub = ros1()->subscribe<Msg1>(topic, 10, [&](const typename Msg1::ConstPtr &msg)
    {pub->publish(convertROS2(*msg));});
  }


private:
  // ros 1
  ros::Subscriber sub;

  // ros 2
  typename rclcpp::Publisher<Msg2>::SharedPtr pub;
};

}

#endif
