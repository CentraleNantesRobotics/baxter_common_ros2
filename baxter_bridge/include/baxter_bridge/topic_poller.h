#ifndef BAXTER_IO_H
#define BAXTER_IO_H

#include <map>
#include <string>
#include <ros/ros.h>
#include <rclcpp/node.hpp>

namespace baxter_bridge
{

using namespace std::chrono_literals;

class Bridge;

struct TopicPoller
{
  TopicPoller(rclcpp::Node* node2);

  std::vector<std::string> pendingBridges() const;

private:

  std::vector<std::string> ros1_published, ros1_subscribed, ros2_published, ros2_subscribed;

  ros::NodeHandle node1;
  rclcpp::Node* node2;
  rclcpp::TimerBase::SharedPtr poller;

  void poll();

};

}

#endif // BAXTER_IO_H
