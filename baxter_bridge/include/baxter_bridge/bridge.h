#ifndef BAXTER_BRIDGE_H
#define BAXTER_BRIDGE_H

#include <ros/node_handle.h>
#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>
#include <baxter_bridge/monitor.h>

namespace baxter_bridge
{

class Bridge
{
public:

  enum class Direction{ROS_1_TO_2, ROS_2_TO_1};

  static bool init(int argc, char** argv);

  inline static void spinOnce()
  {
    exec->spin_once();
  }

  inline static bool onBaxter() {return on_baxter;}
  inline static bool isStatic() {return is_static;}

  inline static ros::NodeHandle* ros1() {return ros1_node.get();}
  inline static rclcpp::Node* ros2() {return  ros2_node.get();}

  inline static rclcpp::Time ros2_now()
  {
    return ros2_node->get_clock()->now();
  }

  inline static ros::Time ros1_now()
  {
    return ros::Time::now();
  }

  virtual std::string topic() const = 0;
  virtual Direction direction() const = 0;

protected:

  static bool initRSP();
  inline static bool canPublishOn(const std::string &topic)
  {
    if(!monitor) return true;
    return monitor->canPublishOn(topic);
  }

private:
  static std::unique_ptr<ros::NodeHandle> ros1_node;
  static rclcpp::Node::SharedPtr ros2_node;
  static robot_state_publisher::RobotStatePublisher::SharedPtr rsp_node;
  static rclcpp::executors::SingleThreadedExecutor::SharedPtr exec;
  static bool on_baxter;
  static bool is_static;

  // monitor who publishes on which topics
  static std::unique_ptr<Monitor> monitor;

};

}

#endif // BAXTER_BRIDGE_H
