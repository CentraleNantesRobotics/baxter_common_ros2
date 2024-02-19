#ifndef BAXTER_BRIDGE_H
#define BAXTER_BRIDGE_H

#include <ros/node_handle.h>
#include <rclcpp/node.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <baxter_bridge/monitor.h>

namespace baxter_bridge
{

class Bridge
{
public:

  enum class Direction{ROS_1_TO_2, ROS_2_TO_1};

  static std::tuple<bool, bool, bool> init(int argc, char** argv);

  inline static void spinOnce()
  {
    exec->spin_once();
  }

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

private:
  static inline std::unique_ptr<ros::NodeHandle> ros1_node;
  static inline rclcpp::Node::SharedPtr ros2_node;
  static inline rclcpp::executors::SingleThreadedExecutor::SharedPtr exec;
  static inline bool is_static{false};

};

class Monitored
{
private:
  // monitor who publishes on which topics
  static inline std::unique_ptr<Monitor> monitor;
protected:
  inline static bool canPublishOn(const std::string &topic)
  {
    if(!monitor) return true;
    return monitor->canPublishOn(topic);
  }
public:
  inline static void init(const std::string &name, ros::NodeHandle* ros1, bool run_server)
  {
    monitor = std::make_unique<Monitor>(name, ros1, run_server);
  }
};

}

#endif // BAXTER_BRIDGE_H
