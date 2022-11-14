#include <ros/node_handle.h>
#include <rclcpp/node.hpp>
#include <baxter_bridge/bridge.h>
#include <baxter_bridge/factory.h>
#include <fstream>

namespace baxter_bridge
{

  std::unique_ptr<ros::NodeHandle> Bridge::ros1_node{};
  rclcpp::Node::SharedPtr Bridge::ros2_node{};
  bool Bridge::on_baxter{false}, Bridge::is_static{false};
  rclcpp::executors::SingleThreadedExecutor::SharedPtr Bridge::exec;
  robot_state_publisher::RobotStatePublisher::SharedPtr Bridge::rsp_node;

  std::unique_ptr<Monitor> Bridge::monitor;

  bool Bridge::init(int argc, char** argv)
  {
    // parse raw args to call the node easily from cmd line
    bool baxter_display{true};
    for(int idx = 0; idx < argc; ++idx)
    {
      const auto arg{std::string(argv[idx])};
      if(arg == "-b") on_baxter = true;
      else if(arg == "-d") baxter_display = false;
      else if(arg == "-s") is_static = true;
    }

    if(!on_baxter)
    {
      // identify if we are actually on baxter
      try
      {
        const std::string rosmaster{std::getenv("ROS_MASTER_URI")};
        on_baxter = rosmaster.find("baxter.local") != rosmaster.npos;
      }
      catch (...) {}
    }

    const std::string name{std::getenv("USER")};

    // ros 1
    if(on_baxter)
      ros::init(argc, argv, "baxter_ros1_bridge_" + name);
    else
      ros::init(argc, argv, "baxter_ros1_bridge");
    ros1_node = std::make_unique<ros::NodeHandle>();

    // ros 2
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions bridge_arg;
    bridge_arg.arguments({"--ros-args", "-r", "__ns:=/robot"});
    ros2_node = std::make_shared<rclcpp::Node>("baxter_ros2_bridge", bridge_arg);
    exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    exec->add_node(ros2_node);

    if(!initRSP())
      return false;

    // let some people configure Baxter before running the bridge
    const auto allow_multiple{ros1_node->param<bool>("allow_multiple", false)};

    if(on_baxter && !allow_multiple)
    {
      monitor = std::make_unique<Monitor>(name, baxter_display);
    }
    return true;
  }

  // init internal robot state publisher
  bool Bridge::initRSP()
  {
    // get Baxter's description from ROS 1 param
    const auto description{ros1_node->param<std::string>("robot_description", "")};

    if(description.empty())
    {
      RCLCPP_ERROR(ros2()->get_logger(), "Cannot find param /robot_description: run a simulation or connect to Baxter");
      return false;
    }

    urdf::Model model;
    model.initString(description);

    if(model.getName() == "baxter")
      RCLCPP_INFO(ros2()->get_logger(), "Using Baxter's robot description");
    else
    {
      const std::string msg{"Using description of robot '" + model.getName() + "'"};
      RCLCPP_WARN(ros2()->get_logger(), "%s", msg.c_str());
    }

    // instanciate local robot_state_publisher
    auto rsp_arg{rclcpp::NodeOptions()
          .arguments({"--ros-args", "-r", "__ns:=/robot", "-p", "robot_description:=" + description})};
    rsp_node = std::make_shared<robot_state_publisher::RobotStatePublisher>(rsp_arg);

    exec->add_node(rsp_node);

    return true;
  }

}
