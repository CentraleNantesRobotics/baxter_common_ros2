#include <ros/node_handle.h>
#include <rclcpp/node.hpp>
#include <baxter_bridge/bridge.h>
#include <baxter_bridge/factory.h>

namespace baxter_bridge
{

std::unique_ptr<ros::NodeHandle> Bridge::ros1_node{};
rclcpp::Node::SharedPtr Bridge::ros2_node{};
bool Bridge::on_baxter{false}, Bridge::is_static{false};
rclcpp::executors::SingleThreadedExecutor::SharedPtr Bridge::exec;

std::unique_ptr<Monitor> Bridge::monitor;

void Bridge::init(int argc, char** argv)
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
  ros2_node = std::make_shared<rclcpp::Node>("baxter_ros2_bridge");
  exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  exec->add_node(ros2_node);

  // let some people configure Baxter before running the bridge
  const auto allow_multiple{ros1_node->param<bool>("allow_multiple", false)};

  if(on_baxter && !allow_multiple)
  {
    monitor = std::make_unique<Monitor>(name, baxter_display);
  }
}

}
