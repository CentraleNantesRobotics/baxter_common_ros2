//Generated from baxter_io.yaml, edit is not recommended
#include <baxter_bridge/bridge_2to1.h>
#include <baxter_bridge/factory.h>
//messages
#include <baxter_core_msgs/DigitalOutputCommand.h>
#include <baxter_core_msgs/msg/digital_output_command.hpp>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/msg/end_effector_command.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/msg/bool.hpp>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/msg/head_pan_command.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/msg/empty.hpp>

namespace baxter_bridge
{
// converters
template<>
void convertMsg(const baxter_core_msgs::msg::DigitalOutputCommand &src, baxter_core_msgs::DigitalOutputCommand &dst)
{
  dst.name = src.name;
  dst.value = src.value;
}

template<>
void convertMsg(const baxter_core_msgs::msg::EndEffectorCommand &src, baxter_core_msgs::EndEffectorCommand &dst)
{
  dst.id = src.id;
  dst.command = src.command;
  dst.args = src.args;
  dst.sender = src.sender;
  dst.sequence = src.sequence;
}

template<>
void convertMsg(const std_msgs::msg::Bool &src, std_msgs::Bool &dst)
{
  dst.data = src.data;
}

template<>
void convertMsg(const baxter_core_msgs::msg::HeadPanCommand &src, baxter_core_msgs::HeadPanCommand &dst)
{
  dst.target = src.target;
  dst.speed_ratio = src.speed_ratio;
  dst.enable_pan_request = src.enable_pan_request;
}

template<>
void convertMsg(const std_msgs::msg::Empty &, std_msgs::Empty &)
{
}

std::map<std::string, std::string> Factory::topics_2to1 = {
  {"/robot/digital_io/command", "baxter_core_msgs/DigitalOutputCommand"},
  {"/robot/end_effector/left_gripper/command", "baxter_core_msgs/EndEffectorCommand"},
  {"/robot/end_effector/right_gripper/command", "baxter_core_msgs/EndEffectorCommand"},
  {"/robot/head/command_head_nod", "std_msgs/Bool"},
  {"/robot/head/command_head_pan", "baxter_core_msgs/HeadPanCommand"},
  {"/robot/limb/left/suppress_collision_avoidance", "std_msgs/Empty"},
  {"/robot/limb/left/suppress_contact_safety", "std_msgs/Empty"},
  {"/robot/limb/left/suppress_hand_overwrench_safety", "std_msgs/Empty"},
  {"/robot/limb/left/use_default_spring_model", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_collision_avoidance", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_contact_safety", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_hand_overwrench_safety", "std_msgs/Empty"},
  {"/robot/limb/right/use_default_spring_model", "std_msgs/Empty"},
  {"/robot/set_super_enable", "std_msgs/Bool"},
  {"/robot/set_super_reset", "std_msgs/Empty"}};

void Factory::createBridge_2to1(const std::string &topic, const std::string &msg)
{
  if(msg == "baxter_core_msgs/DigitalOutputCommand")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::DigitalOutputCommand, baxter_core_msgs::msg::DigitalOutputCommand>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/EndEffectorCommand")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::EndEffectorCommand, baxter_core_msgs::msg::EndEffectorCommand>>
        (topic));
  }
  else if(msg == "std_msgs/Bool")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::Bool, std_msgs::msg::Bool>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/HeadPanCommand")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::HeadPanCommand, baxter_core_msgs::msg::HeadPanCommand>>
        (topic));
  }
  else if(msg == "std_msgs/Empty")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::Empty, std_msgs::msg::Empty>>
        (topic));
  }
}
}