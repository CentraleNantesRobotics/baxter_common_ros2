//Generated with gen_factory.py, edit is not recommended
#include <baxter_bridge/bridge_2to1.h>
#include <baxter_bridge/factory.h>
//messages
#include <baxter_core_msgs/AnalogOutputCommand.h>
#include <baxter_core_msgs/msg/analog_output_command.hpp>
#include <baxter_core_msgs/DigitalOutputCommand.h>
#include <baxter_core_msgs/msg/digital_output_command.hpp>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/msg/end_effector_command.hpp>
#include <baxter_core_msgs/EndEffectorProperties.h>
#include <baxter_core_msgs/msg/end_effector_properties.hpp>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/msg/end_effector_state.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/msg/bool.hpp>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/msg/head_pan_command.hpp>
#include <std_msgs/UInt16.h>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/UInt32.h>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <baxter_core_msgs/JointCommand.h>
#include <baxter_core_msgs/msg/joint_command.hpp>
#include <std_msgs/Float64.h>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/msg/float32.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/msg/image.hpp>
#include <baxter_maintenance_msgs/CalibrateArmEnable.h>
#include <baxter_maintenance_msgs/msg/calibrate_arm_enable.hpp>
#include <baxter_maintenance_msgs/TareEnable.h>
#include <baxter_maintenance_msgs/msg/tare_enable.hpp>
#include <actionlib_msgs/GoalID.h>
#include <actionlib_msgs/msg/goal_id.hpp>

namespace baxter_bridge
{
// converters
template<>
void convertMsg(const baxter_core_msgs::msg::AnalogOutputCommand &src, baxter_core_msgs::AnalogOutputCommand &dst)
{
  dst.name = src.name;
  dst.value = src.value;
}

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
void convertMsg(const baxter_core_msgs::msg::EndEffectorProperties &src, baxter_core_msgs::EndEffectorProperties &dst)
{
  dst.id = src.id;
  dst.ui_type = src.ui_type;
  dst.manufacturer = src.manufacturer;
  dst.product = src.product;
  dst.serial_number = src.serial_number;
  dst.hardware_rev = src.hardware_rev;
  dst.firmware_rev = src.firmware_rev;
  dst.firmware_date = src.firmware_date;
  dst.has_calibration = src.has_calibration;
  dst.controls_grip = src.controls_grip;
  dst.senses_grip = src.senses_grip;
  dst.reverses_grip = src.reverses_grip;
  dst.controls_force = src.controls_force;
  dst.senses_force = src.senses_force;
  dst.controls_position = src.controls_position;
  dst.senses_position = src.senses_position;
  dst.properties = src.properties;
}

template<>
void convertMsg(const baxter_core_msgs::msg::EndEffectorState &src, baxter_core_msgs::EndEffectorState &dst)
{
  dst.timestamp = Bridge::ros1_now();
  dst.id = src.id;
  dst.enabled = src.enabled;
  dst.calibrated = src.calibrated;
  dst.ready = src.ready;
  dst.moving = src.moving;
  dst.gripping = src.gripping;
  dst.missed = src.missed;
  dst.error = src.error;
  dst.reverse = src.reverse;
  dst.state = src.state;
  dst.command = src.command;
  dst.command_sender = src.command_sender;
  dst.command_sequence = src.command_sequence;
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
void convertMsg(const std_msgs::msg::UInt16 &src, std_msgs::UInt16 &dst)
{
  dst.data = src.data;
}

template<>
void convertMsg(const std_msgs::msg::UInt32 &src, std_msgs::UInt32 &dst)
{
  dst.data = src.data;
}

template<>
void convertMsg(const std_msgs::msg::Header &src, std_msgs::Header &dst)
{
  dst.stamp = Bridge::ros1_now();
  dst.frame_id = src.frame_id;
}

template<>
void convertMsg(const geometry_msgs::msg::Vector3 &src, geometry_msgs::Vector3 &dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}

template<>
void convertMsg(const geometry_msgs::msg::Twist &src, geometry_msgs::Twist &dst)
{
  convertMsg(src.linear, dst.linear);
  convertMsg(src.angular, dst.angular);
}

template<>
void convertMsg(const geometry_msgs::msg::TwistStamped &src, geometry_msgs::TwistStamped &dst)
{
  convertMsg(src.header, dst.header);
  convertMsg(src.twist, dst.twist);
}

template<>
void convertMsg(const trajectory_msgs::msg::JointTrajectoryPoint &src, trajectory_msgs::JointTrajectoryPoint &dst)
{
  dst.positions = src.positions;
  dst.velocities = src.velocities;
  dst.accelerations = src.accelerations;
  dst.effort = src.effort;
  dst.time_from_start.sec = src.time_from_start.sec;
  dst.time_from_start.nsec = src.time_from_start.nanosec;
}

template<>
void convertMsg(const baxter_core_msgs::msg::JointCommand &src, baxter_core_msgs::JointCommand &dst)
{
  dst.mode = src.mode;
  dst.command = src.command;
  dst.names = src.names;
}

template<>
void convertMsg(const std_msgs::msg::Float64 &src, std_msgs::Float64 &dst)
{
  dst.data = src.data;
}

template<>
void convertMsg(const std_msgs::msg::Empty &, std_msgs::Empty &)
{
}

template<>
void convertMsg(const std_msgs::msg::Float32 &src, std_msgs::Float32 &dst)
{
  dst.data = src.data;
}

template<>
void convertMsg(const sensor_msgs::msg::Image &src, sensor_msgs::Image &dst)
{
  convertMsg(src.header, dst.header);
  dst.height = src.height;
  dst.width = src.width;
  dst.encoding = src.encoding;
  dst.is_bigendian = src.is_bigendian;
  dst.step = src.step;
  dst.data = src.data;
}

template<>
void convertMsg(const baxter_maintenance_msgs::msg::CalibrateArmData &src, baxter_maintenance_msgs::CalibrateArmData &dst)
{
  dst.suppressWriteToFile = src.suppress_write_to_file;
}

template<>
void convertMsg(const baxter_maintenance_msgs::msg::CalibrateArmEnable &src, baxter_maintenance_msgs::CalibrateArmEnable &dst)
{
  dst.isEnabled = src.is_enabled;
  dst.uid = src.uid;
  convertMsg(src.data, dst.data);
}

template<>
void convertMsg(const baxter_maintenance_msgs::msg::TareData &src, baxter_maintenance_msgs::TareData &dst)
{
  dst.tuneGravitySpring = src.tune_gravity_spring;
}

template<>
void convertMsg(const baxter_maintenance_msgs::msg::TareEnable &src, baxter_maintenance_msgs::TareEnable &dst)
{
  dst.isEnabled = src.is_enabled;
  dst.uid = src.uid;
  convertMsg(src.data, dst.data);
}

template<>
void convertMsg(const actionlib_msgs::msg::GoalID &src, actionlib_msgs::GoalID &dst)
{
  dst.stamp = Bridge::ros1_now();
  dst.id = src.id;
}

std::map<std::string, std::string> Factory::topics_2to1 = {
  {"/robot/analog_io/command", "baxter_core_msgs/AnalogOutputCommand"},
  {"/robot/digital_io/command", "baxter_core_msgs/DigitalOutputCommand"},
  {"/robot/end_effector/left_gripper/command", "baxter_core_msgs/EndEffectorCommand"},
  {"/robot/end_effector/left_gripper/rsdk/set_properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/left_gripper/rsdk/set_state", "baxter_core_msgs/EndEffectorState"},
  {"/robot/end_effector/right_gripper/command", "baxter_core_msgs/EndEffectorCommand"},
  {"/robot/end_effector/right_gripper/rsdk/set_properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/right_gripper/rsdk/set_state", "baxter_core_msgs/EndEffectorState"},
  {"/robot/head/command_head_nod", "std_msgs/Bool"},
  {"/robot/head/command_head_pan", "baxter_core_msgs/HeadPanCommand"},
  {"/robot/joint_state_publish_rate", "std_msgs/UInt16"},
  {"/robot/limb/left/command_stiffness", "std_msgs/UInt32"},
  {"/robot/limb/left/command_twist_stamped", "geometry_msgs/TwistStamped"},
  {"/robot/limb/left/command_velocity_tozero", "std_msgs/Bool"},
  {"/robot/limb/left/inverse_dynamics_command", "trajectory_msgs/JointTrajectoryPoint"},
  {"/robot/limb/left/joint_command", "baxter_core_msgs/JointCommand"},
  {"/robot/limb/left/joint_command_timeout", "std_msgs/Float64"},
  {"/robot/limb/left/set_dominance", "std_msgs/Bool"},
  {"/robot/limb/left/set_speed_ratio", "std_msgs/Float64"},
  {"/robot/limb/left/suppress_collision_avoidance", "std_msgs/Empty"},
  {"/robot/limb/left/suppress_contact_safety", "std_msgs/Empty"},
  {"/robot/limb/left/suppress_cuff_interaction", "std_msgs/Empty"},
  {"/robot/limb/left/suppress_gravity_compensation", "std_msgs/Empty"},
  {"/robot/limb/left/suppress_hand_overwrench_safety", "std_msgs/Empty"},
  {"/robot/limb/left/use_default_spring_model", "std_msgs/Empty"},
  {"/robot/limb/right/command_stiffness", "std_msgs/UInt32"},
  {"/robot/limb/right/command_twist_stamped", "geometry_msgs/TwistStamped"},
  {"/robot/limb/right/command_velocity_tozero", "std_msgs/Bool"},
  {"/robot/limb/right/inverse_dynamics_command", "trajectory_msgs/JointTrajectoryPoint"},
  {"/robot/limb/right/joint_command", "baxter_core_msgs/JointCommand"},
  {"/robot/limb/right/joint_command_timeout", "std_msgs/Float64"},
  {"/robot/limb/right/set_dominance", "std_msgs/Bool"},
  {"/robot/limb/right/set_speed_ratio", "std_msgs/Float64"},
  {"/robot/limb/right/suppress_collision_avoidance", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_contact_safety", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_cuff_interaction", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_gravity_compensation", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_hand_overwrench_safety", "std_msgs/Empty"},
  {"/robot/limb/right/use_default_spring_model", "std_msgs/Empty"},
  {"/robot/set_motor_voltage_low", "std_msgs/Bool"},
  {"/robot/set_super_enable", "std_msgs/Bool"},
  {"/robot/set_super_reset", "std_msgs/Empty"},
  {"/robot/set_super_stop", "std_msgs/Empty"},
  {"/robot/sonar/head_sonar/lights/set_green_level", "std_msgs/Float32"},
  {"/robot/sonar/head_sonar/lights/set_lights", "std_msgs/UInt16"},
  {"/robot/sonar/head_sonar/lights/set_red_level", "std_msgs/Float32"},
  {"/robot/sonar/head_sonar/set_sonars_enabled", "std_msgs/UInt16"},
  {"/robot/xdisplay", "sensor_msgs/Image"},
  {"/robustcontroller/left/CalibrateArm/enable", "baxter_maintenance_msgs/CalibrateArmEnable"},
  {"/robustcontroller/left/Tare/enable", "baxter_maintenance_msgs/TareEnable"},
  {"/robustcontroller/right/CalibrateArm/enable", "baxter_maintenance_msgs/CalibrateArmEnable"},
  {"/robustcontroller/right/Tare/enable", "baxter_maintenance_msgs/TareEnable"},
  {"/tf2_web_republisher/cancel", "actionlib_msgs/GoalID"}};

void Factory::createBridge_2to1(const std::string &topic, const std::string &msg)
{
  if(msg == "baxter_core_msgs/AnalogOutputCommand")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::AnalogOutputCommand, baxter_core_msgs::msg::AnalogOutputCommand>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/DigitalOutputCommand")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::DigitalOutputCommand, baxter_core_msgs::msg::DigitalOutputCommand>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/EndEffectorCommand")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::EndEffectorCommand, baxter_core_msgs::msg::EndEffectorCommand>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/EndEffectorProperties")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::EndEffectorProperties, baxter_core_msgs::msg::EndEffectorProperties>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/EndEffectorState")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::EndEffectorState, baxter_core_msgs::msg::EndEffectorState>>
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
  else if(msg == "std_msgs/UInt16")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::UInt16, std_msgs::msg::UInt16>>
        (topic));
  }
  else if(msg == "std_msgs/UInt32")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::UInt32, std_msgs::msg::UInt32>>
        (topic));
  }
  else if(msg == "geometry_msgs/TwistStamped")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<geometry_msgs::TwistStamped, geometry_msgs::msg::TwistStamped>>
        (topic));
  }
  else if(msg == "trajectory_msgs/JointTrajectoryPoint")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<trajectory_msgs::JointTrajectoryPoint, trajectory_msgs::msg::JointTrajectoryPoint>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/JointCommand")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::JointCommand, baxter_core_msgs::msg::JointCommand>>
        (topic));
  }
  else if(msg == "std_msgs/Float64")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::Float64, std_msgs::msg::Float64>>
        (topic));
  }
  else if(msg == "std_msgs/Empty")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::Empty, std_msgs::msg::Empty>>
        (topic));
  }
  else if(msg == "std_msgs/Float32")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::Float32, std_msgs::msg::Float32>>
        (topic));
  }
  else if(msg == "sensor_msgs/Image")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<sensor_msgs::Image, sensor_msgs::msg::Image>>
        (topic));
  }
  else if(msg == "baxter_maintenance_msgs/CalibrateArmEnable")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_maintenance_msgs::CalibrateArmEnable, baxter_maintenance_msgs::msg::CalibrateArmEnable>>
        (topic));
  }
  else if(msg == "baxter_maintenance_msgs/TareEnable")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_maintenance_msgs::TareEnable, baxter_maintenance_msgs::msg::TareEnable>>
        (topic));
  }
  else if(msg == "actionlib_msgs/GoalID")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<actionlib_msgs::GoalID, actionlib_msgs::msg::GoalID>>
        (topic));
  }
}
}