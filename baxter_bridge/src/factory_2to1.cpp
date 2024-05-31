//Generated with gen_factory.py, edit is not recommended
#include <baxter_bridge/bridge_2to1.h>
#include <baxter_bridge/factory.h>
//messages
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
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

namespace baxter_bridge
{
// converters
template<>
void convert(const std_msgs::msg::Header &src, std_msgs::Header &dst)
{
  dst.stamp = Bridge::ros1_now();
  convert(src.frame_id, dst.frame_id);
}

template<>
void convert(const sensor_msgs::msg::JointState &src, sensor_msgs::JointState &dst)
{
  convert(src.header, dst.header);
  convert(src.name, dst.name);
  convert(src.position, dst.position);
  convert(src.velocity, dst.velocity);
  convert(src.effort, dst.effort);
}

template<>
void convert(const geometry_msgs::msg::Point &src, geometry_msgs::Point &dst)
{
  convert(src.x, dst.x);
  convert(src.y, dst.y);
  convert(src.z, dst.z);
}

template<>
void convert(const geometry_msgs::msg::Quaternion &src, geometry_msgs::Quaternion &dst)
{
  convert(src.x, dst.x);
  convert(src.y, dst.y);
  convert(src.z, dst.z);
  convert(src.w, dst.w);
}

template<>
void convert(const geometry_msgs::msg::Pose &src, geometry_msgs::Pose &dst)
{
  convert(src.position, dst.position);
  convert(src.orientation, dst.orientation);
}

template<>
void convert(const geometry_msgs::msg::PoseStamped &src, geometry_msgs::PoseStamped &dst)
{
  convert(src.header, dst.header);
  convert(src.pose, dst.pose);
}

template<>
void convert(const baxter_core_msgs::msg::AnalogOutputCommand &src, baxter_core_msgs::AnalogOutputCommand &dst)
{
  convert(src.name, dst.name);
  convert(src.value, dst.value);
}

template<>
void convert(const baxter_core_msgs::msg::DigitalOutputCommand &src, baxter_core_msgs::DigitalOutputCommand &dst)
{
  convert(src.name, dst.name);
  convert(src.value, dst.value);
}

template<>
void convert(const baxter_core_msgs::msg::EndEffectorCommand &src, baxter_core_msgs::EndEffectorCommand &dst)
{
  convert(src.id, dst.id);
  convert(src.command, dst.command);
  convert(src.args, dst.args);
  convert(src.sender, dst.sender);
  convert(src.sequence, dst.sequence);
}

template<>
void convert(const baxter_core_msgs::msg::EndEffectorProperties &src, baxter_core_msgs::EndEffectorProperties &dst)
{
  convert(src.id, dst.id);
  convert(src.ui_type, dst.ui_type);
  convert(src.manufacturer, dst.manufacturer);
  convert(src.product, dst.product);
  convert(src.serial_number, dst.serial_number);
  convert(src.hardware_rev, dst.hardware_rev);
  convert(src.firmware_rev, dst.firmware_rev);
  convert(src.firmware_date, dst.firmware_date);
  convert(src.has_calibration, dst.has_calibration);
  convert(src.controls_grip, dst.controls_grip);
  convert(src.senses_grip, dst.senses_grip);
  convert(src.reverses_grip, dst.reverses_grip);
  convert(src.controls_force, dst.controls_force);
  convert(src.senses_force, dst.senses_force);
  convert(src.controls_position, dst.controls_position);
  convert(src.senses_position, dst.senses_position);
  convert(src.properties, dst.properties);
}

template<>
void convert(const baxter_core_msgs::msg::EndEffectorState &src, baxter_core_msgs::EndEffectorState &dst)
{
  dst.timestamp = Bridge::ros1_now();
  convert(src.id, dst.id);
  convert(src.enabled, dst.enabled);
  convert(src.calibrated, dst.calibrated);
  convert(src.ready, dst.ready);
  convert(src.moving, dst.moving);
  convert(src.gripping, dst.gripping);
  convert(src.missed, dst.missed);
  convert(src.error, dst.error);
  convert(src.reverse, dst.reverse);
  convert(src.state, dst.state);
  convert(src.command, dst.command);
  convert(src.command_sender, dst.command_sender);
  convert(src.command_sequence, dst.command_sequence);
}

template<>
void convert(const std_msgs::msg::Bool &src, std_msgs::Bool &dst)
{
  convert(src.data, dst.data);
}

template<>
void convert(const baxter_core_msgs::msg::HeadPanCommand &src, baxter_core_msgs::HeadPanCommand &dst)
{
  convert(src.target, dst.target);
  convert(src.speed_ratio, dst.speed_ratio);
  convert(src.enable_pan_request, dst.enable_pan_request);
}

template<>
void convert(const std_msgs::msg::UInt16 &src, std_msgs::UInt16 &dst)
{
  convert(src.data, dst.data);
}

template<>
void convert(const std_msgs::msg::UInt32 &src, std_msgs::UInt32 &dst)
{
  convert(src.data, dst.data);
}

template<>
void convert(const geometry_msgs::msg::Vector3 &src, geometry_msgs::Vector3 &dst)
{
  convert(src.x, dst.x);
  convert(src.y, dst.y);
  convert(src.z, dst.z);
}

template<>
void convert(const geometry_msgs::msg::Twist &src, geometry_msgs::Twist &dst)
{
  convert(src.linear, dst.linear);
  convert(src.angular, dst.angular);
}

template<>
void convert(const geometry_msgs::msg::TwistStamped &src, geometry_msgs::TwistStamped &dst)
{
  convert(src.header, dst.header);
  convert(src.twist, dst.twist);
}

template<>
void convert(const trajectory_msgs::msg::JointTrajectoryPoint &src, trajectory_msgs::JointTrajectoryPoint &dst)
{
  convert(src.positions, dst.positions);
  convert(src.velocities, dst.velocities);
  convert(src.accelerations, dst.accelerations);
  convert(src.effort, dst.effort);
  dst.time_from_start.sec = src.time_from_start.sec;
  dst.time_from_start.nsec = src.time_from_start.nanosec;
}

template<>
void convert(const baxter_core_msgs::msg::JointCommand &src, baxter_core_msgs::JointCommand &dst)
{
#ifdef BAXTER_BRIDGE_SAFE_CMD
  convert(std::min(src.mode,2), dst.mode);
#else
  convert(src.mode, dst.mode);
#endif
  convert(src.command, dst.command);
  convert(src.names, dst.names);
}

template<>
void convert(const std_msgs::msg::Float64 &src, std_msgs::Float64 &dst)
{
  convert(src.data, dst.data);
}

template<>
void convert(const std_msgs::msg::Empty &, std_msgs::Empty &)
{
}

template<>
void convert(const std_msgs::msg::Float32 &src, std_msgs::Float32 &dst)
{
  convert(src.data, dst.data);
}

template<>
void convert(const sensor_msgs::msg::Image &src, sensor_msgs::Image &dst)
{
  convert(src.header, dst.header);
  convert(src.height, dst.height);
  convert(src.width, dst.width);
  convert(src.encoding, dst.encoding);
  convert(src.is_bigendian, dst.is_bigendian);
  convert(src.step, dst.step);
  convert(src.data, dst.data);
}

template<>
void convert(const baxter_maintenance_msgs::msg::CalibrateArmData &src, baxter_maintenance_msgs::CalibrateArmData &dst)
{
  convert(src.suppress_write_to_file, dst.suppressWriteToFile);
}

template<>
void convert(const baxter_maintenance_msgs::msg::CalibrateArmEnable &src, baxter_maintenance_msgs::CalibrateArmEnable &dst)
{
  convert(src.is_enabled, dst.isEnabled);
  convert(src.uid, dst.uid);
  convert(src.data, dst.data);
}

template<>
void convert(const baxter_maintenance_msgs::msg::TareData &src, baxter_maintenance_msgs::TareData &dst)
{
  convert(src.tune_gravity_spring, dst.tuneGravitySpring);
}

template<>
void convert(const baxter_maintenance_msgs::msg::TareEnable &src, baxter_maintenance_msgs::TareEnable &dst)
{
  convert(src.is_enabled, dst.isEnabled);
  convert(src.uid, dst.uid);
  convert(src.data, dst.data);
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
  {"/robustcontroller/right/Tare/enable", "baxter_maintenance_msgs/TareEnable"}};

void Factory::createBridge_2to1(const std::string &topic, const std::string &msg)
{
  if(msg == "sensor_msgs/JointState")
    bridges.push_back(std::make_unique<Bridge_2to1<sensor_msgs::JointState, sensor_msgs::msg::JointState>>(topic));
  else if(msg == "geometry_msgs/PoseStamped")
    bridges.push_back(std::make_unique<Bridge_2to1<geometry_msgs::PoseStamped, geometry_msgs::msg::PoseStamped>>(topic));
  else if(msg == "baxter_core_msgs/AnalogOutputCommand")
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::AnalogOutputCommand, baxter_core_msgs::msg::AnalogOutputCommand>>(topic));
  else if(msg == "baxter_core_msgs/DigitalOutputCommand")
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::DigitalOutputCommand, baxter_core_msgs::msg::DigitalOutputCommand>>(topic));
  else if(msg == "baxter_core_msgs/EndEffectorCommand")
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::EndEffectorCommand, baxter_core_msgs::msg::EndEffectorCommand>>(topic));
  else if(msg == "baxter_core_msgs/EndEffectorProperties")
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::EndEffectorProperties, baxter_core_msgs::msg::EndEffectorProperties>>(topic));
  else if(msg == "baxter_core_msgs/EndEffectorState")
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::EndEffectorState, baxter_core_msgs::msg::EndEffectorState>>(topic));
  else if(msg == "std_msgs/Bool")
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::Bool, std_msgs::msg::Bool>>(topic));
  else if(msg == "baxter_core_msgs/HeadPanCommand")
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::HeadPanCommand, baxter_core_msgs::msg::HeadPanCommand>>(topic));
  else if(msg == "std_msgs/UInt16")
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::UInt16, std_msgs::msg::UInt16>>(topic));
  else if(msg == "std_msgs/UInt32")
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::UInt32, std_msgs::msg::UInt32>>(topic));
  else if(msg == "geometry_msgs/TwistStamped")
    bridges.push_back(std::make_unique<Bridge_2to1<geometry_msgs::TwistStamped, geometry_msgs::msg::TwistStamped>>(topic));
  else if(msg == "trajectory_msgs/JointTrajectoryPoint")
    bridges.push_back(std::make_unique<Bridge_2to1<trajectory_msgs::JointTrajectoryPoint, trajectory_msgs::msg::JointTrajectoryPoint>>(topic));
  else if(msg == "baxter_core_msgs/JointCommand")
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::JointCommand, baxter_core_msgs::msg::JointCommand>>(topic));
  else if(msg == "std_msgs/Float64")
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::Float64, std_msgs::msg::Float64>>(topic));
  else if(msg == "std_msgs/Empty")
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::Empty, std_msgs::msg::Empty>>(topic));
  else if(msg == "std_msgs/Float32")
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::Float32, std_msgs::msg::Float32>>(topic));
  else if(msg == "sensor_msgs/Image")
    bridges.push_back(std::make_unique<Bridge_2to1<sensor_msgs::Image, sensor_msgs::msg::Image>>(topic));
  else if(msg == "baxter_maintenance_msgs/CalibrateArmEnable")
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_maintenance_msgs::CalibrateArmEnable, baxter_maintenance_msgs::msg::CalibrateArmEnable>>(topic));
  else if(msg == "baxter_maintenance_msgs/TareEnable")
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_maintenance_msgs::TareEnable, baxter_maintenance_msgs::msg::TareEnable>>(topic));
}
}