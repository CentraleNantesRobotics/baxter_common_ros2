//Generated from baxter_io.yaml, edit is not recommended
#include <baxter_bridge/bridge_2to1.h>
#include <baxter_bridge/factory.h>
//messages
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
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
#include <baxter_core_msgs/HeadState.h>
#include <baxter_core_msgs/msg/head_state.hpp>
#include <std_msgs/UInt16.h>
#include <std_msgs/msg/u_int16.hpp>
#include <baxter_core_msgs/CollisionAvoidanceState.h>
#include <baxter_core_msgs/msg/collision_avoidance_state.hpp>
#include <std_msgs/UInt32.h>
#include <std_msgs/msg/u_int32.hpp>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/msg/endpoint_state.hpp>
#include <baxter_core_msgs/SEAJointState.h>
#include <baxter_core_msgs/msg/sea_joint_state.hpp>
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
#include <baxter_core_msgs/URDFConfiguration.h>
#include <baxter_core_msgs/msg/urdf_configuration.hpp>
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
auto convertROS1(const std_msgs::msg::Header &msg2)
{
  std_msgs::Header msg1;
  msg1.stamp = Bridge::ros1_now();
  msg1.frame_id = msg2.frame_id;
  return msg1;
}

template<>
auto convertROS1(const diagnostic_msgs::msg::KeyValue &msg2)
{
  diagnostic_msgs::KeyValue msg1;
  msg1.key = msg2.key;
  msg1.value = msg2.value;
  return msg1;
}

template<>
auto convertROS1(const diagnostic_msgs::msg::DiagnosticStatus &msg2)
{
  diagnostic_msgs::DiagnosticStatus msg1;
  msg1.level = msg2.level;
  msg1.name = msg2.name;
  msg1.message = msg2.message;
  msg1.hardware_id = msg2.hardware_id;
  std::transform(msg2.values.begin(), msg2.values.end(),
                 std::back_inserter(msg1.values),
                 convertROS1<diagnostic_msgs::msg::KeyValue>);
  return msg1;
}

template<>
auto convertROS1(const diagnostic_msgs::msg::DiagnosticArray &msg2)
{
  diagnostic_msgs::DiagnosticArray msg1;
  msg1.header = convertROS1(msg2.header);
  std::transform(msg2.status.begin(), msg2.status.end(),
                 std::back_inserter(msg1.status),
                 convertROS1<diagnostic_msgs::msg::DiagnosticStatus>);
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::AnalogOutputCommand &msg2)
{
  baxter_core_msgs::AnalogOutputCommand msg1;
  msg1.name = msg2.name;
  msg1.value = msg2.value;
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::DigitalOutputCommand &msg2)
{
  baxter_core_msgs::DigitalOutputCommand msg1;
  msg1.name = msg2.name;
  msg1.value = msg2.value;
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::EndEffectorCommand &msg2)
{
  baxter_core_msgs::EndEffectorCommand msg1;
  msg1.id = msg2.id;
  msg1.command = msg2.command;
  msg1.args = msg2.args;
  msg1.sender = msg2.sender;
  msg1.sequence = msg2.sequence;
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::EndEffectorProperties &msg2)
{
  baxter_core_msgs::EndEffectorProperties msg1;
  msg1.id = msg2.id;
  msg1.ui_type = msg2.ui_type;
  msg1.manufacturer = msg2.manufacturer;
  msg1.product = msg2.product;
  msg1.serial_number = msg2.serial_number;
  msg1.hardware_rev = msg2.hardware_rev;
  msg1.firmware_rev = msg2.firmware_rev;
  msg1.firmware_date = msg2.firmware_date;
  msg1.has_calibration = msg2.has_calibration;
  msg1.controls_grip = msg2.controls_grip;
  msg1.senses_grip = msg2.senses_grip;
  msg1.reverses_grip = msg2.reverses_grip;
  msg1.controls_force = msg2.controls_force;
  msg1.senses_force = msg2.senses_force;
  msg1.controls_position = msg2.controls_position;
  msg1.senses_position = msg2.senses_position;
  msg1.properties = msg2.properties;
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::EndEffectorState &msg2)
{
  baxter_core_msgs::EndEffectorState msg1;
  msg1.timestamp = Bridge::ros1_now();
  msg1.id = msg2.id;
  msg1.enabled = msg2.enabled;
  msg1.calibrated = msg2.calibrated;
  msg1.ready = msg2.ready;
  msg1.moving = msg2.moving;
  msg1.gripping = msg2.gripping;
  msg1.missed = msg2.missed;
  msg1.error = msg2.error;
  msg1.reverse = msg2.reverse;
  msg1.state = msg2.state;
  msg1.command = msg2.command;
  msg1.command_sender = msg2.command_sender;
  msg1.command_sequence = msg2.command_sequence;
  return msg1;
}

template<>
auto convertROS1(const std_msgs::msg::Bool &msg2)
{
  std_msgs::Bool msg1;
  msg1.data = msg2.data;
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::HeadPanCommand &msg2)
{
  baxter_core_msgs::HeadPanCommand msg1;
  msg1.target = msg2.target;
  msg1.speed_ratio = msg2.speed_ratio;
  msg1.enable_pan_request = msg2.enable_pan_request;
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::HeadState &msg2)
{
  baxter_core_msgs::HeadState msg1;
  msg1.pan = msg2.pan;
  msg1.isTurning = msg2.is_turning;
  msg1.isNodding = msg2.is_nodding;
  msg1.isPanEnabled = msg2.is_pan_enabled;
  return msg1;
}

template<>
auto convertROS1(const std_msgs::msg::UInt16 &msg2)
{
  std_msgs::UInt16 msg1;
  msg1.data = msg2.data;
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::CollisionAvoidanceState &msg2)
{
  baxter_core_msgs::CollisionAvoidanceState msg1;
  msg1.header = convertROS1(msg2.header);
  msg1.other_arm = msg2.other_arm;
  msg1.collision_object = msg2.collision_object;
  return msg1;
}

template<>
auto convertROS1(const std_msgs::msg::UInt32 &msg2)
{
  std_msgs::UInt32 msg1;
  msg1.data = msg2.data;
  return msg1;
}

template<>
auto convertROS1(const geometry_msgs::msg::Vector3 &msg2)
{
  geometry_msgs::Vector3 msg1;
  msg1.x = msg2.x;
  msg1.y = msg2.y;
  msg1.z = msg2.z;
  return msg1;
}

template<>
auto convertROS1(const geometry_msgs::msg::Twist &msg2)
{
  geometry_msgs::Twist msg1;
  msg1.linear = convertROS1(msg2.linear);
  msg1.angular = convertROS1(msg2.angular);
  return msg1;
}

template<>
auto convertROS1(const geometry_msgs::msg::TwistStamped &msg2)
{
  geometry_msgs::TwistStamped msg1;
  msg1.header = convertROS1(msg2.header);
  msg1.twist = convertROS1(msg2.twist);
  return msg1;
}

template<>
auto convertROS1(const geometry_msgs::msg::Point &msg2)
{
  geometry_msgs::Point msg1;
  msg1.x = msg2.x;
  msg1.y = msg2.y;
  msg1.z = msg2.z;
  return msg1;
}

template<>
auto convertROS1(const geometry_msgs::msg::Quaternion &msg2)
{
  geometry_msgs::Quaternion msg1;
  msg1.x = msg2.x;
  msg1.y = msg2.y;
  msg1.z = msg2.z;
  msg1.w = msg2.w;
  return msg1;
}

template<>
auto convertROS1(const geometry_msgs::msg::Pose &msg2)
{
  geometry_msgs::Pose msg1;
  msg1.position = convertROS1(msg2.position);
  msg1.orientation = convertROS1(msg2.orientation);
  return msg1;
}

template<>
auto convertROS1(const geometry_msgs::msg::Wrench &msg2)
{
  geometry_msgs::Wrench msg1;
  msg1.force = convertROS1(msg2.force);
  msg1.torque = convertROS1(msg2.torque);
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::EndpointState &msg2)
{
  baxter_core_msgs::EndpointState msg1;
  msg1.header = convertROS1(msg2.header);
  msg1.pose = convertROS1(msg2.pose);
  msg1.twist = convertROS1(msg2.twist);
  msg1.wrench = convertROS1(msg2.wrench);
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::SEAJointState &msg2)
{
  baxter_core_msgs::SEAJointState msg1;
  msg1.header = convertROS1(msg2.header);
  msg1.name = msg2.name;
  msg1.commanded_position = msg2.commanded_position;
  msg1.commanded_velocity = msg2.commanded_velocity;
  msg1.commanded_acceleration = msg2.commanded_acceleration;
  msg1.commanded_effort = msg2.commanded_effort;
  msg1.actual_position = msg2.actual_position;
  msg1.actual_velocity = msg2.actual_velocity;
  msg1.actual_effort = msg2.actual_effort;
  msg1.gravity_model_effort = msg2.gravity_model_effort;
  msg1.gravity_only = msg2.gravity_only;
  msg1.hysteresis_model_effort = msg2.hysteresis_model_effort;
  msg1.crosstalk_model_effort = msg2.crosstalk_model_effort;
  msg1.hystState = msg2.hyst_state;
  return msg1;
}

template<>
auto convertROS1(const trajectory_msgs::msg::JointTrajectoryPoint &msg2)
{
  trajectory_msgs::JointTrajectoryPoint msg1;
  msg1.positions = msg2.positions;
  msg1.velocities = msg2.velocities;
  msg1.accelerations = msg2.accelerations;
  msg1.effort = msg2.effort;
  msg1.time_from_start.sec = msg2.time_from_start.sec;
  msg1.time_from_start.nsec = msg2.time_from_start.nanosec;
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::JointCommand &msg2)
{
  baxter_core_msgs::JointCommand msg1;
  msg1.mode = msg2.mode;
  msg1.command = msg2.command;
  msg1.names = msg2.names;
  return msg1;
}

template<>
auto convertROS1(const std_msgs::msg::Float64 &msg2)
{
  std_msgs::Float64 msg1;
  msg1.data = msg2.data;
  return msg1;
}

template<>
auto convertROS1(const std_msgs::msg::Empty &)
{
  std_msgs::Empty msg1;
  return msg1;
}

template<>
auto convertROS1(const std_msgs::msg::Float32 &msg2)
{
  std_msgs::Float32 msg1;
  msg1.data = msg2.data;
  return msg1;
}

template<>
auto convertROS1(const baxter_core_msgs::msg::URDFConfiguration &msg2)
{
  baxter_core_msgs::URDFConfiguration msg1;
  msg1.time = Bridge::ros1_now();
  msg1.link = msg2.link;
  msg1.joint = msg2.joint;
  msg1.urdf = msg2.urdf;
  return msg1;
}

template<>
auto convertROS1(const sensor_msgs::msg::Image &msg2)
{
  sensor_msgs::Image msg1;
  msg1.header = convertROS1(msg2.header);
  msg1.height = msg2.height;
  msg1.width = msg2.width;
  msg1.encoding = msg2.encoding;
  msg1.is_bigendian = msg2.is_bigendian;
  msg1.step = msg2.step;
  msg1.data = msg2.data;
  return msg1;
}

template<>
auto convertROS1(const baxter_maintenance_msgs::msg::CalibrateArmData &msg2)
{
  baxter_maintenance_msgs::CalibrateArmData msg1;
  msg1.suppressWriteToFile = msg2.suppress_write_to_file;
  return msg1;
}

template<>
auto convertROS1(const baxter_maintenance_msgs::msg::CalibrateArmEnable &msg2)
{
  baxter_maintenance_msgs::CalibrateArmEnable msg1;
  msg1.isEnabled = msg2.is_enabled;
  msg1.uid = msg2.uid;
  msg1.data = convertROS1(msg2.data);
  return msg1;
}

template<>
auto convertROS1(const baxter_maintenance_msgs::msg::TareData &msg2)
{
  baxter_maintenance_msgs::TareData msg1;
  msg1.tuneGravitySpring = msg2.tune_gravity_spring;
  return msg1;
}

template<>
auto convertROS1(const baxter_maintenance_msgs::msg::TareEnable &msg2)
{
  baxter_maintenance_msgs::TareEnable msg1;
  msg1.isEnabled = msg2.is_enabled;
  msg1.uid = msg2.uid;
  msg1.data = convertROS1(msg2.data);
  return msg1;
}

std::map<std::string, std::string> Factory::topics_2to1 = {
  {"/diagnostics", "diagnostic_msgs/DiagnosticArray"},
  {"/robot/analog_io/command", "baxter_core_msgs/AnalogOutputCommand"},
  {"/robot/digital_io/command", "baxter_core_msgs/DigitalOutputCommand"},
  {"/robot/end_effector/left_gripper/command", "baxter_core_msgs/EndEffectorCommand"},
  {"/robot/end_effector/left_gripper/properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/left_gripper/rsdk/set_properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/left_gripper/rsdk/set_state", "baxter_core_msgs/EndEffectorState"},
  {"/robot/end_effector/right_gripper/command", "baxter_core_msgs/EndEffectorCommand"},
  {"/robot/end_effector/right_gripper/properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/right_gripper/rsdk/set_properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/right_gripper/rsdk/set_state", "baxter_core_msgs/EndEffectorState"},
  {"/robot/head/command_head_nod", "std_msgs/Bool"},
  {"/robot/head/command_head_pan", "baxter_core_msgs/HeadPanCommand"},
  {"/robot/head/head_state", "baxter_core_msgs/HeadState"},
  {"/robot/joint_state_publish_rate", "std_msgs/UInt16"},
  {"/robot/limb/left/collision_avoidance_state", "baxter_core_msgs/CollisionAvoidanceState"},
  {"/robot/limb/left/command_stiffness", "std_msgs/UInt32"},
  {"/robot/limb/left/command_twist_stamped", "geometry_msgs/TwistStamped"},
  {"/robot/limb/left/command_velocity_tozero", "std_msgs/Bool"},
  {"/robot/limb/left/commanded_endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/left/endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/left/gravity_compensation_torques", "baxter_core_msgs/SEAJointState"},
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
  {"/robot/limb/right/collision_avoidance_state", "baxter_core_msgs/CollisionAvoidanceState"},
  {"/robot/limb/right/command_stiffness", "std_msgs/UInt32"},
  {"/robot/limb/right/command_twist_stamped", "geometry_msgs/TwistStamped"},
  {"/robot/limb/right/command_velocity_tozero", "std_msgs/Bool"},
  {"/robot/limb/right/commanded_endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/right/endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/right/gravity_compensation_torques", "baxter_core_msgs/SEAJointState"},
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
  {"/robot/urdf", "baxter_core_msgs/URDFConfiguration"},
  {"/robot/xdisplay", "sensor_msgs/Image"},
  {"/robustcontroller/left/CalibrateArm/enable", "baxter_maintenance_msgs/CalibrateArmEnable"},
  {"/robustcontroller/left/Tare/enable", "baxter_maintenance_msgs/TareEnable"},
  {"/robustcontroller/right/CalibrateArm/enable", "baxter_maintenance_msgs/CalibrateArmEnable"},
  {"/robustcontroller/right/Tare/enable", "baxter_maintenance_msgs/TareEnable"}};

void Factory::createBridge_2to1(const std::string &topic, const std::string &msg)
{
  if(msg == "diagnostic_msgs/DiagnosticArray")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<diagnostic_msgs::DiagnosticArray, diagnostic_msgs::msg::DiagnosticArray>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/AnalogOutputCommand")
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
  else if(msg == "baxter_core_msgs/HeadState")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::HeadState, baxter_core_msgs::msg::HeadState>>
        (topic));
  }
  else if(msg == "std_msgs/UInt16")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<std_msgs::UInt16, std_msgs::msg::UInt16>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/CollisionAvoidanceState")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::CollisionAvoidanceState, baxter_core_msgs::msg::CollisionAvoidanceState>>
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
  else if(msg == "baxter_core_msgs/EndpointState")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::EndpointState, baxter_core_msgs::msg::EndpointState>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/SEAJointState")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::SEAJointState, baxter_core_msgs::msg::SEAJointState>>
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
  else if(msg == "baxter_core_msgs/URDFConfiguration")
  {
    bridges.push_back(std::make_unique<Bridge_2to1<baxter_core_msgs::URDFConfiguration, baxter_core_msgs::msg::URDFConfiguration>>
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
}
}