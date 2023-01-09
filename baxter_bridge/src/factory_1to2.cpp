//Generated from baxter_io.yaml, edit is not recommended
#include <baxter_bridge/bridge_1to2.h>
#include <baxter_bridge/factory.h>
//messages
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <baxter_core_msgs/DigitalIOState.h>
#include <baxter_core_msgs/msg/digital_io_state.hpp>
#include <baxter_core_msgs/EndEffectorProperties.h>
#include <baxter_core_msgs/msg/end_effector_properties.hpp>
#include <baxter_core_msgs/EndEffectorState.h>
#include <baxter_core_msgs/msg/end_effector_state.hpp>
#include <baxter_core_msgs/HeadState.h>
#include <baxter_core_msgs/msg/head_state.hpp>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <baxter_core_msgs/CollisionAvoidanceState.h>
#include <baxter_core_msgs/msg/collision_avoidance_state.hpp>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/msg/endpoint_state.hpp>
#include <baxter_core_msgs/SEAJointState.h>
#include <baxter_core_msgs/msg/sea_joint_state.hpp>
#include <baxter_core_msgs/AssemblyState.h>
#include <baxter_core_msgs/msg/assembly_state.hpp>

namespace baxter_bridge
{
// converters
template<>
void convertMsg(const std_msgs::Header &src, std_msgs::msg::Header &dst)
{
  dst.stamp = Bridge::ros2_now();
  dst.frame_id = src.frame_id;
}

template<>
void convertMsg(const diagnostic_msgs::KeyValue &src, diagnostic_msgs::msg::KeyValue &dst)
{
  dst.key = src.key;
  dst.value = src.value;
}

template<>
void convertMsg(const diagnostic_msgs::DiagnosticStatus &src, diagnostic_msgs::msg::DiagnosticStatus &dst)
{
  dst.level = src.level;
  dst.name = src.name;
  dst.message = src.message;
  dst.hardware_id = src.hardware_id;
  convertMsg(src.values, dst.values);
}

template<>
void convertMsg(const diagnostic_msgs::DiagnosticArray &src, diagnostic_msgs::msg::DiagnosticArray &dst)
{
  convertMsg(src.header, dst.header);
  convertMsg(src.status, dst.status);
}

template<>
void convertMsg(const baxter_core_msgs::DigitalIOState &src, baxter_core_msgs::msg::DigitalIOState &dst)
{
  dst.state = src.state;
  dst.is_input_only = src.isInputOnly;
}

template<>
void convertMsg(const baxter_core_msgs::EndEffectorProperties &src, baxter_core_msgs::msg::EndEffectorProperties &dst)
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
void convertMsg(const baxter_core_msgs::EndEffectorState &src, baxter_core_msgs::msg::EndEffectorState &dst)
{
  dst.timestamp = Bridge::ros2_now();
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
void convertMsg(const baxter_core_msgs::HeadState &src, baxter_core_msgs::msg::HeadState &dst)
{
  dst.pan = src.pan;
  dst.is_turning = src.isTurning;
  dst.is_nodding = src.isNodding;
  dst.is_pan_enabled = src.isPanEnabled;
}

template<>
void convertMsg(const sensor_msgs::JointState &src, sensor_msgs::msg::JointState &dst)
{
  convertMsg(src.header, dst.header);
  dst.name = src.name;
  dst.position = src.position;
  dst.velocity = src.velocity;
  dst.effort = src.effort;
}

template<>
void convertMsg(const baxter_core_msgs::CollisionAvoidanceState &src, baxter_core_msgs::msg::CollisionAvoidanceState &dst)
{
  convertMsg(src.header, dst.header);
  dst.other_arm = src.other_arm;
  dst.collision_object = src.collision_object;
}

template<>
void convertMsg(const geometry_msgs::Point &src, geometry_msgs::msg::Point &dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}

template<>
void convertMsg(const geometry_msgs::Quaternion &src, geometry_msgs::msg::Quaternion &dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
  dst.w = src.w;
}

template<>
void convertMsg(const geometry_msgs::Pose &src, geometry_msgs::msg::Pose &dst)
{
  convertMsg(src.position, dst.position);
  convertMsg(src.orientation, dst.orientation);
}

template<>
void convertMsg(const geometry_msgs::Vector3 &src, geometry_msgs::msg::Vector3 &dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}

template<>
void convertMsg(const geometry_msgs::Twist &src, geometry_msgs::msg::Twist &dst)
{
  convertMsg(src.linear, dst.linear);
  convertMsg(src.angular, dst.angular);
}

template<>
void convertMsg(const geometry_msgs::Wrench &src, geometry_msgs::msg::Wrench &dst)
{
  convertMsg(src.force, dst.force);
  convertMsg(src.torque, dst.torque);
}

template<>
void convertMsg(const baxter_core_msgs::EndpointState &src, baxter_core_msgs::msg::EndpointState &dst)
{
  convertMsg(src.header, dst.header);
  convertMsg(src.pose, dst.pose);
  convertMsg(src.twist, dst.twist);
  convertMsg(src.wrench, dst.wrench);
}

template<>
void convertMsg(const baxter_core_msgs::SEAJointState &src, baxter_core_msgs::msg::SEAJointState &dst)
{
  convertMsg(src.header, dst.header);
  dst.name = src.name;
  dst.commanded_position = src.commanded_position;
  dst.commanded_velocity = src.commanded_velocity;
  dst.commanded_acceleration = src.commanded_acceleration;
  dst.commanded_effort = src.commanded_effort;
  dst.actual_position = src.actual_position;
  dst.actual_velocity = src.actual_velocity;
  dst.actual_effort = src.actual_effort;
  dst.gravity_model_effort = src.gravity_model_effort;
  dst.gravity_only = src.gravity_only;
  dst.hysteresis_model_effort = src.hysteresis_model_effort;
  dst.crosstalk_model_effort = src.crosstalk_model_effort;
  dst.hyst_state = src.hystState;
}

template<>
void convertMsg(const baxter_core_msgs::AssemblyState &src, baxter_core_msgs::msg::AssemblyState &dst)
{
  dst.ready = src.ready;
  dst.enabled = src.enabled;
  dst.stopped = src.stopped;
  dst.error = src.error;
  dst.estop_button = src.estop_button;
  dst.estop_source = src.estop_source;
}

std::map<std::string, std::string> Factory::topics_1to2 = {
  {"/diagnostics", "diagnostic_msgs/DiagnosticArray"},
  {"/robot/digital_io/left_hand_camera_power/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_hand_camera_power/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_camera_power/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_process_sense0/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_safety_stop/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/end_effector/left_gripper/properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/left_gripper/state", "baxter_core_msgs/EndEffectorState"},
  {"/robot/end_effector/right_gripper/properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/right_gripper/state", "baxter_core_msgs/EndEffectorState"},
  {"/robot/head/head_state", "baxter_core_msgs/HeadState"},
  {"/robot/joint_states", "sensor_msgs/JointState"},
  {"/robot/limb/left/collision_avoidance_state", "baxter_core_msgs/CollisionAvoidanceState"},
  {"/robot/limb/left/commanded_endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/left/endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/left/gravity_compensation_torques", "baxter_core_msgs/SEAJointState"},
  {"/robot/limb/right/collision_avoidance_state", "baxter_core_msgs/CollisionAvoidanceState"},
  {"/robot/limb/right/commanded_endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/right/endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/right/gravity_compensation_torques", "baxter_core_msgs/SEAJointState"},
  {"/robot/ref_joint_states", "sensor_msgs/JointState"},
  {"/robot/state", "baxter_core_msgs/AssemblyState"}};

void Factory::createBridge_1to2(const std::string &topic, const std::string &msg)
{
  if(msg == "diagnostic_msgs/DiagnosticArray")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<diagnostic_msgs::DiagnosticArray, diagnostic_msgs::msg::DiagnosticArray>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/DigitalIOState")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::DigitalIOState, baxter_core_msgs::msg::DigitalIOState>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/EndEffectorProperties")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::EndEffectorProperties, baxter_core_msgs::msg::EndEffectorProperties>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/EndEffectorState")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::EndEffectorState, baxter_core_msgs::msg::EndEffectorState>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/HeadState")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::HeadState, baxter_core_msgs::msg::HeadState>>
        (topic));
  }
  else if(msg == "sensor_msgs/JointState")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<sensor_msgs::JointState, sensor_msgs::msg::JointState>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/CollisionAvoidanceState")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::CollisionAvoidanceState, baxter_core_msgs::msg::CollisionAvoidanceState>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/EndpointState")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::EndpointState, baxter_core_msgs::msg::EndpointState>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/SEAJointState")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::SEAJointState, baxter_core_msgs::msg::SEAJointState>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/AssemblyState")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::AssemblyState, baxter_core_msgs::msg::AssemblyState>>
        (topic));
  }
}
}