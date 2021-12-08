//Generated from baxter_io.yaml, edit is not recommended
#include <baxter_bridge/bridge_1to2.h>
#include <baxter_bridge/factory.h>
//messages
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/msg/image.hpp>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/UInt32.h>
#include <std_msgs/msg/u_int32.hpp>
#include <baxter_core_msgs/AnalogIOStates.h>
#include <baxter_core_msgs/msg/analog_io_states.hpp>
#include <baxter_core_msgs/DigitalOutputCommand.h>
#include <baxter_core_msgs/msg/digital_output_command.hpp>
#include <baxter_core_msgs/DigitalIOStates.h>
#include <baxter_core_msgs/msg/digital_io_states.hpp>
#include <baxter_core_msgs/EndEffectorCommand.h>
#include <baxter_core_msgs/msg/end_effector_command.hpp>
#include <baxter_core_msgs/EndEffectorProperties.h>
#include <baxter_core_msgs/msg/end_effector_properties.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/msg/bool.hpp>
#include <baxter_core_msgs/HeadPanCommand.h>
#include <baxter_core_msgs/msg/head_pan_command.hpp>
#include <baxter_core_msgs/HeadState.h>
#include <baxter_core_msgs/msg/head_state.hpp>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <baxter_core_msgs/CollisionAvoidanceState.h>
#include <baxter_core_msgs/msg/collision_avoidance_state.hpp>
#include <baxter_core_msgs/CollisionDetectionState.h>
#include <baxter_core_msgs/msg/collision_detection_state.hpp>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/msg/endpoint_state.hpp>
#include <baxter_core_msgs/SEAJointState.h>
#include <baxter_core_msgs/msg/sea_joint_state.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/msg/empty.hpp>
#include <baxter_core_msgs/NavigatorStates.h>
#include <baxter_core_msgs/msg/navigator_states.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/UInt16.h>
#include <std_msgs/msg/u_int16.hpp>
#include <baxter_core_msgs/URDFConfiguration.h>
#include <baxter_core_msgs/msg/urdf_configuration.hpp>
#include <baxter_core_msgs/RobustControllerStatus.h>
#include <baxter_core_msgs/msg/robust_controller_status.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/msg/int32.hpp>

namespace baxter_bridge
{
// converters
template<>
auto convertROS2(const std_msgs::Header &msg1)
{
  std_msgs::msg::Header msg2;
  msg2.stamp = Bridge::ros2_now();
  msg2.frame_id = msg1.frame_id;
  return msg2;
}

template<>
auto convertROS2(const sensor_msgs::RegionOfInterest &msg1)
{
  sensor_msgs::msg::RegionOfInterest msg2;
  msg2.x_offset = msg1.x_offset;
  msg2.y_offset = msg1.y_offset;
  msg2.height = msg1.height;
  msg2.width = msg1.width;
  msg2.do_rectify = msg1.do_rectify;
  return msg2;
}

template<>
auto convertROS2(const sensor_msgs::CameraInfo &msg1)
{
  sensor_msgs::msg::CameraInfo msg2;
  msg2.header = convertROS2(msg1.header);
  msg2.height = msg1.height;
  msg2.width = msg1.width;
  msg2.distortion_model = msg1.distortion_model;
  msg2.d = msg1.D;
  std::copy(msg1.K.begin(), msg1.K.end(), msg2.k.begin());
  std::copy(msg1.R.begin(), msg1.R.end(), msg2.r.begin());
  std::copy(msg1.P.begin(), msg1.P.end(), msg2.p.begin());
  msg2.binning_x = msg1.binning_x;
  msg2.binning_y = msg1.binning_y;
  msg2.roi = convertROS2(msg1.roi);
  return msg2;
}

template<>
auto convertROS2(const sensor_msgs::Image &msg1)
{
  sensor_msgs::msg::Image msg2;
  msg2.header = convertROS2(msg1.header);
  msg2.height = msg1.height;
  msg2.width = msg1.width;
  msg2.encoding = msg1.encoding;
  msg2.is_bigendian = msg1.is_bigendian;
  msg2.step = msg1.step;
  msg2.data = msg1.data;
  return msg2;
}

template<>
auto convertROS2(const diagnostic_msgs::KeyValue &msg1)
{
  diagnostic_msgs::msg::KeyValue msg2;
  msg2.key = msg1.key;
  msg2.value = msg1.value;
  return msg2;
}

template<>
auto convertROS2(const diagnostic_msgs::DiagnosticStatus &msg1)
{
  diagnostic_msgs::msg::DiagnosticStatus msg2;
  msg2.level = msg1.level;
  msg2.name = msg1.name;
  msg2.message = msg1.message;
  msg2.hardware_id = msg1.hardware_id;
  std::transform(msg1.values.begin(), msg1.values.end(),
                 std::back_inserter(msg2.values),
                 convertROS2<diagnostic_msgs::KeyValue>);
  return msg2;
}

template<>
auto convertROS2(const diagnostic_msgs::DiagnosticArray &msg1)
{
  diagnostic_msgs::msg::DiagnosticArray msg2;
  msg2.header = convertROS2(msg1.header);
  std::transform(msg1.status.begin(), msg1.status.end(),
                 std::back_inserter(msg2.status),
                 convertROS2<diagnostic_msgs::DiagnosticStatus>);
  return msg2;
}

template<>
auto convertROS2(const std_msgs::MultiArrayDimension &msg1)
{
  std_msgs::msg::MultiArrayDimension msg2;
  msg2.label = msg1.label;
  msg2.size = msg1.size;
  msg2.stride = msg1.stride;
  return msg2;
}

template<>
auto convertROS2(const std_msgs::MultiArrayLayout &msg1)
{
  std_msgs::msg::MultiArrayLayout msg2;
  std::transform(msg1.dim.begin(), msg1.dim.end(),
                 std::back_inserter(msg2.dim),
                 convertROS2<std_msgs::MultiArrayDimension>);
  msg2.data_offset = msg1.data_offset;
  return msg2;
}

template<>
auto convertROS2(const std_msgs::UInt8MultiArray &msg1)
{
  std_msgs::msg::UInt8MultiArray msg2;
  msg2.layout = convertROS2(msg1.layout);
  msg2.data = msg1.data;
  return msg2;
}

template<>
auto convertROS2(const std_msgs::UInt32 &msg1)
{
  std_msgs::msg::UInt32 msg2;
  msg2.data = msg1.data;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::AnalogIOState &msg1)
{
  baxter_core_msgs::msg::AnalogIOState msg2;
  msg2.timestamp = Bridge::ros2_now();
  msg2.value = msg1.value;
  msg2.is_input_only = msg1.isInputOnly;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::AnalogIOStates &msg1)
{
  baxter_core_msgs::msg::AnalogIOStates msg2;
  msg2.names = msg1.names;
  std::transform(msg1.states.begin(), msg1.states.end(),
                 std::back_inserter(msg2.states),
                 convertROS2<baxter_core_msgs::AnalogIOState>);
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::DigitalOutputCommand &msg1)
{
  baxter_core_msgs::msg::DigitalOutputCommand msg2;
  msg2.name = msg1.name;
  msg2.value = msg1.value;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::DigitalIOState &msg1)
{
  baxter_core_msgs::msg::DigitalIOState msg2;
  msg2.state = msg1.state;
  msg2.is_input_only = msg1.isInputOnly;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::DigitalIOStates &msg1)
{
  baxter_core_msgs::msg::DigitalIOStates msg2;
  msg2.names = msg1.names;
  std::transform(msg1.states.begin(), msg1.states.end(),
                 std::back_inserter(msg2.states),
                 convertROS2<baxter_core_msgs::DigitalIOState>);
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::EndEffectorCommand &msg1)
{
  baxter_core_msgs::msg::EndEffectorCommand msg2;
  msg2.id = msg1.id;
  msg2.command = msg1.command;
  msg2.args = msg1.args;
  msg2.sender = msg1.sender;
  msg2.sequence = msg1.sequence;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::EndEffectorProperties &msg1)
{
  baxter_core_msgs::msg::EndEffectorProperties msg2;
  msg2.id = msg1.id;
  msg2.ui_type = msg1.ui_type;
  msg2.manufacturer = msg1.manufacturer;
  msg2.product = msg1.product;
  msg2.serial_number = msg1.serial_number;
  msg2.hardware_rev = msg1.hardware_rev;
  msg2.firmware_rev = msg1.firmware_rev;
  msg2.firmware_date = msg1.firmware_date;
  msg2.has_calibration = msg1.has_calibration;
  msg2.controls_grip = msg1.controls_grip;
  msg2.senses_grip = msg1.senses_grip;
  msg2.reverses_grip = msg1.reverses_grip;
  msg2.controls_force = msg1.controls_force;
  msg2.senses_force = msg1.senses_force;
  msg2.controls_position = msg1.controls_position;
  msg2.senses_position = msg1.senses_position;
  msg2.properties = msg1.properties;
  return msg2;
}

template<>
auto convertROS2(const std_msgs::Bool &msg1)
{
  std_msgs::msg::Bool msg2;
  msg2.data = msg1.data;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::HeadPanCommand &msg1)
{
  baxter_core_msgs::msg::HeadPanCommand msg2;
  msg2.target = msg1.target;
  msg2.speed_ratio = msg1.speed_ratio;
  msg2.enable_pan_request = msg1.enable_pan_request;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::HeadState &msg1)
{
  baxter_core_msgs::msg::HeadState msg2;
  msg2.pan = msg1.pan;
  msg2.is_turning = msg1.isTurning;
  msg2.is_nodding = msg1.isNodding;
  msg2.is_pan_enabled = msg1.isPanEnabled;
  return msg2;
}

template<>
auto convertROS2(const sensor_msgs::JointState &msg1)
{
  sensor_msgs::msg::JointState msg2;
  msg2.header = convertROS2(msg1.header);
  msg2.name = msg1.name;
  msg2.position = msg1.position;
  msg2.velocity = msg1.velocity;
  msg2.effort = msg1.effort;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::CollisionAvoidanceState &msg1)
{
  baxter_core_msgs::msg::CollisionAvoidanceState msg2;
  msg2.header = convertROS2(msg1.header);
  msg2.other_arm = msg1.other_arm;
  msg2.collision_object = msg1.collision_object;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::CollisionDetectionState &msg1)
{
  baxter_core_msgs::msg::CollisionDetectionState msg2;
  msg2.header = convertROS2(msg1.header);
  msg2.collision_state = msg1.collision_state;
  return msg2;
}

template<>
auto convertROS2(const geometry_msgs::Point &msg1)
{
  geometry_msgs::msg::Point msg2;
  msg2.x = msg1.x;
  msg2.y = msg1.y;
  msg2.z = msg1.z;
  return msg2;
}

template<>
auto convertROS2(const geometry_msgs::Quaternion &msg1)
{
  geometry_msgs::msg::Quaternion msg2;
  msg2.x = msg1.x;
  msg2.y = msg1.y;
  msg2.z = msg1.z;
  msg2.w = msg1.w;
  return msg2;
}

template<>
auto convertROS2(const geometry_msgs::Pose &msg1)
{
  geometry_msgs::msg::Pose msg2;
  msg2.position = convertROS2(msg1.position);
  msg2.orientation = convertROS2(msg1.orientation);
  return msg2;
}

template<>
auto convertROS2(const geometry_msgs::Vector3 &msg1)
{
  geometry_msgs::msg::Vector3 msg2;
  msg2.x = msg1.x;
  msg2.y = msg1.y;
  msg2.z = msg1.z;
  return msg2;
}

template<>
auto convertROS2(const geometry_msgs::Twist &msg1)
{
  geometry_msgs::msg::Twist msg2;
  msg2.linear = convertROS2(msg1.linear);
  msg2.angular = convertROS2(msg1.angular);
  return msg2;
}

template<>
auto convertROS2(const geometry_msgs::Wrench &msg1)
{
  geometry_msgs::msg::Wrench msg2;
  msg2.force = convertROS2(msg1.force);
  msg2.torque = convertROS2(msg1.torque);
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::EndpointState &msg1)
{
  baxter_core_msgs::msg::EndpointState msg2;
  msg2.header = convertROS2(msg1.header);
  msg2.pose = convertROS2(msg1.pose);
  msg2.twist = convertROS2(msg1.twist);
  msg2.wrench = convertROS2(msg1.wrench);
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::SEAJointState &msg1)
{
  baxter_core_msgs::msg::SEAJointState msg2;
  msg2.header = convertROS2(msg1.header);
  msg2.name = msg1.name;
  msg2.commanded_position = msg1.commanded_position;
  msg2.commanded_velocity = msg1.commanded_velocity;
  msg2.commanded_acceleration = msg1.commanded_acceleration;
  msg2.commanded_effort = msg1.commanded_effort;
  msg2.actual_position = msg1.actual_position;
  msg2.actual_velocity = msg1.actual_velocity;
  msg2.actual_effort = msg1.actual_effort;
  msg2.gravity_model_effort = msg1.gravity_model_effort;
  msg2.gravity_only = msg1.gravity_only;
  msg2.hysteresis_model_effort = msg1.hysteresis_model_effort;
  msg2.crosstalk_model_effort = msg1.crosstalk_model_effort;
  msg2.hyst_state = msg1.hystState;
  return msg2;
}

template<>
auto convertROS2(const std_msgs::Empty &)
{
  std_msgs::msg::Empty msg2;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::NavigatorState &msg1)
{
  baxter_core_msgs::msg::NavigatorState msg2;
  msg2.button_names = msg1.button_names;
  std::transform(msg1.buttons.begin(), msg1.buttons.end(), 
    std::back_inserter(msg2.buttons), [](auto b){return static_cast<bool>(b);});
  msg2.wheel = msg1.wheel;
  msg2.light_names = msg1.light_names;
  std::transform(msg1.lights.begin(), msg1.lights.end(), 
    std::back_inserter(msg2.lights), [](auto b){return static_cast<bool>(b);});
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::NavigatorStates &msg1)
{
  baxter_core_msgs::msg::NavigatorStates msg2;
  msg2.names = msg1.names;
  std::transform(msg1.states.begin(), msg1.states.end(),
                 std::back_inserter(msg2.states),
                 convertROS2<baxter_core_msgs::NavigatorState>);
  return msg2;
}

template<>
auto convertROS2(const std_msgs::Float32 &msg1)
{
  std_msgs::msg::Float32 msg2;
  msg2.data = msg1.data;
  return msg2;
}

template<>
auto convertROS2(const std_msgs::UInt16 &msg1)
{
  std_msgs::msg::UInt16 msg2;
  msg2.data = msg1.data;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::URDFConfiguration &msg1)
{
  baxter_core_msgs::msg::URDFConfiguration msg2;
  msg2.time = Bridge::ros2_now();
  msg2.link = msg1.link;
  msg2.joint = msg1.joint;
  msg2.urdf = msg1.urdf;
  return msg2;
}

template<>
auto convertROS2(const baxter_core_msgs::RobustControllerStatus &msg1)
{
  baxter_core_msgs::msg::RobustControllerStatus msg2;
  msg2.is_enabled = msg1.isEnabled;
  msg2.complete = msg1.complete;
  msg2.control_uid = msg1.controlUid;
  msg2.timed_out = msg1.timedOut;
  msg2.error_codes = msg1.errorCodes;
  msg2.labels = msg1.labels;
  return msg2;
}

template<>
auto convertROS2(const std_msgs::Int32 &msg1)
{
  std_msgs::msg::Int32 msg2;
  msg2.data = msg1.data;
  return msg2;
}

std::map<std::string, std::string> Factory::topics_1to2 = {
  {"/cameras/left_hand_camera/camera_info", "sensor_msgs/CameraInfo"},
  {"/cameras/left_hand_camera/camera_info_std", "sensor_msgs/CameraInfo"},
  {"/cameras/left_hand_camera/image", "sensor_msgs/Image"},
  {"/cameras/right_hand_camera/camera_info", "sensor_msgs/CameraInfo"},
  {"/cameras/right_hand_camera/camera_info_std", "sensor_msgs/CameraInfo"},
  {"/cameras/right_hand_camera/image", "sensor_msgs/Image"},
  {"/diagnostics", "diagnostic_msgs/DiagnosticArray"},
  {"/diagnostics_agg", "diagnostic_msgs/DiagnosticArray"},
  {"/diagnostics_toplevel_state", "diagnostic_msgs/DiagnosticStatus"},
  {"/hdraw", "std_msgs/UInt8MultiArray"},
  {"/robot/analog_io/left_hand_range/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/left_vacuum_sensor_analog/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/left_wheel/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/right_hand_range/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/right_vacuum_sensor_analog/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/right_wheel/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/torso_fan/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/torso_left_wheel/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/torso_lighting/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/torso_right_wheel/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io_states", "baxter_core_msgs/AnalogIOStates"},
  {"/robot/digital_io/command", "baxter_core_msgs/DigitalOutputCommand"},
  {"/robot/digital_io_states", "baxter_core_msgs/DigitalIOStates"},
  {"/robot/end_effector/left_gripper/command", "baxter_core_msgs/EndEffectorCommand"},
  {"/robot/end_effector/left_gripper/properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/right_gripper/command", "baxter_core_msgs/EndEffectorCommand"},
  {"/robot/end_effector/right_gripper/properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/head/command_head_nod", "std_msgs/Bool"},
  {"/robot/head/command_head_pan", "baxter_core_msgs/HeadPanCommand"},
  {"/robot/head/head_state", "baxter_core_msgs/HeadState"},
  {"/robot/joint_states", "sensor_msgs/JointState"},
  {"/robot/limb/left/collision_avoidance_state", "baxter_core_msgs/CollisionAvoidanceState"},
  {"/robot/limb/left/collision_detection_state", "baxter_core_msgs/CollisionDetectionState"},
  {"/robot/limb/left/commanded_endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/left/endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/left/gravity_compensation_torques", "baxter_core_msgs/SEAJointState"},
  {"/robot/limb/left/suppress_body_avoidance", "std_msgs/Empty"},
  {"/robot/limb/left/suppress_collision_avoidance", "std_msgs/Empty"},
  {"/robot/limb/left/suppress_contact_safety", "std_msgs/Empty"},
  {"/robot/limb/left/suppress_hand_overwrench_safety", "std_msgs/Empty"},
  {"/robot/limb/left/use_default_spring_model", "std_msgs/Empty"},
  {"/robot/limb/right/collision_avoidance_state", "baxter_core_msgs/CollisionAvoidanceState"},
  {"/robot/limb/right/collision_detection_state", "baxter_core_msgs/CollisionDetectionState"},
  {"/robot/limb/right/commanded_endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/right/endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/right/gravity_compensation_torques", "baxter_core_msgs/SEAJointState"},
  {"/robot/limb/right/suppress_body_avoidance", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_collision_avoidance", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_contact_safety", "std_msgs/Empty"},
  {"/robot/limb/right/suppress_hand_overwrench_safety", "std_msgs/Empty"},
  {"/robot/limb/right/use_default_spring_model", "std_msgs/Empty"},
  {"/robot/navigators_states", "baxter_core_msgs/NavigatorStates"},
  {"/robot/set_super_enable", "std_msgs/Bool"},
  {"/robot/set_super_reset", "std_msgs/Empty"},
  {"/robot/sonar/head_sonar/lights/green_level", "std_msgs/Float32"},
  {"/robot/sonar/head_sonar/lights/red_level", "std_msgs/Float32"},
  {"/robot/sonar/head_sonar/sonars_enabled", "std_msgs/UInt16"},
  {"/robot/urdf", "baxter_core_msgs/URDFConfiguration"},
  {"/robustcontroller/left/CalibrateArm/status", "baxter_core_msgs/RobustControllerStatus"},
  {"/robustcontroller/left/Tare/status", "baxter_core_msgs/RobustControllerStatus"},
  {"/robustcontroller/left/rc_plugins_loaded", "std_msgs/Bool"},
  {"/robustcontroller/right/CalibrateArm/status", "baxter_core_msgs/RobustControllerStatus"},
  {"/robustcontroller/right/Tare/status", "baxter_core_msgs/RobustControllerStatus"},
  {"/robustcontroller/right/rc_plugins_loaded", "std_msgs/Bool"},
  {"/update/progress", "std_msgs/Int32"},
  {"/update/status", "std_msgs/Int32"},
  {"/usb/ready", "std_msgs/Bool"}};

void Factory::createBridge_1to2(const std::string &topic, const std::string &msg)
{
  if(msg == "sensor_msgs/CameraInfo")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<sensor_msgs::CameraInfo, sensor_msgs::msg::CameraInfo>>
        (topic));
  }
  else if(msg == "sensor_msgs/Image")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<sensor_msgs::Image, sensor_msgs::msg::Image>>
        (topic));
  }
  else if(msg == "diagnostic_msgs/DiagnosticArray")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<diagnostic_msgs::DiagnosticArray, diagnostic_msgs::msg::DiagnosticArray>>
        (topic));
  }
  else if(msg == "diagnostic_msgs/DiagnosticStatus")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<diagnostic_msgs::DiagnosticStatus, diagnostic_msgs::msg::DiagnosticStatus>>
        (topic));
  }
  else if(msg == "std_msgs/UInt8MultiArray")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::UInt8MultiArray, std_msgs::msg::UInt8MultiArray>>
        (topic));
  }
  else if(msg == "std_msgs/UInt32")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::UInt32, std_msgs::msg::UInt32>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/AnalogIOStates")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::AnalogIOStates, baxter_core_msgs::msg::AnalogIOStates>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/DigitalOutputCommand")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::DigitalOutputCommand, baxter_core_msgs::msg::DigitalOutputCommand>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/DigitalIOStates")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::DigitalIOStates, baxter_core_msgs::msg::DigitalIOStates>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/EndEffectorCommand")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::EndEffectorCommand, baxter_core_msgs::msg::EndEffectorCommand>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/EndEffectorProperties")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::EndEffectorProperties, baxter_core_msgs::msg::EndEffectorProperties>>
        (topic));
  }
  else if(msg == "std_msgs/Bool")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::Bool, std_msgs::msg::Bool>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/HeadPanCommand")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::HeadPanCommand, baxter_core_msgs::msg::HeadPanCommand>>
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
  else if(msg == "baxter_core_msgs/CollisionDetectionState")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::CollisionDetectionState, baxter_core_msgs::msg::CollisionDetectionState>>
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
  else if(msg == "std_msgs/Empty")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::Empty, std_msgs::msg::Empty>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/NavigatorStates")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::NavigatorStates, baxter_core_msgs::msg::NavigatorStates>>
        (topic));
  }
  else if(msg == "std_msgs/Float32")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::Float32, std_msgs::msg::Float32>>
        (topic));
  }
  else if(msg == "std_msgs/UInt16")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::UInt16, std_msgs::msg::UInt16>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/URDFConfiguration")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::URDFConfiguration, baxter_core_msgs::msg::URDFConfiguration>>
        (topic));
  }
  else if(msg == "baxter_core_msgs/RobustControllerStatus")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::RobustControllerStatus, baxter_core_msgs::msg::RobustControllerStatus>>
        (topic));
  }
  else if(msg == "std_msgs/Int32")
  {
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::Int32, std_msgs::msg::Int32>>
        (topic));
  }
}
}