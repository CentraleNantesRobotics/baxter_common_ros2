//Generated with gen_factory.py, edit is not recommended
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
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/msg/imu.hpp>
#include <baxter_core_msgs/AnalogIOState.h>
#include <baxter_core_msgs/msg/analog_io_state.hpp>
#include <std_msgs/UInt32.h>
#include <std_msgs/msg/u_int32.hpp>
#include <baxter_core_msgs/AnalogIOStates.h>
#include <baxter_core_msgs/msg/analog_io_states.hpp>
#include <baxter_core_msgs/AssemblyState.h>
#include <baxter_core_msgs/msg/assembly_state.hpp>
#include <baxter_core_msgs/DigitalIOState.h>
#include <baxter_core_msgs/msg/digital_io_state.hpp>
#include <baxter_core_msgs/DigitalIOStates.h>
#include <baxter_core_msgs/msg/digital_io_states.hpp>
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
#include <baxter_core_msgs/CollisionDetectionState.h>
#include <baxter_core_msgs/msg/collision_detection_state.hpp>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/msg/endpoint_state.hpp>
#include <baxter_core_msgs/SEAJointState.h>
#include <baxter_core_msgs/msg/sea_joint_state.hpp>
#include <std_msgs/Empty.h>
#include <std_msgs/msg/empty.hpp>
#include <baxter_core_msgs/NavigatorState.h>
#include <baxter_core_msgs/msg/navigator_state.hpp>
#include <baxter_core_msgs/NavigatorStates.h>
#include <baxter_core_msgs/msg/navigator_states.hpp>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/msg/range.hpp>
#include <std_msgs/Float32.h>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/UInt16.h>
#include <std_msgs/msg/u_int16.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <baxter_core_msgs/RobustControllerStatus.h>
#include <baxter_core_msgs/msg/robust_controller_status.hpp>
#include <std_msgs/Bool.h>
#include <std_msgs/msg/bool.hpp>
#include <actionlib_msgs/GoalStatusArray.h>
#include <actionlib_msgs/msg/goal_status_array.hpp>
#include <std_msgs/Int32.h>
#include <std_msgs/msg/int32.hpp>

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
void convertMsg(const sensor_msgs::RegionOfInterest &src, sensor_msgs::msg::RegionOfInterest &dst)
{
  dst.x_offset = src.x_offset;
  dst.y_offset = src.y_offset;
  dst.height = src.height;
  dst.width = src.width;
  dst.do_rectify = src.do_rectify;
}

template<>
void convertMsg(const sensor_msgs::CameraInfo &src, sensor_msgs::msg::CameraInfo &dst)
{
  convertMsg(src.header, dst.header);
  dst.height = src.height;
  dst.width = src.width;
  dst.distortion_model = src.distortion_model;
  dst.d = src.D;
  convertMsg(src.K, dst.k);
  convertMsg(src.R, dst.r);
  convertMsg(src.P, dst.p);
  dst.binning_x = src.binning_x;
  dst.binning_y = src.binning_y;
  convertMsg(src.roi, dst.roi);
}

template<>
void convertMsg(const sensor_msgs::Image &src, sensor_msgs::msg::Image &dst)
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
void convertMsg(const std_msgs::MultiArrayDimension &src, std_msgs::msg::MultiArrayDimension &dst)
{
  dst.label = src.label;
  dst.size = src.size;
  dst.stride = src.stride;
}

template<>
void convertMsg(const std_msgs::MultiArrayLayout &src, std_msgs::msg::MultiArrayLayout &dst)
{
  convertMsg(src.dim, dst.dim);
  dst.data_offset = src.data_offset;
}

template<>
void convertMsg(const std_msgs::UInt8MultiArray &src, std_msgs::msg::UInt8MultiArray &dst)
{
  convertMsg(src.layout, dst.layout);
  dst.data = src.data;
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
void convertMsg(const geometry_msgs::Vector3 &src, geometry_msgs::msg::Vector3 &dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}

template<>
void convertMsg(const sensor_msgs::Imu &src, sensor_msgs::msg::Imu &dst)
{
  convertMsg(src.header, dst.header);
  convertMsg(src.orientation, dst.orientation);
  convertMsg(src.orientation_covariance, dst.orientation_covariance);
  convertMsg(src.angular_velocity, dst.angular_velocity);
  convertMsg(src.angular_velocity_covariance, dst.angular_velocity_covariance);
  convertMsg(src.linear_acceleration, dst.linear_acceleration);
  convertMsg(src.linear_acceleration_covariance, dst.linear_acceleration_covariance);
}

template<>
void convertMsg(const baxter_core_msgs::AnalogIOState &src, baxter_core_msgs::msg::AnalogIOState &dst)
{
  dst.timestamp = Bridge::ros2_now();
  dst.value = src.value;
  dst.is_input_only = src.isInputOnly;
}

template<>
void convertMsg(const std_msgs::UInt32 &src, std_msgs::msg::UInt32 &dst)
{
  dst.data = src.data;
}

template<>
void convertMsg(const baxter_core_msgs::AnalogIOStates &src, baxter_core_msgs::msg::AnalogIOStates &dst)
{
  dst.names = src.names;
  convertMsg(src.states, dst.states);
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

template<>
void convertMsg(const baxter_core_msgs::DigitalIOState &src, baxter_core_msgs::msg::DigitalIOState &dst)
{
  dst.state = src.state;
  dst.is_input_only = src.isInputOnly;
}

template<>
void convertMsg(const baxter_core_msgs::DigitalIOStates &src, baxter_core_msgs::msg::DigitalIOStates &dst)
{
  dst.names = src.names;
  convertMsg(src.states, dst.states);
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
void convertMsg(const baxter_core_msgs::CollisionDetectionState &src, baxter_core_msgs::msg::CollisionDetectionState &dst)
{
  convertMsg(src.header, dst.header);
  dst.collision_state = src.collision_state;
}

template<>
void convertMsg(const geometry_msgs::Point &src, geometry_msgs::msg::Point &dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}

template<>
void convertMsg(const geometry_msgs::Pose &src, geometry_msgs::msg::Pose &dst)
{
  convertMsg(src.position, dst.position);
  convertMsg(src.orientation, dst.orientation);
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
void convertMsg(const std_msgs::Empty &, std_msgs::msg::Empty &)
{
}

template<>
void convertMsg(const baxter_core_msgs::NavigatorState &src, baxter_core_msgs::msg::NavigatorState &dst)
{
  dst.button_names = src.button_names;
  convertMsg(src.buttons, dst.buttons);
  dst.wheel = src.wheel;
  dst.light_names = src.light_names;
  convertMsg(src.lights, dst.lights);
}

template<>
void convertMsg(const baxter_core_msgs::NavigatorStates &src, baxter_core_msgs::msg::NavigatorStates &dst)
{
  dst.names = src.names;
  convertMsg(src.states, dst.states);
}

template<>
void convertMsg(const sensor_msgs::Range &src, sensor_msgs::msg::Range &dst)
{
  convertMsg(src.header, dst.header);
  dst.radiation_type = src.radiation_type;
  dst.field_of_view = src.field_of_view;
  dst.min_range = src.min_range;
  dst.max_range = src.max_range;
  dst.range = src.range;
}

template<>
void convertMsg(const std_msgs::Float32 &src, std_msgs::msg::Float32 &dst)
{
  dst.data = src.data;
}

template<>
void convertMsg(const std_msgs::UInt16 &src, std_msgs::msg::UInt16 &dst)
{
  dst.data = src.data;
}

template<>
void convertMsg(const geometry_msgs::Point32 &src, geometry_msgs::msg::Point32 &dst)
{
  dst.x = src.x;
  dst.y = src.y;
  dst.z = src.z;
}

template<>
void convertMsg(const sensor_msgs::ChannelFloat32 &src, sensor_msgs::msg::ChannelFloat32 &dst)
{
  dst.name = src.name;
  dst.values = src.values;
}

template<>
void convertMsg(const sensor_msgs::PointCloud &src, sensor_msgs::msg::PointCloud &dst)
{
  convertMsg(src.header, dst.header);
  convertMsg(src.points, dst.points);
  convertMsg(src.channels, dst.channels);
}

template<>
void convertMsg(const baxter_core_msgs::RobustControllerStatus &src, baxter_core_msgs::msg::RobustControllerStatus &dst)
{
  dst.is_enabled = src.isEnabled;
  dst.complete = src.complete;
  dst.control_uid = src.controlUid;
  dst.timed_out = src.timedOut;
  dst.error_codes = src.errorCodes;
  dst.labels = src.labels;
}

template<>
void convertMsg(const std_msgs::Bool &src, std_msgs::msg::Bool &dst)
{
  dst.data = src.data;
}

template<>
void convertMsg(const actionlib_msgs::GoalID &src, actionlib_msgs::msg::GoalID &dst)
{
  dst.stamp = Bridge::ros2_now();
  dst.id = src.id;
}

template<>
void convertMsg(const actionlib_msgs::GoalStatus &src, actionlib_msgs::msg::GoalStatus &dst)
{
  convertMsg(src.goal_id, dst.goal_id);
  dst.status = src.status;
  dst.text = src.text;
}

template<>
void convertMsg(const actionlib_msgs::GoalStatusArray &src, actionlib_msgs::msg::GoalStatusArray &dst)
{
  convertMsg(src.header, dst.header);
  convertMsg(src.status_list, dst.status_list);
}

template<>
void convertMsg(const std_msgs::Int32 &src, std_msgs::msg::Int32 &dst)
{
  dst.data = src.data;
}

std::map<std::string, std::string> Factory::topics_1to2 = {
  {"/cameras/head_camera/camera_info", "sensor_msgs/CameraInfo"},
  {"/cameras/head_camera/camera_info_std", "sensor_msgs/CameraInfo"},
  {"/cameras/head_camera/image", "sensor_msgs/Image"},
  {"/cameras/right_hand_camera/camera_info", "sensor_msgs/CameraInfo"},
  {"/cameras/right_hand_camera/camera_info_std", "sensor_msgs/CameraInfo"},
  {"/cameras/right_hand_camera/image", "sensor_msgs/Image"},
  {"/cameras/left_hand_camera/camera_info", "sensor_msgs/CameraInfo"},
  {"/cameras/left_hand_camera/camera_info_std", "sensor_msgs/CameraInfo"},
  {"/cameras/left_hand_camera/image", "sensor_msgs/Image"},
  {"/diagnostics", "diagnostic_msgs/DiagnosticArray"},
  {"/diagnostics_agg", "diagnostic_msgs/DiagnosticArray"},
  {"/diagnostics_toplevel_state", "diagnostic_msgs/DiagnosticStatus"},
  {"/hdraw", "std_msgs/UInt8MultiArray"},
  {"/robot/accelerometer/left_accelerometer/state", "sensor_msgs/Imu"},
  {"/robot/accelerometer/right_accelerometer/state", "sensor_msgs/Imu"},
  {"/robot/analog_io/left_hand_range/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/left_hand_range/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/left_vacuum_sensor_analog/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/left_vacuum_sensor_analog/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/left_wheel/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/left_wheel/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/right_hand_range/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/right_hand_range/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/right_vacuum_sensor_analog/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/right_vacuum_sensor_analog/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/right_wheel/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/right_wheel/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/torso_fan/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/torso_fan/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/torso_left_wheel/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/torso_left_wheel/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/torso_lighting/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/torso_lighting/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io/torso_right_wheel/state", "baxter_core_msgs/AnalogIOState"},
  {"/robot/analog_io/torso_right_wheel/value_uint32", "std_msgs/UInt32"},
  {"/robot/analog_io_states", "baxter_core_msgs/AnalogIOStates"},
  {"/robot/assembly/head/state", "baxter_core_msgs/AssemblyState"},
  {"/robot/assembly/left/state", "baxter_core_msgs/AssemblyState"},
  {"/robot/assembly/right/state", "baxter_core_msgs/AssemblyState"},
  {"/robot/assembly/torso/state", "baxter_core_msgs/AssemblyState"},
  {"/robot/digital_io/head_green_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/head_lcd_auto_config/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/head_red_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/head_yellow_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_blow/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_button_back/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_button_ok/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_button_show/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_hand_camera_power/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_inner_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_lower_button/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_lower_cuff/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_outer_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_pneumatic/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_shoulder_button/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_suck/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/left_upper_button/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/limit_switch_1/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/limit_switch_2/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/limit_switch_3/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/motor_fault_signal/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_blow/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_blue_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_button_back/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_button_ok/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_button_show/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_hand_camera_power/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_inner_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_lower_button/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_lower_cuff/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_outer_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_pneumatic/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_shoulder_button/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_suck/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/right_upper_button/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_brake/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_brake_sensor/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_camera_power/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_digital_input0/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_foot_pedal/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_left_button_back/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_left_button_ok/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_left_button_show/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_left_inner_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_left_outer_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_process_sense0/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_process_sense1/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_right_button_back/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_right_button_ok/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_right_button_show/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_right_inner_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_right_outer_light/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_safety_stop/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_ui_output0/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_ui_output1/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_ui_output2/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io/torso_ui_output3/state", "baxter_core_msgs/DigitalIOState"},
  {"/robot/digital_io_states", "baxter_core_msgs/DigitalIOStates"},
  {"/robot/end_effector/left_gripper/properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/left_gripper/state", "baxter_core_msgs/EndEffectorState"},
  {"/robot/end_effector/right_gripper/properties", "baxter_core_msgs/EndEffectorProperties"},
  {"/robot/end_effector/right_gripper/state", "baxter_core_msgs/EndEffectorState"},
  {"/robot/head/head_state", "baxter_core_msgs/HeadState"},
  {"/robot/head/state", "baxter_core_msgs/AssemblyState"},
  {"/robot/joint_states", "sensor_msgs/JointState"},
  {"/robot/limb/left/collision_avoidance_state", "baxter_core_msgs/CollisionAvoidanceState"},
  {"/robot/limb/left/collision_detection_state", "baxter_core_msgs/CollisionDetectionState"},
  {"/robot/limb/left/commanded_endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/left/endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/left/gravity_compensation_torques", "baxter_core_msgs/SEAJointState"},
  {"/robot/limb/left/state", "baxter_core_msgs/AssemblyState"},
  {"/robot/limb/left/suppress_body_avoidance", "std_msgs/Empty"},
  {"/robot/limb/right/collision_avoidance_state", "baxter_core_msgs/CollisionAvoidanceState"},
  {"/robot/limb/right/collision_detection_state", "baxter_core_msgs/CollisionDetectionState"},
  {"/robot/limb/right/commanded_endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/right/endpoint_state", "baxter_core_msgs/EndpointState"},
  {"/robot/limb/right/gravity_compensation_torques", "baxter_core_msgs/SEAJointState"},
  {"/robot/limb/right/state", "baxter_core_msgs/AssemblyState"},
  {"/robot/limb/right/suppress_body_avoidance", "std_msgs/Empty"},
  {"/robot/navigators/left_navigator/state", "baxter_core_msgs/NavigatorState"},
  {"/robot/navigators/right_navigator/state", "baxter_core_msgs/NavigatorState"},
  {"/robot/navigators/torso_left_navigator/state", "baxter_core_msgs/NavigatorState"},
  {"/robot/navigators/torso_right_navigator/state", "baxter_core_msgs/NavigatorState"},
  {"/robot/navigators_states", "baxter_core_msgs/NavigatorStates"},
  {"/robot/range/left_hand_range/state", "sensor_msgs/Range"},
  {"/robot/range/right_hand_range/state", "sensor_msgs/Range"},
  {"/robot/ref_joint_states", "sensor_msgs/JointState"},
  {"/robot/sonar/head_sonar/lights/green_level", "std_msgs/Float32"},
  {"/robot/sonar/head_sonar/lights/red_level", "std_msgs/Float32"},
  {"/robot/sonar/head_sonar/lights/state", "std_msgs/UInt16"},
  {"/robot/sonar/head_sonar/sonars_enabled", "std_msgs/UInt16"},
  {"/robot/sonar/head_sonar/state", "sensor_msgs/PointCloud"},
  {"/robot/state", "baxter_core_msgs/AssemblyState"},
  {"/robustcontroller/left/CalibrateArm/status", "baxter_core_msgs/RobustControllerStatus"},
  {"/robustcontroller/left/Tare/status", "baxter_core_msgs/RobustControllerStatus"},
  {"/robustcontroller/left/rc_plugins_loaded", "std_msgs/Bool"},
  {"/robustcontroller/right/CalibrateArm/status", "baxter_core_msgs/RobustControllerStatus"},
  {"/robustcontroller/right/Tare/status", "baxter_core_msgs/RobustControllerStatus"},
  {"/robustcontroller/right/rc_plugins_loaded", "std_msgs/Bool"},
  {"/tf2_web_republisher/status", "actionlib_msgs/GoalStatusArray"},
  {"/update/progress", "std_msgs/Int32"},
  {"/update/status", "std_msgs/Int32"},
  {"/usb/ready", "std_msgs/Bool"}};

void Factory::createBridge_1to2(const std::string &topic, const std::string &msg)
{
  if(msg == "sensor_msgs/CameraInfo")
    bridges.push_back(std::make_unique<Bridge_1to2<sensor_msgs::CameraInfo, sensor_msgs::msg::CameraInfo>>(topic));
  else if(msg == "sensor_msgs/Image")
    bridges.push_back(std::make_unique<Bridge_1to2<sensor_msgs::Image, sensor_msgs::msg::Image>>(topic));
  else if(msg == "diagnostic_msgs/DiagnosticArray")
    bridges.push_back(std::make_unique<Bridge_1to2<diagnostic_msgs::DiagnosticArray, diagnostic_msgs::msg::DiagnosticArray>>(topic));
  else if(msg == "diagnostic_msgs/DiagnosticStatus")
    bridges.push_back(std::make_unique<Bridge_1to2<diagnostic_msgs::DiagnosticStatus, diagnostic_msgs::msg::DiagnosticStatus>>(topic));
  else if(msg == "std_msgs/UInt8MultiArray")
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::UInt8MultiArray, std_msgs::msg::UInt8MultiArray>>(topic));
  else if(msg == "sensor_msgs/Imu")
    bridges.push_back(std::make_unique<Bridge_1to2<sensor_msgs::Imu, sensor_msgs::msg::Imu>>(topic));
  else if(msg == "baxter_core_msgs/AnalogIOState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::AnalogIOState, baxter_core_msgs::msg::AnalogIOState>>(topic));
  else if(msg == "std_msgs/UInt32")
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::UInt32, std_msgs::msg::UInt32>>(topic));
  else if(msg == "baxter_core_msgs/AnalogIOStates")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::AnalogIOStates, baxter_core_msgs::msg::AnalogIOStates>>(topic));
  else if(msg == "baxter_core_msgs/AssemblyState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::AssemblyState, baxter_core_msgs::msg::AssemblyState>>(topic));
  else if(msg == "baxter_core_msgs/DigitalIOState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::DigitalIOState, baxter_core_msgs::msg::DigitalIOState>>(topic));
  else if(msg == "baxter_core_msgs/DigitalIOStates")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::DigitalIOStates, baxter_core_msgs::msg::DigitalIOStates>>(topic));
  else if(msg == "baxter_core_msgs/EndEffectorProperties")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::EndEffectorProperties, baxter_core_msgs::msg::EndEffectorProperties>>(topic));
  else if(msg == "baxter_core_msgs/EndEffectorState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::EndEffectorState, baxter_core_msgs::msg::EndEffectorState>>(topic));
  else if(msg == "baxter_core_msgs/HeadState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::HeadState, baxter_core_msgs::msg::HeadState>>(topic));
  else if(msg == "sensor_msgs/JointState")
    bridges.push_back(std::make_unique<Bridge_1to2<sensor_msgs::JointState, sensor_msgs::msg::JointState>>(topic));
  else if(msg == "baxter_core_msgs/CollisionAvoidanceState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::CollisionAvoidanceState, baxter_core_msgs::msg::CollisionAvoidanceState>>(topic));
  else if(msg == "baxter_core_msgs/CollisionDetectionState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::CollisionDetectionState, baxter_core_msgs::msg::CollisionDetectionState>>(topic));
  else if(msg == "baxter_core_msgs/EndpointState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::EndpointState, baxter_core_msgs::msg::EndpointState>>(topic));
  else if(msg == "baxter_core_msgs/SEAJointState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::SEAJointState, baxter_core_msgs::msg::SEAJointState>>(topic));
  else if(msg == "std_msgs/Empty")
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::Empty, std_msgs::msg::Empty>>(topic));
  else if(msg == "baxter_core_msgs/NavigatorState")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::NavigatorState, baxter_core_msgs::msg::NavigatorState>>(topic));
  else if(msg == "baxter_core_msgs/NavigatorStates")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::NavigatorStates, baxter_core_msgs::msg::NavigatorStates>>(topic));
  else if(msg == "sensor_msgs/Range")
    bridges.push_back(std::make_unique<Bridge_1to2<sensor_msgs::Range, sensor_msgs::msg::Range>>(topic));
  else if(msg == "std_msgs/Float32")
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::Float32, std_msgs::msg::Float32>>(topic));
  else if(msg == "std_msgs/UInt16")
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::UInt16, std_msgs::msg::UInt16>>(topic));
  else if(msg == "sensor_msgs/PointCloud")
    bridges.push_back(std::make_unique<Bridge_1to2<sensor_msgs::PointCloud, sensor_msgs::msg::PointCloud>>(topic));
  else if(msg == "baxter_core_msgs/RobustControllerStatus")
    bridges.push_back(std::make_unique<Bridge_1to2<baxter_core_msgs::RobustControllerStatus, baxter_core_msgs::msg::RobustControllerStatus>>(topic));
  else if(msg == "std_msgs/Bool")
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::Bool, std_msgs::msg::Bool>>(topic));
  else if(msg == "actionlib_msgs/GoalStatusArray")
    bridges.push_back(std::make_unique<Bridge_1to2<actionlib_msgs::GoalStatusArray, actionlib_msgs::msg::GoalStatusArray>>(topic));
  else if(msg == "std_msgs/Int32")
    bridges.push_back(std::make_unique<Bridge_1to2<std_msgs::Int32, std_msgs::msg::Int32>>(topic));
}
}