cmake_minimum_required(VERSION 3.5)
project(baxter_core_msgs)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/RobustControllerStatus.msg"  
"msg/EndEffectorState.msg"  
"msg/HeadState.msg"  
"msg/AnalogOutputCommand.msg"  
"msg/URDFConfiguration.msg"  
"msg/DigitalIOStates.msg"  
"msg/NavigatorState.msg"  
"msg/JointCommand.msg"  
"msg/CameraControl.msg"  
"msg/DigitalOutputCommand.msg"  
"msg/NavigatorStates.msg"  
"msg/AssemblyStates.msg"  
"msg/EndEffectorCommand.msg"  
"msg/AssemblyState.msg"  
"msg/EndpointState.msg"  
"msg/CollisionDetectionState.msg"  
"msg/AnalogIOStates.msg"  
"msg/AnalogIOState.msg"  
"msg/EndEffectorProperties.msg"  
"msg/CollisionAvoidanceState.msg"  
"msg/HeadPanCommand.msg"  
"msg/SEAJointState.msg"  
"msg/CameraSettings.msg"  
"msg/DigitalIOState.msg"  
"msg/EndpointStates.msg"  
"srv/OpenCamera.srv"  
"srv/CloseCamera.srv"  
"srv/SolvePositionIK.srv"  
"srv/ListCameras.srv"
DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(FILES mapping_rules.yaml DESTINATION share/${PROJECT_NAME})

ament_package()
