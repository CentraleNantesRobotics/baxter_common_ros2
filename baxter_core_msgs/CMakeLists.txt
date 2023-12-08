cmake_minimum_required(VERSION 3.5)
project(baxter_core_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
"msg/EndEffectorState.msg"  
"msg/EndEffectorProperties.msg"  
"msg/DigitalIOStates.msg"  
"msg/EndEffectorCommand.msg"  
"msg/NavigatorState.msg"  
"msg/AnalogOutputCommand.msg"  
"msg/EndpointState.msg"  
"msg/CollisionDetectionState.msg"  
"msg/SEAJointState.msg"  
"msg/CollisionAvoidanceState.msg"  
"msg/URDFConfiguration.msg"  
"msg/CameraControl.msg"  
"msg/NavigatorStates.msg"  
"msg/AnalogIOState.msg"  
"msg/CameraSettings.msg"  
"msg/JointCommand.msg"  
"msg/DigitalIOState.msg"  
"msg/RobustControllerStatus.msg"  
"msg/AssemblyStates.msg"  
"msg/DigitalOutputCommand.msg"  
"msg/HeadPanCommand.msg"  
"msg/AssemblyState.msg"  
"msg/HeadState.msg"  
"msg/AnalogIOStates.msg"  
"msg/BridgePublisher.msg"  
"msg/EndpointStates.msg"  
"srv/ListCameras.srv"  
"srv/CloseCamera.srv"  
"srv/OpenCamera.srv"  
"srv/BridgePublishersForce.srv"  
"srv/BridgePublishersAuth.srv"  
"srv/SolvePositionIK.srv"
DEPENDENCIES builtin_interfaces geometry_msgs std_msgs sensor_msgs)

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
