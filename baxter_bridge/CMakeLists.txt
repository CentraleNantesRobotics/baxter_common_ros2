cmake_minimum_required(VERSION 3.5)
project(baxter_bridge)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 pkgs
find_package(ament_cmake REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baxter_core_msgs REQUIRED)
find_package(baxter_maintenance_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE Release)

# find ROS 1 packages we depend on
set(ROS1_ROOT "/opt/ros/noetic")
#set(ROS1_BAXTER_WS "/opt/local_ws/ros1" CACHE STRING "Path to ROS 1 workspace where baxter_core_msgs is placed")
set(ROS1_BAXTER_WS "/home/olivier/code/libs/ros" CACHE STRING "Path to ROS 1 workspace where baxter_core_msgs is placed")

set(ROS1_INCLUDES "${ROS1_ROOT}/include;${ROS1_BAXTER_WS}/install/include;${ROS1_BAXTER_WS}/devel/include")
set(ROS1_LIBS roscpp rosconsole roscpp_serialization rostime)

add_executable(baxter_bridge    src/baxter_bridge.cpp 
                                src/bridge.cpp 
                                src/monitor.cpp
                                src/factory.cpp
                                src/factory_1to2.cpp
                                src/factory_2to1.cpp
                                src/topic_poller.cpp)
target_include_directories(baxter_bridge PRIVATE ${ROS1_INCLUDES} include ${OpenCV_INCLUDE_DIRS})
target_link_directories(baxter_bridge PUBLIC ${ROS1_ROOT}/lib)
target_link_libraries(baxter_bridge ${ROS1_LIBS} ${OpenCV_LIBS})
ament_target_dependencies(baxter_bridge rclcpp sensor_msgs baxter_core_msgs baxter_maintenance_msgs diagnostic_msgs trajectory_msgs)

install(TARGETS baxter_bridge
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()