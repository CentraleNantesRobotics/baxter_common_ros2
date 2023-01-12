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
find_package(robot_state_publisher REQUIRED)
find_package(actionlib_msgs REQUIRED)

find_package(OpenCV REQUIRED)

set(CMAKE_BUILD_TYPE Release)

# find ROS 1 packages we depend on
set(ROS1_ROOT "/opt/ros/noetic")
set(ROS1_LIBS roscpp rosconsole roscpp_serialization rostime xmlrpcpp)

# if libraries in ROS1_ROOT, explicitely give their location
if(EXISTS ${ROS1_ROOT})
    foreach(ROS1_LIB ${ROS1_LIBS})
        add_library(${ROS1_LIB} UNKNOWN IMPORTED)
        set_property(TARGET ${ROS1_LIB} PROPERTY IMPORTED_LOCATION "${ROS1_ROOT}/lib")
    endforeach()
endif()

add_executable(baxter_bridge    src/baxter_bridge.cpp
                                src/bridge.cpp
                                src/monitor.cpp
                                src/factory.cpp
                                src/factory_1to2.cpp
                                src/factory_2to1.cpp
                                src/topic_poller.cpp)
target_include_directories(baxter_bridge PRIVATE
                            ${ROS1_ROOT}/include
                            include include/ros1_msgs
                            ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

target_link_libraries(baxter_bridge ${ROS1_LIBS} ${OpenCV_LIBS})

ament_target_dependencies(baxter_bridge
rclcpp sensor_msgs baxter_core_msgs baxter_maintenance_msgs diagnostic_msgs trajectory_msgs robot_state_publisher actionlib_msgs)

install(TARGETS baxter_bridge
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
