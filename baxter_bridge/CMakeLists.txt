cmake_minimum_required(VERSION 3.5)
project(baxter_bridge)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

option(SAFE_CMD "Ensure JointCommand is limited to Position or Velocity" TRUE)

if(SAFE_CMD)
  add_definitions(-DBAXTER_BRIDGE_SAFE_CMD)
endif()

# ROS 2 pkgs
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Service files
rosidl_generate_interfaces(${PROJECT_NAME}
    "srv/Open.srv"
    "srv/Exists.srv")
ament_export_dependencies(rosidl_default_runtime)

# use custom services as node dependencies
if("$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "foxy")
    set(LEGACY_IDL TRUE)
else()
    set(LEGACY_IDL FALSE)
endif()
if(NOT ${LEGACY_IDL})
    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
endif()

find_package(OpenCV REQUIRED)

# find ROS 1 packages we depend on
set(ROS1_ROOT "/opt/ros/noetic")
set(ROS1_LIBS roscpp rosconsole roscpp_serialization rostime xmlrpcpp)

# if libraries in ROS1_ROOT, explicitely give their location
if(EXISTS ${ROS1_ROOT})
    foreach(ROS1_LIB ${ROS1_LIBS})
        add_library(${ROS1_LIB} UNKNOWN IMPORTED)
        set_property(TARGET ${ROS1_LIB} PROPERTY IMPORTED_LOCATION "${ROS1_ROOT}/lib/lib${ROS1_LIB}.so")
    endforeach()
endif()

ament_auto_add_executable(bridge src/baxter_bridge.cpp
                                src/bridge.cpp
                                src/monitor.cpp
                                src/factory.cpp
                                src/factory_1to2.cpp
                                src/factory_2to1.cpp
                                src/topic_poller.cpp)                                    
target_include_directories(bridge PRIVATE
                            include
                            ros1_msgs
                            ${ROS1_ROOT}/include
                            ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(bridge ${ROS1_LIBS} ${OpenCV_LIBS})
if(${LEGACY_IDL})
    rosidl_target_interfaces(bridge ${PROJECT_NAME} "rosidl_typesupport_cpp")
else()
    target_link_libraries(bridge "${cpp_typesupport_target}")
endif()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_auto_package()
