#ifndef BAXTER_BRIDGE_SOLVE_IK_H
#define BAXTER_BRIDGE_SOLVE_IK_H

// This bridge exposes the IK service in ROS 2

// SolveIK
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>

#include <baxter_bridge/factory.h>
#include <baxter_bridge/bridge.h>
#include <baxter_bridge/conversions.h>

namespace baxter_bridge
{

using SolveIK2 = baxter_core_msgs::srv::SolvePositionIK;
using SolveIK1 = baxter_core_msgs::SolvePositionIK;

template<>
void convert(const sensor_msgs::msg::JointState &src, sensor_msgs::JointState &dst);

template<>
void convert(const geometry_msgs::msg::PoseStamped &src, geometry_msgs::PoseStamped &dst);

template<>
void convert(const sensor_msgs::JointState &src, sensor_msgs::msg::JointState &dst);

struct SolveIK
{
  SolveIK(const std::string &side)
  {
    Factory::createBridge("/robot/limb/" + side + "/joint_command");

    const auto ik_srv{"/ExternalTools/" + side + "/PositionKinematicsNode/IKService"};
    // ros 1
    ik_client = Bridge::ros1()->serviceClient<SolveIK1>(ik_srv);

    // ros 2
    ik_service = Bridge::ros2()->create_service<SolveIK2>
                 (ik_srv, [&](SolveIK2::Request::SharedPtr req, SolveIK2::Response::SharedPtr res){solveIK(req,res);});
  }

private:

  // ros 1
  ros::ServiceClient ik_client;

  // ros 2
  rclcpp::Service<SolveIK2>::SharedPtr ik_service;

  void solveIK(SolveIK2::Request::SharedPtr req2, SolveIK2::Response::SharedPtr res2)
  {
    static SolveIK1::Request req1;
    static SolveIK1::Response res1;

    // convert request from 2 to 1
    req1.seed_mode = req2->seed_mode;
    convert(req2->pose_stamp, req1.pose_stamp);
    convert(req2->seed_angles, req1.seed_angles);

    // call ROS 1 solver
    ik_client.call(req1, res1);

    // remap to ROS 2 response
    convert(res1.joints, res2->joints);
    convert(res1.isValid, res2->is_valid);
    convert(res1.result_type, res2->result_type);
  }
};
}

#endif
