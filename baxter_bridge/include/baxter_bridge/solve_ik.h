#ifndef BAXTER_BRIDGE_SOLVE_IK_H
#define BAXTER_BRIDGE_ARM_RELAY_H

// This bridge exposes the IK service in ROS 2
// it also bridges joint commands and range / images

// SolveIK
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/srv/solve_position_ik.hpp>

#include <baxter_bridge/factory.h>
#include <baxter_bridge/bridge.h>

namespace baxter_bridge
{

using SolveIK2 = baxter_core_msgs::srv::SolvePositionIK;
using SolveIK1 = baxter_core_msgs::SolvePositionIK;

struct SolveIK : public Bridge
{
  SolveIK(const std::string &side)
  {
    Factory::createBridge("/robot/limb/" + side + "/joint_command");

    const auto ik_srv{"/ExternalTools/" + side + "/PositionKinematicsNode/IKService"};
    // ros 1
    ik_client = ros1()->serviceClient<SolveIK1>(ik_srv);

    // ros 2
    ik_service = ros2()->create_service<SolveIK2>
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

    req1.seed_mode = req2->seed_mode;

    // transfer poses
    const auto dim{req2->pose_stamp.size()};

    req1.pose_stamp.resize(dim);
    for(size_t i = 0; i < dim; ++i)
    {
      auto &pose1{req1.pose_stamp[i]};
      auto &pose2{req2->pose_stamp[i]};
      pose1.pose.position.x = pose2.pose.position.x;
      pose1.pose.position.y = pose2.pose.position.y;
      pose1.pose.position.z = pose2.pose.position.z;
      pose1.pose.orientation.x = pose2.pose.orientation.x;
      pose1.pose.orientation.y = pose2.pose.orientation.y;
      pose1.pose.orientation.z = pose2.pose.orientation.z;
      pose1.pose.orientation.w = pose2.pose.orientation.w;
      pose1.header.frame_id = pose2.header.frame_id;
      pose1.header.stamp = ros::Time::now();
    }

    // transfer seed
    req1.seed_angles.resize(req2->seed_angles.size());
    for(size_t i = 0; i < req2->seed_angles.size(); ++i)
    {
      auto &seed1{req1.seed_angles[i]};
      auto &seed2{req2->seed_angles[i]};
      seed1.name = seed2.name;
      seed1.position = seed2.position;
    }

    // call ROS 1 solver
    ik_client.call(req1, res1);

    // remap to ROS 2 response
    res2->joints.resize(dim);
    res2->is_valid.resize(dim);
    res2->result_type.resize(dim);
    for(size_t i = 0; i < dim; ++i)
    {
      auto &js1{res1.joints[i]};
      auto &js2{res2->joints[i]};

      js2.name = js1.name;
      js2.position = js1.position;
      res2->is_valid[i] = res1.isValid[i];
      //res2->result_type[i] = res1.result_type[i];
    }
  }
};
}

#endif
