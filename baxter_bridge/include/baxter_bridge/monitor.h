#ifndef BAXTER_BRIDGE_MONITOR_H
#define BAXTER_BRIDGE_MONITOR_H

#include <ros/ros.h>
#include <rclcpp/node.hpp>
#include <baxter_core_msgs/BridgePublishersAuth.h>
#include <baxter_core_msgs/BridgePublishersForce.h>

namespace baxter_bridge
{

constexpr auto AUTH_SRV{"/bridge_auth"};
constexpr auto FORCE_SRV{"/bridge_force"};

struct Monitor
{
  using BridgePublishersAuth = baxter_core_msgs::BridgePublishersAuth;
  using BridgePublishersForce = baxter_core_msgs::BridgePublishersForce;

  struct Server
  {
    ros::ServiceServer auth, force;
    ros::Timer im_timer;

    explicit Server(Monitor *monitor);
  };

  Monitor(const std::string &name, ros::NodeHandle* nh, bool init_server = false);
  inline bool canPublishOn(const std::string &topic)
  {
    return canPublishOn(topic, true);
  }

  enum class Side{LEFT,RIGHT,NONE};

private:

  ros::NodeHandle* nh {};
  BridgePublishersAuth::Request publish_req;

  std::unique_ptr<Server> server;
  ros::ServiceClient client;
  BridgePublishersAuth::Response authorized_publishers;

  void publishXDisplay();

  inline auto currentUser() const
  {
    return publish_req.user;
  }

  inline std::string reservedUser(Side side) const
  {
    switch(side)
    {
      case Side::LEFT:
        return authorized_publishers.forced_left;
      case Side::RIGHT:
        return authorized_publishers.forced_right;
      default:
        return "";
    }
  }

  inline auto findPublisher(const std::string &topic)
  {
    return std::find_if(authorized_publishers.publishers.begin(), authorized_publishers.publishers.end(),
                        [&topic](const auto &pub){return pub.topic == topic;});
  }

  void updateAuthorizedPublishers(const std::string &user, const std::string &topic, const Side &side);
  bool userCallback(BridgePublishersAuth::Request &req,
                    BridgePublishersAuth::Response &res);

  bool forceCallback(BridgePublishersForce::Request &req,
                     [[maybe_unused]] BridgePublishersForce::Response &res);
  bool canPublishOn(const std::string &topic, bool test_client);

};
}


#endif // DISPLAY_H
