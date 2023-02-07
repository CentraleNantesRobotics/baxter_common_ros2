#ifndef BAXTER_BRIDGE_DISPLAY_H
#define BAXTER_BRIDGE_DISPLAY_H

#include <ros/ros.h>
#include <rclcpp/node.hpp>
#include <baxter_core_msgs/BridgePublishersAuth.h>
#include <baxter_core_msgs/BridgePublishersForce.h>

namespace baxter_bridge
{

constexpr auto AUTH_SRV{"/bridge_auth"};
constexpr auto FORCE_SRV{"/bridge_force"};

using namespace baxter_core_msgs;

struct Server
{
  ros::ServiceServer auth, force;

  template <class Monitor>
  inline explicit Server(Monitor *monitor)
    : auth{monitor->nh.advertiseService(AUTH_SRV, &Monitor::userCallback, monitor)},
      force{monitor->nh.advertiseService(FORCE_SRV, &Monitor::userCallback, monitor)}
  {}
};

struct Monitor
{
  friend struct Server;
  Monitor(const std::string &name, bool display);
  inline bool canPublishOn(const std::string &topic)
  {
    return canPublishOn(topic, true);
  }

  enum class Side{LEFT,RIGHT,NONE};

  static inline Side getSide(const std::string &topic)
  {
    if(topic.find("left") != topic.npos)
      return Side::LEFT;
    if(topic.find("right") != topic.npos)
      return Side::RIGHT;
    return Side::NONE;
  }

private:

  ros::NodeHandle nh;
  BridgePublishersAuth::Request publish_req;

  std::unique_ptr<Server> server;
  ros::ServiceClient client;
  BridgePublishersAuth::Response authorized_publishers;

  bool display{false};
  std::unique_ptr<ros::Publisher> im_pub;
  ros::Timer im_timer;
  void publishXDisplay();

  inline auto currentUser() const
  {
    return publish_req.user;
  }

  inline auto leftUser() const
  {
    return authorized_publishers.forced_left;
  }
  inline auto rightUser() const
  {
    return authorized_publishers.forced_right;
  }

  inline auto findPublisher(const std::string &topic)
  {
    return std::find_if(authorized_publishers.publishers.begin(), authorized_publishers.publishers.end(),
                        [&topic](const auto &pub){return pub.topic == topic;});
  }

  void parsePublishRequest(const std::string &user, const std::string &topic, const Side &side);
  bool userCallback(BridgePublishersAuth::Request &req,
                    BridgePublishersAuth::Response &res);

  bool forceCallback(BridgePublishersForce::Request &req,
                            [[maybe_unused]] BridgePublishersForceResponse &res);
  bool canPublishOn(const std::string &topic, bool test_client);

};
}


#endif // DISPLAY_H
