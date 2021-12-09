#ifndef BAXTER_BRIDGE_DISPLAY_H
#define BAXTER_BRIDGE_DISPLAY_H

#include <ros/ros.h>
#include <rclcpp/node.hpp>
#include <baxter_bridge/BaxterPublishers.h>

namespace baxter_bridge
{

struct Monitor
{
  Monitor(std::string name, bool display);
  inline bool canPublishOn(const std::string &topic)
  {
    return canPublishOn(topic, true);
  }

private:

  ros::NodeHandle nh;
  std::string name;  

  ros::ServiceClient client;
  std::unique_ptr<ros::ServiceServer> server;
  BaxterPublishersResponse pub_state;

  bool display{false};
  std::unique_ptr<ros::Publisher> im_pub;
  void publishXDisplay(const double now_s);

  void parsePublishRequest(const std::string &user, const std::string &topic);
  bool userCallback(BaxterPublishersRequest &req,
                    BaxterPublishersResponse &res);

  bool canPublishOn(const std::string &topic, bool test_client);

};
}


#endif // DISPLAY_H
