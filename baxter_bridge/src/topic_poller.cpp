#include <baxter_bridge/topic_poller.h>
#include <baxter_bridge/factory.h>

namespace baxter_bridge
{

TopicPoller::TopicPoller(rclcpp::Node* node2) : node2{node2}
{
  poller = node2->create_wall_timer(std::chrono::seconds(1), [&](){poll();});
}

void TopicPoller::poll()
{
  const auto info1{[]()
                     {ros::master::V_TopicInfo info;
      ros::master::getTopics(info);return info;}()};

  const auto info2{node2->get_topic_names_and_types()};

  ros1_published.clear();
  ros1_subscribed.clear();
  ros2_published.clear();
  ros2_subscribed.clear();

  // only register topics that have not been bridged
  for(const auto &info: info1)
  {
    const auto &topic{info.name};
    if(Factory::isPublishedByBaxter(topic))
      ros1_published.push_back(topic);
    else if(Factory::isSubscribedByBaxter(topic))
      ros1_subscribed.push_back(topic);
  }

  for(const auto &[topic, msg]: info2)
  {
    if(Factory::isPublishedByBaxter(topic))
      ros2_subscribed.push_back(topic);
    else if(Factory::isSubscribedByBaxter(topic))
      ros2_published.push_back(topic);
  }

  std::sort(ros1_published.begin(), ros1_published.end());
  std::sort(ros1_subscribed.begin(), ros1_subscribed.end());
  std::sort(ros2_published.begin(), ros2_published.end());
  std::sort(ros2_subscribed.begin(), ros2_subscribed.end());

}

std::vector<std::string> TopicPoller::pendingBridges() const
{
  std::vector<std::string> pending;

  std::set_intersection(ros1_published.begin(),ros1_published.end(),
                            ros2_subscribed.begin(),ros2_subscribed.end(),
                            back_inserter(pending));
  std::set_intersection(ros2_published.begin(),ros2_published.end(),
                            ros1_subscribed.begin(),ros1_subscribed.end(),
                            back_inserter(pending));
  return pending;
}
}
