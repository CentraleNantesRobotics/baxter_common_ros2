#include <baxter_bridge/factory.h>
#include <baxter_bridge/bridge.h>
#include <algorithm>


namespace baxter_bridge
{

void Factory::createRemainingBridges()
{
  // work on backups as createBridge erases topics_*to*
  const auto remaining{[](){
      std::vector<std::string> out;
      for(const auto &rem: {topics_1to2, topics_2to1})
        std::transform(rem.begin(), rem.end(), std::back_inserter(out),
                       [](const auto &elem){return elem.first;});
      return out;
                       }()};

  for(const auto &topic: remaining)
    createBridge(topic);
}

std::optional<Bridge::Direction> Factory::exists(const std::string &topic)
{
  const auto bridge = std::find_if(bridges.begin(), bridges.end(),
                                   [&](auto &bridge){return bridge->topic() == topic;});

  if(bridge == bridges.end())
    return std::nullopt;

  return (*bridge)->direction();
}

bool Factory::createBridge(const std::string &topic, const std::string &msg)
{
  // known Baxter topics are checked first
  if(const auto bridge = topics_1to2.find(topic);bridge != topics_1to2.end())
  {
    const auto msg{bridge->second};
    topics_1to2.erase(bridge);
    RCLCPP_INFO(Bridge::ros2()->get_logger(), "Creating bridge 1->2 %s", topic.c_str());
    createBridge_1to2(topic, msg);
    return true;
  }

  if(const auto bridge = topics_2to1.find(topic);bridge != topics_2to1.end())
  {
    const auto msg{bridge->second};
    topics_2to1.erase(bridge);
    RCLCPP_INFO(Bridge::ros2()->get_logger(), "Creating bridge 2->1 %s", topic.c_str());
    createBridge_2to1(topic, msg);
    return true;
  }

  // we might open any bridge as long as it does not exist yet
  if(const auto bridge = std::find_if(bridges.begin(), bridges.end(),
                                      [&](auto &bridge){return bridge->topic() == topic;});
     bridge != bridges.end())
    return false;

  const auto prev_bridges_count{bridges.size()};
  createBridge_1to2(topic, msg);
  return bridges.size() != prev_bridges_count;
}
}
