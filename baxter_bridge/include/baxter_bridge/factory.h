#ifndef BAXTER_BRIDGE_FACTORY_H
#define BAXTER_BRIDGE_FACTORY_H

#include <string>
#include <memory>
#include <map>
#include <vector>
#include <baxter_bridge/bridge.h>

namespace baxter_bridge
{

class Factory
{
public:

  static void createRemainingBridges();

  static bool createBridge(const std::string &topic, const std::string &msg = "");

  inline static bool isSubscribedByBaxter(const std::string &topic)
  {
    return topics_2to1.find(topic) != topics_2to1.end();
  }
  inline static bool isPublishedByBaxter(const std::string &topic)
  {
    return topics_1to2.find(topic) != topics_1to2.end();
  }

  static std::optional<Bridge::Direction> exists(const std::string &topic);

private:

  static std::map<std::string, std::string> topics_1to2;
  static std::map<std::string, std::string> topics_2to1;
  static inline std::vector<std::unique_ptr<Bridge>> bridges;

  static void createBridge_1to2(const std::string &topic, const std::string &msg);
  static void createBridge_2to1(const std::string &topic, const std::string &msg);
};

}

#endif
