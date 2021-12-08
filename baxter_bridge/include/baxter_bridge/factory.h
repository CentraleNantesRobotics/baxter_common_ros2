#ifndef BAXTER_BRIDGE_FACTORY_H
#define BAXTER_BRIDGE_FACTORY_H

#include <string>
#include <memory>
#include <map>
#include <vector>

namespace baxter_bridge
{

class Bridge;

class Factory
{
public:
  static void createRemainingBridges();

  static void createBridge(const std::string &topic);

  inline static bool isSubscribedByBaxter(const std::string &topic)
  {
    return topics_2to1.find(topic) != topics_2to1.end();
  }
  inline static bool isPublishedByBaxter(const std::string &topic)
  {
    return topics_1to2.find(topic) != topics_1to2.end();
  }

private:

  static std::map<std::string, std::string> topics_1to2;
  static std::map<std::string, std::string> topics_2to1;
  static std::vector<std::unique_ptr<Bridge>> bridges;

  static void createBridge_1to2(const std::string &topic, const std::string &msg);
  static void createBridge_2to1(const std::string &topic, const std::string &msg);
};

}

#endif
