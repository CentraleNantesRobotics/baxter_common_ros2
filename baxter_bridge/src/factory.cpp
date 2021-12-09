#include <baxter_bridge/factory.h>
#include <baxter_bridge/bridge.h>
#include <algorithm>


namespace baxter_bridge
{

std::vector<std::unique_ptr<Bridge>> Factory::bridges;

void Factory::createRemainingBridges()
{
  // work on backups as createBridge changes topics_itoj
  const auto remaining_1to2{topics_1to2};
  const auto remaining_2to1{topics_2to1};

  for(const auto &[topic,msg]: remaining_1to2)
    createBridge_1to2(topic, msg);

  for(const auto &[topic,msg]: remaining_2to1)
    createBridge_2to1(topic, msg);
}

void Factory::createBridge(const std::string &topic)
{
  if(const auto bridge = topics_1to2.find(topic);bridge != topics_1to2.end())
  {
    const auto msg{topics_1to2[topic]};
    topics_1to2.erase(bridge);
    std::cout << "Creating bridge 1->2 " + topic << std::endl;
    createBridge_1to2(topic, msg);
  }
  else if(const auto bridge = topics_2to1.find(topic);bridge != topics_2to1.end())
  {
    const auto msg{topics_2to1[topic]};
    topics_2to1.erase(bridge);
    std::cout << "Creating bridge 2->1 " + topic << std::endl;
    createBridge_2to1(topic, msg);
  }
}
}
