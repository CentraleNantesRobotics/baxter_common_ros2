#include <baxter_bridge/solve_ik.h>
#include <baxter_bridge/bridge_1to2.h>
#include <baxter_bridge/topic_poller.h>
#include <rclcpp/rclcpp.hpp>

using namespace baxter_bridge;

int main(int argc, char** argv)
{
  Bridge::init(argc, argv);

  std::unique_ptr<TopicPoller> poller;

  if(!Bridge::isStatic())
    poller = std::make_unique<TopicPoller>(Bridge::ros2());

  SolveIK left("left"), right("right");

  Factory::createBridge("/robot/joint_states");

  if(Bridge::onBaxter())
  {
    // other bridges we are usually interested in


    if(Bridge::isStatic())
      Factory::createRemainingBridges();
  }

  ros::AsyncSpinner async(1);
  async.start();

  while(ros::ok() && rclcpp::ok())
  {
    Bridge::spinOnce();

    // create new bridges if needed
    if(poller)
    {
      const auto pending{poller->pendingBridges()};
      for(const auto &topic: pending)
        Factory::createBridge(topic);
    }
  }


}
