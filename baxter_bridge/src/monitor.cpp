#include <baxter_bridge/monitor.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc.hpp>

constexpr double timeout_s{1.};

using namespace baxter_bridge;

namespace
{
static inline auto getSide(const std::string &topic)
{
  if(topic.find("left") != topic.npos)
    return Monitor::Side::LEFT;
  if(topic.find("right") != topic.npos)
    return Monitor::Side::RIGHT;
  return Monitor::Side::NONE;
}

void eraseSideTopics(baxter_core_msgs::BridgePublishersAuth::Response::_publishers_type &publishers,
                     const Monitor::Side &side)
{
  using baxter_core_msgs::BridgePublisher;
  publishers.erase(std::remove_if(publishers.begin(), publishers.end(), [&](const BridgePublisher &pub)
  {return getSide(pub.topic) == side;}), publishers.end());
}
}

Monitor::Server::Server(Monitor *monitor)
  : auth{monitor->nh->advertiseService(AUTH_SRV, &Monitor::userCallback, monitor)},
    force{monitor->nh->advertiseService(FORCE_SRV, &Monitor::forceCallback, monitor)}
{
  const auto display{monitor->nh->param<bool>("forward_display", false)};
  if(display)
  {
    im_timer = monitor->nh->createTimer(ros::Duration(timeout_s/2),
                               [&](const ros::TimerEvent&)
    {
      monitor->publishXDisplay();
    });
  }
}


Monitor::Monitor(const std::string &name, ros::NodeHandle *nh, bool init_server) : nh{nh}
{
  publish_req.user = name;
  client = nh->serviceClient<BridgePublishersAuth>(AUTH_SRV, true);

  if(init_server)
    server = std::make_unique<Server>(this);
}

// local call to the monitor
bool Monitor::canPublishOn(const std::string &topic, bool test_client)
{
  const auto side{getSide(topic)};
  if(server)
  {
    // I am the one monitor, call service locally
    updateAuthorizedPublishers(currentUser(), topic, side);
  }
  else if(client.exists())
  {
    // the one monitor is available outside
    publish_req.topic = topic;
    client.call(publish_req, authorized_publishers);
  }
  else if(test_client)
  {
    // try to re-establish connection
    if(!client.isValid())
      client = nh->serviceClient<BridgePublishersAuth>(AUTH_SRV, true);
    return canPublishOn(topic, false);
  }
  else
  {
    // no (more?) auth server, let it be me
    server = std::make_unique<Server>(this);

    updateAuthorizedPublishers(currentUser(), topic, side);
  }

  if(const auto user{reservedUser(side)}; !user.empty())
    return user == currentUser();

  const auto authorized{findPublisher(topic)};

  if(authorized->user == currentUser())
    return true;

  std::cout << "Cannot publish on " << topic
            << ": already published by " << authorized->user << std::endl;
  return false;
}

// all calls to the monitor
void Monitor::updateAuthorizedPublishers(const std::string &user, const std::string &topic, const Side &side)
{
  // first deal with forced users
  if(!reservedUser(side).empty())
    return;

  // find it by hand
  auto prev{findPublisher(topic)};
  const auto now_s{ros::Time::now().toSec()};

  // check nobody has published recently
  if(prev == authorized_publishers.publishers.end())
  {
    // new topic!
    auto &last{authorized_publishers.publishers.emplace_back()};
    last.topic = topic;
    last.user = user;
    last.time = now_s;
  }
  else
  {
    if(prev->user == user)
    {
      // was the last publisher anyway / no change
      prev->time = now_s;
    }
    else if(now_s - prev->time > timeout_s)
    {
      // previous user has not published for 1 sec
      prev->user = user;
      prev->time = now_s;
    }
  }
}

bool Monitor::userCallback(BridgePublishersAuth::Request &req,
                           BridgePublishersAuth::Response &res)
{
  if(!req.user.empty() && !req.topic.empty())
  {
    std::cout << req.user << " wants to publish on " << req.topic << std::endl;
    updateAuthorizedPublishers(req.user, req.topic, getSide(req.topic));
  }
  res = authorized_publishers;
  return true;
}

bool Monitor::forceCallback(BridgePublishersForce::Request &req,
                            [[maybe_unused]] BridgePublishersForce::Response &res)
{
  authorized_publishers.forced_left = req.left_user;
  authorized_publishers.forced_right = req.right_user;

  if(!req.left_user.empty())
    eraseSideTopics(authorized_publishers.publishers, Side::LEFT);
  if(!req.right_user.empty())
    eraseSideTopics(authorized_publishers.publishers, Side::RIGHT);
  return true;
}

void Monitor::publishXDisplay()
{
  static const std::array<cv::Scalar, 2> colors{cv::Scalar{0,255,0}, cv::Scalar{255,120,80}};
  static auto im_pub = nh->advertise<sensor_msgs::Image>("/robot/xdisplay", 1);

  static sensor_msgs::Image im_msg = []()
  {
    sensor_msgs::Image msg;
    msg.height = 600;
    msg.width = 1024;
    msg.step = msg.width*3;
    msg.encoding = "bgr8";
    msg.is_bigendian = true;
    msg.data.resize(msg.height * msg.step);
    return msg;
  }();

  // cv::Mat sharing the same memory as im_msg, better than ros1 cv_bridge
  cv::Mat im = [](sensor_msgs::Image &msg)
  {
    cv::Mat im_sized(msg.height, msg.width, CV_8UC3);
    cv::Mat im = im_sized;  // copy size but shares memory
    im_sized.release();
    im.data = msg.data.data();
    return im;
  }(im_msg);

  // all black
  im.setTo(0);

  // should probably adapt to publishers size
  constexpr auto row_inc{50};
  constexpr auto font_size{1.4};

  auto row{40};
  const auto now_s = ros::Time::now().toSec();
  for(const auto &[topic, user, time]: authorized_publishers.publishers)
  {
    const auto timeout{now_s-time > timeout_s};

    const auto text{topic.substr(7) + ": " + (timeout ? "available" : user)};
    cv::putText(im, text, {10, row},
                cv::FONT_HERSHEY_SIMPLEX, font_size, colors[timeout], 2, cv::LINE_AA);

    row += row_inc;
  }
  im_pub.publish(im_msg);
}
