#include <baxter_bridge/monitor.h>
#include <sensor_msgs/Image.h>
#include <opencv2/imgproc.hpp>

constexpr double timeout_s{1.};

using namespace baxter_bridge;


Monitor::Monitor(std::string name, bool display) : name{name}, display{display}
{
  client = nh.serviceClient<baxter_bridge::BaxterPublishers>("/monitor", true);

  im_timer = nh.createTimer(ros::Duration(timeout_s/2),
                            [&](const ros::TimerEvent&)
  {
    if(im_pub.get())
      publishXDisplay();
  });
}

// local call to the monitor
bool Monitor::canPublishOn(const std::string &topic, bool test_client)
{
  if(server)
  {
    // I am the one monitor, call service locally
    parsePublishRequest(name, topic);
  }
  else if(client.exists())
  {
    // the one monitor is available outside
    BaxterPublishersRequest req;
    req.user = name;
    req.topic = topic;
    client.call(req, pub_state);
  }
  else if(test_client)
  {
    // try to re-establish connection
    if(!client.isValid())
      client = nh.serviceClient<baxter_bridge::BaxterPublishers>("/monitor", true);
    return canPublishOn(topic, false);
  }
  else
  {
    // no (more?) monitor, let it be me
    server = std::make_unique<ros::ServiceServer>(
               nh.advertiseService("/monitor", &Monitor::userCallback, this));
    if(display)
      im_pub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::Image>("/robot/xdisplay", 1));

    parsePublishRequest(name, topic);
  }

  if(pub_state.authorized_user == name)
    return true;

  std::cout << "Cannot publish on " << topic
            << ": already published by " << pub_state.authorized_user << std::endl;
  return false;
}

// all calls to the monitor
void Monitor::parsePublishRequest(const std::string &user, const std::string &topic)
{
  // check nobody has published recently
  auto prev{std::find_if(pub_state.publishers.begin(), pub_state.publishers.end(),
                         [topic](const auto &pub){return pub.topic == topic;})};
  const auto now_s{ros::Time::now().toSec()};

  //auto change{false};
  std::string user_authorized{user};

  if(prev == pub_state.publishers.end())
  {
    // new topic!
    //change = true;
    pub_state.publishers.push_back({});
    auto &last{pub_state.publishers.back()};
    last.topic = topic;
    last.user = user;
    last.time = now_s;
    pub_state.authorized_user = user;
  }
  else
  {
    if(prev->user == user)
    {
      // was the last publisher anyway / no change
      prev->time = now_s;
      pub_state.authorized_user = user;
    }
    else if(now_s - prev->time > timeout_s)
    {
      // previous user has not published for 1 sec
      prev->user = pub_state.authorized_user = user;
      prev->time = now_s;
      //change = true;
    }
    else
    {
      // previous user has just published
      pub_state.authorized_user = prev->user;
    }
  }
}

bool Monitor::userCallback(baxter_bridge::BaxterPublishersRequest &req,
                           baxter_bridge::BaxterPublishersResponse &res)
{
  std::cout << req.user << " wants to publish on " << req.topic << std::endl;
  parsePublishRequest(req.user, req.topic);
  res = pub_state;
  return true;
}

void Monitor::publishXDisplay()
{
  static const std::array<cv::Scalar, 2> colors{cv::Scalar{0,255,0}, cv::Scalar{255,120,80}};
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
  for(const auto &[topic, user, time]: pub_state.publishers)
  {
    const auto timeout{now_s-time > timeout_s};

    const auto text{topic.substr(7) + ": " + (timeout ? "available" : user)};
    cv::putText(im, text, {10, row},
                cv::FONT_HERSHEY_SIMPLEX, font_size, colors[timeout], 2, cv::LINE_AA);

    row += row_inc;
  }
  im_pub->publish(im_msg);
}
