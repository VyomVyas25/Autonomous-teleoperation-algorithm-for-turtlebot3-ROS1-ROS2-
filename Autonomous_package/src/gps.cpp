#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using namespace std;


class rover_pubsub : public rclcpp::Node
{
public:
  rover_pubsub() : Node("gps_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/fix", 10, std::bind(&rover_pubsub::topic_callback, this, std::placeholders::_1));

  }

private:
  void topic_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    auto x_g = msg->latitude;
    auto y_g = msg->longitude;
    RCLCPP_INFO(this->get_logger(), "lat : %f, long : %f", x_g, y_g);  
  }

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rover_pubsub>());
  rclcpp::shutdown();
  return 0;
}
