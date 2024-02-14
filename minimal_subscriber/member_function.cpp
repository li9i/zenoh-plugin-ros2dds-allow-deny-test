// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <functional>
#include <memory>
#include <fstream>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/image.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    subscription_denied_ = this->create_subscription<sensor_msgs::msg::Image>(
      "topic_denied", 10, std::bind(&MinimalSubscriber::topic_callback_denied, this, _1));
  }

private:
  void topic_callback(const sensor_msgs::msg::Image& msg)
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d.%d'", msg.header.stamp.sec,
      msg.header.stamp.nanosec);
  }
  void topic_callback_denied(const sensor_msgs::msg::Image& msg)
  {
    RCLCPP_WARN(this->get_logger(), "I RECEIVED MSG ON THE DENIED TOPIC");
  }
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_denied_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
