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

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0), count_denied_(), max_count_(250)
  {
    unsigned int hz = 10;
    double dt = 1.0 / hz;
    unsigned int ms = static_cast<unsigned int>(1000*dt);

    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("topic", max_count_);
    publisher_denied_ = this->create_publisher<sensor_msgs::msg::Image>("topic_denied", max_count_);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(ms), std::bind(&MinimalPublisher::timer_callback, this));
    timer_denied_ = this->create_wall_timer(
      std::chrono::milliseconds(ms), std::bind(&MinimalPublisher::timer_callback_denied, this));

    // Construct the message to be published once to avoid extra delays
    img_ = sensor_msgs::msg::Image();

    // size(uint8) x nrows x ncols = 1 x 1280 x 960 = 1.23 MB
    // 1.23 MB @ 25 Hz = 307 MB in 10 sec
    unsigned int nrows = 1280;
    unsigned int ncols = 960;

    for (unsigned int row = 0; row < nrows; row++)
      for (unsigned int col = 0; col < ncols; col++)
        img_.data.push_back(row*ncols + col);
  }

private:
  void timer_callback()
  {
    //if (publisher_->get_subscription_count() == 0)
      //return;

    img_.step = count_;
    img_.header.stamp = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d.%d'",
      img_.header.stamp.sec, img_.header.stamp.nanosec);
    publisher_->publish(img_);
    count_++;

    if (count_ > max_count_)
    {
      timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "Published all that was to be published");
    }
  }
  void timer_callback_denied()
  {
    //if (publisher_denied_->get_subscription_count() == 0)
      //return;

    //img_.step = count_denied_;
    //img_.header.stamp = this->get_clock()->now();
    RCLCPP_WARN(this->get_logger(), "PUBLISHING ON DENIED TOPIC");
    publisher_denied_->publish(img_);
    count_denied_++;

    if (count_denied_ > max_count_)
    {
      timer_denied_->cancel();
      RCLCPP_WARN(this->get_logger(), "PUBLISHED ALL THAT WAS TO BE PUBLISHED");
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_denied_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_denied_;
  sensor_msgs::msg::Image img_;
  size_t count_;
  size_t count_denied_;
  size_t max_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
