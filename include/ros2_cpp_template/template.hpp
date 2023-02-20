// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#ifndef ROS2_CPP_TEMPLATE__TEMPLATE_HPP_
#define ROS2_CPP_TEMPLATE__TEMPLATE_HPP_

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace Ros2CppTemplate
{

class Template : public rclcpp::Node
{
public:
  explicit Template(const rclcpp::NodeOptions & options);

protected:
  void initPublisher();
  void initTimer();
  void on_timer();
  void setParam();
  void getParam();

private:
  size_t count_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string printf_string_;
  std::chrono::milliseconds printf_ms_;
};

}  // namespace Ros2CppTemplate

#endif  // ROS2_CPP_TEMPLATE__TEMPLATE_HPP_
