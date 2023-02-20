// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: GPL-3.0

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_cpp_template/template.hpp"

using namespace std::chrono_literals;

namespace Ros2CppTemplate
{

Template::Template(const rclcpp::NodeOptions & options)
: Node("template", options), count_(0)
{
  setParam();
  getParam();
  initPublisher();
  initTimer();
}

void Template::setParam()
{
  declare_parameter("printf_string", "これは、ROS 2のシンプルなテンプレート（多分）: ");
  declare_parameter("printf_hz", 1.0);
}

void Template::getParam()
{
  printf_string_ = get_parameter("printf_string").as_string();
  int printf_hz = 1000 / get_parameter("printf_hz").as_double();
  printf_ms_ = std::chrono::milliseconds{printf_hz};
}

void Template::initPublisher() {pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);}

void Template::initTimer()
{
  timer_ = create_wall_timer(printf_ms_, std::bind(&Template::on_timer, this));
}

void Template::on_timer()
{
  // ↓std_msgs::msg::String msg;
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = printf_string_ + std::to_string(++count_);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
  std::flush(std::cout);
}

}  // namespace Ros2CppTemplate

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(Ros2CppTemplate::Template)
