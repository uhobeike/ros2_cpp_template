// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_cpp_template/node/template_core.hpp"

using namespace std::chrono_literals;

namespace Ros2CppTemplate
{

TemplateNode::TemplateNode()
: Node("template_node"), count_(0)
{
  setParam();
  getParam();
  initPublisher();
  initTimer();
}

void TemplateNode::setParam()
{
  declare_parameter("printf_string", "これは、ROS 2のシンプルなテンプレート（多分）: ");
  declare_parameter("printf_hz", 1.0);
}

void TemplateNode::getParam()
{
  printf_string_ = get_parameter("printf_string").as_string();
  int printf_hz = 1000 / get_parameter("printf_hz").as_double();
  printf_ms_ = std::chrono::milliseconds{printf_hz};
}

void TemplateNode::initPublisher() {pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);}

void TemplateNode::initTimer()
{
  timer_ = create_wall_timer(printf_ms_, std::bind(&TemplateNode::on_timer, this));
}

void TemplateNode::on_timer()
{
  // ↓std_msgs::msg::String msg;
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = printf_string_ + std::to_string(++count_);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
  std::flush(std::cout);
}

}  // namespace Ros2CppTemplate
