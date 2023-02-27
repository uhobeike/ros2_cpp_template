// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <iostream>
#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_cpp_template/composition/template_composition.hpp"

using namespace std::chrono_literals;

namespace Ros2CppTemplate
{

TemplateComposition::TemplateComposition(const rclcpp::NodeOptions & options)
: Node("template_composition", options), count_(0)
{
  setParam();
  getParam();
  initPublisher();
  initTimer();
}

void TemplateComposition::setParam()
{
  declare_parameter("namespace", "");
  declare_parameter("printf_string", "これは、ROS 2のシンプルなテンプレート（多分）: ");
  declare_parameter("printf_hz", 1.0);
}

void TemplateComposition::getParam()
{
  namespace_ = get_parameter("namespace").as_string();
  auto local_namespace = get_parameter(namespace_ + "namespace").as_string();
  printf_string_ = get_parameter(local_namespace + "printf_string").as_string();
  int printf_hz = 1000 / get_parameter(local_namespace + "printf_hz").as_double();
  printf_ms_ = std::chrono::milliseconds{printf_hz};
}

void TemplateComposition::initPublisher()
{
  pub_ = create_publisher<std_msgs::msg::String>("chatter_composition", 10);
}

void TemplateComposition::initTimer()
{
  timer_ = create_wall_timer(printf_ms_, std::bind(&TemplateComposition::on_timer, this));
}

void TemplateComposition::on_timer()
{
  // ↓std_msgs::msg::String msg;
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = printf_string_ + std::to_string(++count_);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg->data.c_str());
  std::flush(std::cout);

  static std::chrono::milliseconds time_memo = 0ms;
  time_memo = printf_ms_;
  getParam();
  if(time_memo != printf_ms_)
    timer_ = create_wall_timer(printf_ms_, std::bind(&TemplateComposition::on_timer, this));
}

}  // namespace Ros2CppTemplate

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(Ros2CppTemplate::TemplateComposition)
