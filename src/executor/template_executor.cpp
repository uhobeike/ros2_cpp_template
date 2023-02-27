// SPDX-FileCopyrightText: 2023 Tatsuhiro Ikebe <beike315@icloud.com>
// SPDX-License-Identifier: Apache-2.0

#include "ros2_cpp_template/executor/template_core.hpp"
#include "ros2_cpp_template/executor/template_composition.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node1 = std::make_shared<Ros2CppTemplate::TemplateNode>();
  auto node2 = std::make_shared<Ros2CppTemplate::TemplateComposition>(rclcpp::NodeOptions());

  node1->declare_parameter("node1/printf_string", "俺はnode1と申す！：");
  node1->declare_parameter("node1/namespace", "node1/");
  node1->declare_parameter("node1/printf_hz", 1.0);
  node1->set_parameter(rclcpp::Parameter("namespace", "node1/"));

  node2->declare_parameter("node2/printf_string", "私はnode2よ♡ ：");
  node2->declare_parameter("node2/namespace", "node2/");
  node2->declare_parameter("node2/printf_hz", 2.0);
  node2->set_parameter(rclcpp::Parameter("namespace", "node2/"));

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node1);
  executor.add_node(node2);

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
