#ifndef ROS2CPPTEMPLATE__TEMPLATE_HPP_
#define ROS2CPPTEMPLATE__TEMPLATE_HPP_

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

#endif  // COMPOSITION__TALKER_COMPONENT_HPP_
