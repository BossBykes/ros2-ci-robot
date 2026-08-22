#include <functional>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "ci_bot_control/velocity_guard.hpp"

class VelocityGuardNode : public rclcpp::Node
{
public:
  VelocityGuardNode()
  : Node("velocity_guard"),
    guard_(
      declare_parameter<double>("max_linear_velocity", 0.5),
      declare_parameter<double>("max_angular_velocity", 1.5))
  {
    publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10);

    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_raw",
      10,
      std::bind(
        &VelocityGuardNode::command_callback,
        this,
        std::placeholders::_1));
  }

private:
  void command_callback(
    const geometry_msgs::msg::Twist::SharedPtr message)
  {
    publisher_->publish(guard_.sanitize(*message));
  }

  ci_bot_control::VelocityGuard guard_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<VelocityGuardNode>());

  rclcpp::shutdown();

  return 0;
}
