#include <chrono>
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
      declare_parameter<double>("max_angular_velocity", 1.5)),
    command_timeout_seconds_(
      declare_parameter<double>("command_timeout_seconds", 0.5))
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

    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(
        &VelocityGuardNode::watchdog_callback,
        this));
  }

private:
  void command_callback(
    const geometry_msgs::msg::Twist::SharedPtr message)
  {
    last_command_time_ = now();
    has_received_command_ = true;

    publisher_->publish(
      guard_.sanitize(*message));
  }

  void watchdog_callback()
  {
    if (!has_received_command_) {
      return;
    }

    const double elapsed_seconds =
      (now() - last_command_time_).seconds();

    if (elapsed_seconds <= command_timeout_seconds_) {
      return;
    }

    geometry_msgs::msg::Twist stop_command;
    publisher_->publish(stop_command);

    has_received_command_ = false;
  }

  ci_bot_control::VelocityGuard guard_;

  double command_timeout_seconds_;

  bool has_received_command_{false};

  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<VelocityGuardNode>());

  rclcpp::shutdown();

  return 0;
}
