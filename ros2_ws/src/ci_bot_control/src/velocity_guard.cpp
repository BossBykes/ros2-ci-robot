#include "ci_bot_control/velocity_guard.hpp"

#include <algorithm>
#include <cmath>

namespace ci_bot_control
{

VelocityGuard::VelocityGuard(
  double max_linear_velocity,
  double max_angular_velocity)
: max_linear_velocity_(std::abs(max_linear_velocity)),
  max_angular_velocity_(std::abs(max_angular_velocity))
{
}

geometry_msgs::msg::Twist VelocityGuard::sanitize(
  const geometry_msgs::msg::Twist & command) const
{
  geometry_msgs::msg::Twist safe_command;

  if (!std::isfinite(command.linear.x) ||
    !std::isfinite(command.angular.z))
  {
    return safe_command;
  }

  safe_command.linear.x = std::clamp(
    command.linear.x,
    -max_linear_velocity_,
    max_linear_velocity_);

  safe_command.angular.z = std::clamp(
    command.angular.z,
    -max_angular_velocity_,
    max_angular_velocity_);

  return safe_command;
}

}  // namespace ci_bot_control
