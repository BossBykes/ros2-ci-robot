#ifndef CI_BOT_CONTROL__VELOCITY_GUARD_HPP_
#define CI_BOT_CONTROL__VELOCITY_GUARD_HPP_

#include <geometry_msgs/msg/twist.hpp>

namespace ci_bot_control
{

class VelocityGuard
{
public:
  VelocityGuard(double max_linear_velocity, double max_angular_velocity);

  geometry_msgs::msg::Twist sanitize(
    const geometry_msgs::msg::Twist & command) const;

private:
  double max_linear_velocity_;
  double max_angular_velocity_;
};

}  // namespace ci_bot_control

#endif  // CI_BOT_CONTROL__VELOCITY_GUARD_HPP_
