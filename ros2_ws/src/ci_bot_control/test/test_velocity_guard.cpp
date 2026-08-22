#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "geometry_msgs/msg/twist.hpp"

#include "ci_bot_control/velocity_guard.hpp"

TEST(VelocityGuardTest, PassesValidVelocity)
{
  ci_bot_control::VelocityGuard guard(0.5, 1.5);

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.3;
  command.angular.z = 1.0;

  const auto result = guard.sanitize(command);

  EXPECT_DOUBLE_EQ(result.linear.x, 0.3);
  EXPECT_DOUBLE_EQ(result.angular.z, 1.0);
}

TEST(VelocityGuardTest, ClampsExcessiveVelocity)
{
  ci_bot_control::VelocityGuard guard(0.5, 1.5);

  geometry_msgs::msg::Twist command;
  command.linear.x = 2.0;
  command.angular.z = -4.0;

  const auto result = guard.sanitize(command);

  EXPECT_DOUBLE_EQ(result.linear.x, 0.5);
  EXPECT_DOUBLE_EQ(result.angular.z, -1.5);
}

TEST(VelocityGuardTest, RejectsNanVelocity)
{
  ci_bot_control::VelocityGuard guard(0.5, 1.5);

  geometry_msgs::msg::Twist command;
  command.linear.x = std::numeric_limits<double>::quiet_NaN();
  command.angular.z = 0.5;

  const auto result = guard.sanitize(command);

  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.0);
}

TEST(VelocityGuardTest, RejectsInfiniteVelocity)
{
  ci_bot_control::VelocityGuard guard(0.5, 1.5);

  geometry_msgs::msg::Twist command;
  command.linear.x = 0.2;
  command.angular.z = std::numeric_limits<double>::infinity();

  const auto result = guard.sanitize(command);

  EXPECT_DOUBLE_EQ(result.linear.x, 0.0);
  EXPECT_DOUBLE_EQ(result.angular.z, 0.0);
}
