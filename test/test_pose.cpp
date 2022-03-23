/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_pose.cpp
 * @brief Test pose.{hpp, cpp}
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-23
 */

#include <geometry_msgs/msg/pose.hpp>
#include <string>
#include <turtlesim/msg/pose.hpp>

#include "gtest/gtest.h"
#include "turtle_nav_cpp/pose.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
class TestPose : public ::testing::Test
{
protected:
  void SetUp() override
  {
    theta_ = M_PI_4;
    x_ = 1;
    y_ = 2;
    translation_ = Vector2d{x_, y_};
  }

  double theta_;
  double x_;
  double y_;
  Vector2d translation_;
};

TEST_F(TestPose, Constructors)
{
  // Default constructor
  Pose T;
  EXPECT_DOUBLE_EQ(T.x(), 0);
  EXPECT_DOUBLE_EQ(T.y(), 0);
  EXPECT_DOUBLE_EQ(T.angle(), 0);

  // Constructor using `Vector2d` and `Heading` objects
  T = Pose(translation_, Heading(theta_));
  EXPECT_DOUBLE_EQ(T.x(), x_);
  EXPECT_DOUBLE_EQ(T.y(), y_);
  EXPECT_DOUBLE_EQ(T.angle(), theta_);

  // Constructor using `Vector2d` and `double` (for heading)
  T = Pose(translation_, theta_);
  EXPECT_DOUBLE_EQ(T.x(), x_);
  EXPECT_DOUBLE_EQ(T.y(), y_);
  EXPECT_DOUBLE_EQ(T.angle(), theta_);

  // Constructor using scalars
  T = Pose(x_, y_, theta_);
  EXPECT_DOUBLE_EQ(T.x(), x_);
  EXPECT_DOUBLE_EQ(T.y(), y_);
  EXPECT_DOUBLE_EQ(T.angle(), theta_);

  // Constructor from geometry msgs pose
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = x_;
  pose_msg.position.y = y_;
  pose_msg.position.z = 0;
  pose_msg.orientation.x = 0;
  pose_msg.orientation.y = 0;
  pose_msg.orientation.z = sin(theta_ / 2);
  pose_msg.orientation.w = cos(theta_ / 2);
  T = Pose(pose_msg);
  EXPECT_DOUBLE_EQ(T.x(), x_);
  EXPECT_DOUBLE_EQ(T.y(), y_);
  EXPECT_DOUBLE_EQ(T.angle(), theta_);

  // Construct from turtlesim pose
  // Note that the turtlesim pose uses `float` to store the member variables instead of `double`
  turtlesim::msg::Pose pose_turtle;
  pose_turtle.x = x_;
  pose_turtle.y = y_;
  pose_turtle.theta = theta_;
  T = Pose(pose_turtle);
  EXPECT_FLOAT_EQ(T.x(), x_);
  EXPECT_FLOAT_EQ(T.y(), y_);
  EXPECT_FLOAT_EQ(T.angle(), theta_);
}
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
