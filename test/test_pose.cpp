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

  // Construct from Eigen::Affine2D object
  Eigen::Affine2d affine;
  affine.linear() = Eigen::Rotation2Dd(theta_).toRotationMatrix();
  affine.translation() = translation_;
  T = Pose(affine);
  EXPECT_DOUBLE_EQ(T.x(), x_);
  EXPECT_DOUBLE_EQ(T.y(), y_);
  EXPECT_DOUBLE_EQ(T.angle(), theta_);

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

TEST_F(TestPose, Getters)
{
  Pose T(x_, y_, theta_);

  // ROS geometry pose
  const auto pose_msg = T.PoseMsg();
  EXPECT_DOUBLE_EQ(pose_msg.position.x, x_);
  EXPECT_DOUBLE_EQ(pose_msg.position.y, y_);
  EXPECT_DOUBLE_EQ(pose_msg.position.z, 0);
  EXPECT_DOUBLE_EQ(pose_msg.orientation.x, 0);
  EXPECT_DOUBLE_EQ(pose_msg.orientation.y, 0);
  EXPECT_DOUBLE_EQ(pose_msg.orientation.z, sin(theta_ / 2));
  EXPECT_DOUBLE_EQ(pose_msg.orientation.w, cos(theta_ / 2));

  // Turtlesim pose
  const auto pose_turtle = T.TurtlePose();
  EXPECT_FLOAT_EQ(pose_turtle.x, x_);
  EXPECT_FLOAT_EQ(pose_turtle.y, y_);
  EXPECT_FLOAT_EQ(pose_turtle.theta, theta_);

  // Affine
  const auto affine = T.Affine();
  EXPECT_TRUE(affine.translation() == translation_);
  EXPECT_TRUE(affine.linear() == T.heading().RotationMatrix());

  // Translation
  const auto translation_out = T.translation();
  EXPECT_TRUE(translation_out == translation_);

  // Heading
  const auto heading_out = T.heading();
  EXPECT_DOUBLE_EQ(heading_out.angle(), theta_);

  // Scalars
  EXPECT_DOUBLE_EQ(T.x(), x_);
  EXPECT_DOUBLE_EQ(T.y(), y_);
  EXPECT_DOUBLE_EQ(T.angle(), theta_);
}

TEST_F(TestPose, Operators)
{
  // Default pose
  Pose T;

  // Assign ROS geometry pose
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = x_;
  pose_msg.position.y = y_;
  pose_msg.orientation.x = 0;
  pose_msg.orientation.y = 0;
  pose_msg.orientation.z = sin(theta_ / 2);
  pose_msg.orientation.w = cos(theta_ / 2);
  T = pose_msg;
  EXPECT_DOUBLE_EQ(T.x(), x_);
  EXPECT_DOUBLE_EQ(T.y(), y_);
  EXPECT_DOUBLE_EQ(T.angle(), theta_);

  // Turtlesim pose
  // Note that turtlesim stores data in floats
  turtlesim::msg::Pose pose_turtle;
  pose_turtle.x = x_;
  pose_turtle.y = y_;
  pose_turtle.theta = theta_;
  T = Pose();  // Reset pose
  T = pose_turtle;
  EXPECT_FLOAT_EQ(T.x(), x_);
  EXPECT_FLOAT_EQ(T.y(), y_);
  EXPECT_FLOAT_EQ(T.angle(), theta_);

  // Pose compounding
  auto T1 = Pose(x_, y_, theta_);
  Pose T2(1, 2, 0.1);
  auto T3 = T1 * T2;
  EXPECT_DOUBLE_EQ(T3.x(), x_ + 1);
  EXPECT_DOUBLE_EQ(T3.y(), y_ + 2);
  EXPECT_DOUBLE_EQ(T3.angle(), theta_ + 0.1);

  // Transforming a vector into new frame
  T = Pose(x_, y_, theta_);
  Vector2d v{3, 4};
  double x_expect = cos(theta_) * v(0) - sin(theta_) * v(1) + x_;
  double y_expect = sin(theta_) * v(0) + cos(theta_) * v(1) + y_;
  auto v2 = T * v;
  EXPECT_DOUBLE_EQ(v2(0), x_expect);
  EXPECT_DOUBLE_EQ(v2(1), y_expect);

  // Self compounding
  T *= Pose(5, 6, -0.1);
  EXPECT_DOUBLE_EQ(T.x(), x_ + 5);
  EXPECT_DOUBLE_EQ(T.y(), y_ + 6);
  EXPECT_DOUBLE_EQ(T.angle(), theta_ - 0.1);

  // Equality and non-equality operator
  T = Pose(x_, y_, theta_);
  EXPECT_TRUE(T == T);
  EXPECT_FALSE(T != T);

  // Function for checking the equalities and non-equalities for a given pose, where it is expected
  // that the two poses are not equal
  auto expect_not_equal = [&T](Pose T_in) {
    EXPECT_FALSE(T == T_in);
    EXPECT_TRUE(T != T_in);
  };
  expect_not_equal(Pose(x_, y_, 0));
  expect_not_equal(Pose(x_, 0, theta_));
  expect_not_equal(Pose(0, y_, 0));
  expect_not_equal(Pose(0, 0, theta_));
  expect_not_equal(Pose(0, y_, theta_));
  expect_not_equal(Pose(0, 0, 0));
}

TEST_F(TestPose, Inverse)
{
  // T = [C r]
  // T_inv = [C' -C'r]
  const Pose T(x_, y_, theta_);
  const auto T_inv = T.Inverse();
  const double theta_expect = -theta_;
  const double x_expect = -(cos(-theta_) * x_ - sin(-theta_) * y_);
  const double y_expect = -(sin(-theta_) * x_ + cos(-theta_) * y_);
  EXPECT_DOUBLE_EQ(T_inv.x(), x_expect);
  EXPECT_DOUBLE_EQ(T_inv.y(), y_expect);
  EXPECT_DOUBLE_EQ(T_inv.angle(), theta_expect);
}

TEST_F(TestPose, Ostream)
{
  const Pose T(x_, y_, theta_);
  std::stringstream ss;
  ss << T;
  const std::string out_str = ss.str();

  // Function that checks if a number is a substring of the output string
  auto str_contains_double = [&](double val) {
    std::stringstream ss;
    ss << val;
    EXPECT_TRUE(out_str.find(ss.str()) != std::string::npos);
  };
  // Check that each of the internal scalars is a substring of the output string
  str_contains_double(x_);
  str_contains_double(y_);
  str_contains_double(theta_);
}
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
