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
#include <unsupported/Eigen/MatrixFunctions>

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

TEST_F(TestPose, AdjointMatrix)
{
  // Adjoint
  const Pose T(x_, y_, theta_);
  const auto adj = T.Adjoint();

  // Construct true Adjoint matrix using (159) in Sola
  Eigen::Matrix3d adj_true = Eigen::Matrix3d::Zero();
  // clang-format off
  adj_true << cos(theta_), -sin(theta_),  y_,
              sin(theta_),  cos(theta_), -x_,
                  0      ,      0      ,  1;
  // clang-format on

  // Compare
  for (int i = 0; i < adj.size(); i++) {
    EXPECT_DOUBLE_EQ(adj(i), adj_true(i));
  }
}

TEST_F(TestPose, ExpMapNonZeroTheta)
{
  // Exponential map
  const double theta = 0.5;
  const Pose pose = Pose::Exp({1, 2}, theta);
  const Eigen::Matrix3d T = pose.Affine().matrix();

  // The matrix should be approximately close to the exponential map computed numerically
  Eigen::Matrix3d Xi;
  // clang-format off
  Xi <<     0   ,  -theta , 1,
          theta ,     0   , 2,
            0   ,     0   , 0;
  // clang-format on
  const Eigen::Matrix3d T_approx = Xi.exp();

  for (int i = 0; i < T.size(); i++) {
    EXPECT_NEAR(T(i), T_approx(i), 1e-5);
  }
}

TEST_F(TestPose, ExpMapZeroTheta)
{
  // Exponential map
  const double theta = 0.0;
  const Pose pose = Pose::Exp({1, 2}, theta);
  const Eigen::Matrix3d T = pose.Affine().matrix();

  // The matrix should be approximately close to the exponential map computed numerically
  Eigen::Matrix3d Xi;
  // clang-format off
  Xi <<     0   ,  -theta , 1,
          theta ,     0   , 2,
            0   ,     0   , 0;
  // clang-format on
  const Eigen::Matrix3d T_approx = Xi.exp();

  for (int i = 0; i < T.size(); i++) {
    EXPECT_NEAR(T(i), T_approx(i), 1e-5);
  }
}

TEST_F(TestPose, Operators)
{
  // Default pose
  Pose T;

  // Assign affine object
  Eigen::Affine2d affine;
  affine.linear() = Heading(theta_).RotationMatrix();
  affine.translation() = translation_;
  T = affine;
  EXPECT_DOUBLE_EQ(T.x(), x_);
  EXPECT_DOUBLE_EQ(T.y(), y_);
  EXPECT_DOUBLE_EQ(T.angle(), theta_);

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
  auto x_3 = cos(theta_) * 1 - sin(theta_) * 2 + x_;
  auto y_3 = sin(theta_) * 1 + sin(theta_) * 2 + y_;
  EXPECT_DOUBLE_EQ(T3.x(), x_3);
  EXPECT_DOUBLE_EQ(T3.y(), y_3);
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
  x_3 = cos(theta_) * 5 - sin(theta_) * 6 + x_;
  y_3 = sin(theta_) * 5 + sin(theta_) * 6 + y_;
  EXPECT_DOUBLE_EQ(T.x(), x_3);
  EXPECT_DOUBLE_EQ(T.y(), y_3);
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
