/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_heading.cpp
 * @brief Test heading.{hpp, cpp}
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-22
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include "gtest/gtest.h"
#include "turtle_nav_cpp/heading.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
using ::testing::DoubleLE;

class TestHeading : public ::testing::Test
{
protected:
  void SetUp() override
  {
    theta_ = M_PI_4;
    q_eigen_ = Eigen::Quaterniond(cos(theta_ / 2), 0, 0, sin(theta_ / 2));
    q_msg_.x = 0;
    q_msg_.y = 0;
    q_msg_.z = sin(theta_ / 2);
    q_msg_.w = cos(theta_ / 2);
  }

  double theta_;
  Eigen::Quaterniond q_eigen_;
  geometry_msgs::msg::Quaternion q_msg_;
};

TEST_F(TestHeading, AnglesWithinBounds)
{
  EXPECT_DOUBLE_EQ(Heading(M_PI_4).angle(), M_PI_4);
  EXPECT_DOUBLE_EQ(Heading(-M_PI_4).angle(), -M_PI_4);
  EXPECT_DOUBLE_EQ(Heading(M_PI_4l + 4 * M_PIl).angle(), M_PI_4);
  EXPECT_DOUBLE_EQ(Heading(-M_PI_4l - 4 * M_PIl).angle(), -M_PI_4);
}

TEST_F(TestHeading, Boundaries)
{
  // Check that angle is within the bounds
  EXPECT_DOUBLE_EQ(Heading(M_PI).angle(), M_PI);
  EXPECT_DOUBLE_EQ(Heading(-M_PI).angle(), M_PI);
}

TEST_F(TestHeading, Constructors)
{
  EXPECT_DOUBLE_EQ(Heading(theta_).angle(), theta_);
  EXPECT_DOUBLE_EQ(Heading(q_msg_).angle(), theta_);
  EXPECT_DOUBLE_EQ(Heading(q_eigen_).angle(), theta_);
}

TEST_F(TestHeading, Getters)
{
  auto Rotation = Heading(theta_).Rotation();
  auto C_computed = Rotation.toRotationMatrix();
  auto C_member = Heading(theta_).RotationMatrix();

  // Compare expected values
  EXPECT_DOUBLE_EQ(C_member(0, 0), cos(theta_));
  EXPECT_DOUBLE_EQ(C_member(0, 1), -sin(theta_));
  EXPECT_DOUBLE_EQ(C_member(1, 0), sin(theta_));
  EXPECT_DOUBLE_EQ(C_member(1, 1), cos(theta_));

  // Compare them to one another
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      EXPECT_DOUBLE_EQ(C_computed(i, j), C_member(i, j));
    }
  }

  // Quaternions
  Heading rotation = theta_;
  auto q_eigen = rotation.Quaternion();
  EXPECT_DOUBLE_EQ(q_eigen.x(), 0);
  EXPECT_DOUBLE_EQ(q_eigen.y(), 0);
  EXPECT_DOUBLE_EQ(q_eigen.z(), sin(theta_ / 2));
  EXPECT_DOUBLE_EQ(q_eigen.w(), cos(theta_ / 2));

  auto q_msg = rotation.QuaternionMsg();
  EXPECT_DOUBLE_EQ(q_msg.x, 0);
  EXPECT_DOUBLE_EQ(q_msg.y, 0);
  EXPECT_DOUBLE_EQ(q_msg.z, sin(theta_ / 2));
  EXPECT_DOUBLE_EQ(q_msg.w, cos(theta_ / 2));
}

TEST_F(TestHeading, Inverse)
{
  EXPECT_DOUBLE_EQ(Heading(theta_).Inverse().angle(), -theta_);

  // Edge cases
  EXPECT_DOUBLE_EQ(Heading(M_PI).Inverse().angle(), M_PI);
  EXPECT_DOUBLE_EQ(Heading(-M_PI).Inverse().angle(), M_PI);
}

TEST_F(TestHeading, AssignmentOperators)
{
  Heading C;

  // Assignment operators
  //  Double
  C = M_PI_4;
  EXPECT_DOUBLE_EQ(C.angle(), M_PI_4);
  C = M_PI_4l + 4 * M_PIl;
  EXPECT_DOUBLE_EQ(C.angle(), M_PI_4);
  C = -M_PI_4;
  EXPECT_DOUBLE_EQ(C.angle(), -M_PI_4);
  C = -M_PI_4l - 4 * M_PIl;
  EXPECT_DOUBLE_EQ(C.angle(), -M_PI_4);

  //  Eigen quaternion
  C = q_eigen_;
  EXPECT_DOUBLE_EQ(C.angle(), theta_);
  q_eigen_.x() = -q_eigen_.x();
  q_eigen_.y() = -q_eigen_.y();
  q_eigen_.z() = -q_eigen_.z();
  q_eigen_.w() = -q_eigen_.w();
  C = q_eigen_;
  EXPECT_DOUBLE_EQ(C.angle(), theta_);

  //  ROS geometry quaternion
  C = q_msg_;
  EXPECT_DOUBLE_EQ(C.angle(), theta_);
  q_msg_.x = -q_msg_.x;
  q_msg_.y = -q_msg_.y;
  q_msg_.z = -q_msg_.z;
  q_msg_.w = -q_msg_.w;
  C = q_msg_;
  EXPECT_DOUBLE_EQ(C.angle(), theta_);
}

TEST_F(TestHeading, ArithmeticOperators)
{
  double heading_1 = M_PI_2;
  const long double heading_1l = M_PI_2l;
  const Eigen::Quaterniond q_eigen_1(cos(heading_1 / 2), 0, 0, sin(heading_1 / 2));
  geometry_msgs::msg::Quaternion q_msg_1;
  q_msg_1.x = q_eigen_1.x();
  q_msg_1.y = q_eigen_1.y();
  q_msg_1.z = q_eigen_1.z();
  q_msg_1.w = q_eigen_1.w();

  const double heading_2 = theta_;
  const long double heading_2l = M_PI_4l;
  const Eigen::Quaterniond q_eigen_2 = q_eigen_;
  const geometry_msgs::msg::Quaternion q_msg_2 = q_msg_;

  Heading C_1 = heading_1;
  // Addition operators
  //  Double
  EXPECT_DOUBLE_EQ((C_1 + heading_2).angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((C_1 + heading_2 + 2 * M_PIl).angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((C_1 + heading_2).angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((C_1 + heading_2 + 2 * M_PIl).angle(), heading_1l + heading_2l);

  //  Quaternions
  EXPECT_DOUBLE_EQ((C_1 + q_eigen_2).angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((Heading(q_eigen_1) + Heading(q_eigen_2)).angle(), heading_1l + theta_);
  EXPECT_DOUBLE_EQ((C_1 + q_msg_2).angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((Heading(q_msg_1) + Heading(q_msg_2)).angle(), heading_1l + theta_);

  // Subtraction (Use `EXPECT_FLOAT_EQ` because some precision is lost when subtracting)
  EXPECT_FLOAT_EQ((C_1 - heading_2).angle(), M_PI_4);
  EXPECT_FLOAT_EQ((C_1 - heading_2 - 4 * M_PIl).angle(), M_PI_4);
  EXPECT_FLOAT_EQ((C_1 - Heading(heading_2)).angle(), M_PI_4);
  EXPECT_FLOAT_EQ((C_1 - Heading(heading_2) - 4 * M_PIl).angle(), M_PI_4);

  // Multiplication
  EXPECT_FLOAT_EQ((C_1 * 0.5).angle(), heading_1 / 2);
  EXPECT_PRED_FORMAT2(DoubleLE, (Heading(M_PI_2) * (long double)(8)).angle(), 1e-10);
  EXPECT_PRED_FORMAT2(DoubleLE, -(Heading(M_PI_2) * (long double)(8)).angle(), 1e-10);

  // Self addition
  auto C_1_original = C_1;
  C_1 += heading_2l;
  EXPECT_DOUBLE_EQ(C_1.angle(), heading_1l + heading_2l);
  C_1 = C_1_original;
  C_1 += 4 * M_PIl;
  EXPECT_DOUBLE_EQ(C_1.angle(), heading_1);

  C_1 = C_1_original;
  C_1 += Heading(heading_2);
  EXPECT_DOUBLE_EQ(C_1.angle(), heading_1l + heading_2l);

  C_1 = C_1_original;
  C_1 += Heading(4 * M_PIl);
  EXPECT_DOUBLE_EQ(C_1.angle(), heading_1l);

  C_1 = Heading(theta_);
  EXPECT_TRUE(C_1 == C_1);
  EXPECT_FALSE(C_1 != C_1);

  auto C_2 = Heading(0);
  EXPECT_FALSE(C_1 == C_2);
  EXPECT_TRUE(C_1 != C_2);
}

TEST_F(TestHeading, Ostream)
{
  const Heading C(theta_);
  std::stringstream ss;
  ss << C;

  const std::string out_str = ss.str();

  ss.str(std::string());
  ss << theta_;
  const std::string expect_str = ss.str();

  EXPECT_EQ(expect_str, out_str);
}

}  // namespace nav_utils
}  // namespace turtle_nav_cpp
