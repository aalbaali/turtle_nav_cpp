/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_orientation.cpp
 * @brief Test orientation.{hpp, cpp}
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-22
 */

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <string>

#include "gtest/gtest.h"
#include "turtle_nav_cpp/orientation.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
using ::testing::DoubleLE;

class TestOrientation : public ::testing::Test
{
protected:
  void SetUp() override
  {
    heading = M_PI_4;
    q_eigen = Eigen::Quaterniond(cos(heading / 2), 0, 0, sin(heading / 2));
    q_msg.x = 0;
    q_msg.y = 0;
    q_msg.z = sin(heading / 2);
    q_msg.w = cos(heading / 2);
  }

  double heading;
  Eigen::Quaterniond q_eigen;
  geometry_msgs::msg::Quaternion q_msg;
};

TEST_F(TestOrientation, AnglesWithinBounds)
{
  EXPECT_DOUBLE_EQ(Orientation(M_PI_4).Angle(), M_PI_4);
  EXPECT_DOUBLE_EQ(Orientation(-M_PI_4).Angle(), -M_PI_4);
  EXPECT_DOUBLE_EQ(Orientation(M_PI_4l + 4 * M_PIl).Angle(), M_PI_4);
  EXPECT_DOUBLE_EQ(Orientation(-M_PI_4l - 4 * M_PIl).Angle(), -M_PI_4);
}

TEST_F(TestOrientation, Boundaries)
{
  // Check that angle is within the bounds
  EXPECT_DOUBLE_EQ(Orientation(M_PI).Angle(), M_PI);
  EXPECT_DOUBLE_EQ(Orientation(-M_PI).Angle(), M_PI);
}

TEST_F(TestOrientation, Constructors)
{
  EXPECT_DOUBLE_EQ(Orientation(heading).Angle(), heading);
  EXPECT_DOUBLE_EQ(Orientation(q_msg).Angle(), heading);
  EXPECT_DOUBLE_EQ(Orientation(q_eigen).Angle(), heading);
}

TEST_F(TestOrientation, Getters)
{
  auto Rotation = Orientation(heading).Rotation();
  auto C_computed = Rotation.toRotationMatrix();
  auto C_member = Orientation(heading).RotationMatrix();

  // Compare expected values
  EXPECT_DOUBLE_EQ(C_member(0, 0), cos(heading));
  EXPECT_DOUBLE_EQ(C_member(0, 1), -sin(heading));
  EXPECT_DOUBLE_EQ(C_member(1, 0), sin(heading));
  EXPECT_DOUBLE_EQ(C_member(1, 1), cos(heading));

  // Compare them to one another
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      EXPECT_DOUBLE_EQ(C_computed(i, j), C_member(i, j));
    }
  }

  // Quaternions
  Orientation rotation = heading;
  auto q_eigen = rotation.Quaternion();
  EXPECT_DOUBLE_EQ(q_eigen.x(), 0);
  EXPECT_DOUBLE_EQ(q_eigen.y(), 0);
  EXPECT_DOUBLE_EQ(q_eigen.z(), sin(heading / 2));
  EXPECT_DOUBLE_EQ(q_eigen.w(), cos(heading / 2));

  auto q_msg = rotation.QuaternionMsg();
  EXPECT_DOUBLE_EQ(q_msg.x, 0);
  EXPECT_DOUBLE_EQ(q_msg.y, 0);
  EXPECT_DOUBLE_EQ(q_msg.z, sin(heading / 2));
  EXPECT_DOUBLE_EQ(q_msg.w, cos(heading / 2));
}

TEST_F(TestOrientation, EqualityOperators)
{
  Orientation C;

  // Assignment operators
  //  Double
  C = M_PI_4;
  EXPECT_DOUBLE_EQ(C.Angle(), M_PI_4);
  C = M_PI_4l + 4 * M_PIl;
  EXPECT_DOUBLE_EQ(C.Angle(), M_PI_4);
  C = -M_PI_4;
  EXPECT_DOUBLE_EQ(C.Angle(), -M_PI_4);
  C = -M_PI_4l - 4 * M_PIl;
  EXPECT_DOUBLE_EQ(C.Angle(), -M_PI_4);

  //  Eigen quaternion
  C = q_eigen;
  EXPECT_DOUBLE_EQ(C.Angle(), heading);
  q_eigen.x() = -q_eigen.x();
  q_eigen.y() = -q_eigen.y();
  q_eigen.z() = -q_eigen.z();
  q_eigen.w() = -q_eigen.w();
  C = q_eigen;
  EXPECT_DOUBLE_EQ(C.Angle(), heading);

  //  ROS geometry quaternion
  C = q_msg;
  EXPECT_DOUBLE_EQ(C.Angle(), heading);
  q_msg.x = -q_msg.x;
  q_msg.y = -q_msg.y;
  q_msg.z = -q_msg.z;
  q_msg.w = -q_msg.w;
  C = q_msg;
  EXPECT_DOUBLE_EQ(C.Angle(), heading);
}

TEST_F(TestOrientation, ArithmeticOperators)
{
  double heading_1 = M_PI_2;
  const long double heading_1l = M_PI_2l;
  const Eigen::Quaterniond q_eigen_1(cos(heading_1 / 2), 0, 0, sin(heading_1 / 2));
  geometry_msgs::msg::Quaternion q_msg_1;
  q_msg_1.x = q_eigen_1.x();
  q_msg_1.y = q_eigen_1.y();
  q_msg_1.z = q_eigen_1.z();
  q_msg_1.w = q_eigen_1.w();

  const double heading_2 = heading;
  const long double heading_2l = M_PI_4l;
  const Eigen::Quaterniond q_eigen_2 = q_eigen;
  const geometry_msgs::msg::Quaternion q_msg_2 = q_msg;

  Orientation C_1 = heading_1;
  // Addition operators
  //  Double
  EXPECT_DOUBLE_EQ((C_1 + heading_2).Angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((C_1 + heading_2 + 2 * M_PIl).Angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((C_1 + heading_2).Angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((C_1 + heading_2 + 2 * M_PIl).Angle(), heading_1l + heading_2l);

  //  Quaternions
  EXPECT_DOUBLE_EQ((C_1 + q_eigen_2).Angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((Orientation(q_eigen_1) + Orientation(q_eigen_2)).Angle(), heading_1l + heading);
  EXPECT_DOUBLE_EQ((C_1 + q_msg_2).Angle(), heading_1l + heading_2l);
  EXPECT_DOUBLE_EQ((Orientation(q_msg_1) + Orientation(q_msg_2)).Angle(), heading_1l + heading);

  // Subtraction (Use `EXPECT_FLOAT_EQ` because some precision is lost when subtracting)
  EXPECT_FLOAT_EQ((C_1 - heading_2).Angle(), M_PI_4);
  EXPECT_FLOAT_EQ((C_1 - heading_2 - 4 * M_PIl).Angle(), M_PI_4);
  EXPECT_FLOAT_EQ((C_1 - Orientation(heading_2)).Angle(), M_PI_4);
  EXPECT_FLOAT_EQ((C_1 - Orientation(heading_2) - 4 * M_PIl).Angle(), M_PI_4);

  // Multiplication
  EXPECT_FLOAT_EQ((C_1 * 0.5).Angle(), heading_1 / 2);
  EXPECT_PRED_FORMAT2(DoubleLE, (Orientation(M_PI_2) * (long double)(8)).Angle(), 1e-10);
  EXPECT_PRED_FORMAT2(DoubleLE, -(Orientation(M_PI_2) * (long double)(8)).Angle(), 1e-10);

  // Self addition
  auto C_1_original = C_1;
  C_1 += heading_2l;
  EXPECT_DOUBLE_EQ(C_1.Angle(), heading_1l + heading_2l);
  C_1 = C_1_original;
  C_1 += 4 * M_PIl;
  EXPECT_DOUBLE_EQ(C_1.Angle(), heading_1);

  C_1 = C_1_original;
  C_1 += Orientation(heading_2);
  EXPECT_DOUBLE_EQ(C_1.Angle(), heading_1l + heading_2l);

  C_1 = C_1_original;
  C_1 += Orientation(4 * M_PIl);
  EXPECT_DOUBLE_EQ(C_1.Angle(), heading_1l);
}

TEST_F(TestOrientation, Ostream)
{
  const Orientation C(heading);
  std::stringstream ss;
  ss << C;

  const std::string out_str = ss.str();

  ss.str(std::string());
  ss << heading;
  const std::string expect_str = ss.str();

  EXPECT_EQ(expect_str, out_str);
}

}  // namespace nav_utils
}  // namespace turtle_nav_cpp
