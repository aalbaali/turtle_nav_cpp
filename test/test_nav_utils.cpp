/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_nav_utils.cpp
 * @brief Test nav_utils.cpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-22
 */

#include "geometry_msgs/msg/quaternion.hpp"
#include "gtest/gtest.h"
#include "turtle_nav_cpp/nav_utils.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
TEST(HeadingToQuaternion, CheckQuaternion)
{
  const double heading = M_PI_4;

  // The online calculator has been used to calculate the quaternion matrix
  // https://www.andre-gaschler.com/rotationconverter/
  auto q = HeadingToQuaternion(heading);

  EXPECT_DOUBLE_EQ(q.x(), 0);
  EXPECT_DOUBLE_EQ(q.y(), 0);
  EXPECT_DOUBLE_EQ(q.z(), sin(heading / 2));
  EXPECT_DOUBLE_EQ(q.w(), cos(heading / 2));
}

TEST(HeadingToQuaternion, CheckRotationMatrix)
{
  const double heading = M_PI_4;

  auto C = HeadingToQuaternion(heading).toRotationMatrix();

  EXPECT_DOUBLE_EQ(C(0, 0), cos(heading));
  EXPECT_DOUBLE_EQ(C(0, 1), -sin(heading));
  EXPECT_DOUBLE_EQ(C(1, 0), sin(heading));
  EXPECT_DOUBLE_EQ(C(1, 1), cos(heading));
  EXPECT_DOUBLE_EQ(C(0, 2), 0);
  EXPECT_DOUBLE_EQ(C(1, 2), 0);
  EXPECT_DOUBLE_EQ(C(2, 0), 0);
  EXPECT_DOUBLE_EQ(C(2, 1), 0);
  EXPECT_DOUBLE_EQ(C(2, 2), 1);
}

TEST(QuaternionToHeading, ZeroHeadingQuaternion)
{
  Eigen::Quaterniond q;
  q.x() = 0;
  q.y() = 0;
  q.z() = 0;
  q.w() = 1;
  q.normalize();

  EXPECT_DOUBLE_EQ(QuaternionToHeading(q), 0);
}

TEST(QuaternionToHeading, NonZeroHeadingQuaternion)
{
  const double heading = M_PI_4;

  Eigen::Quaterniond q(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));

  EXPECT_DOUBLE_EQ(QuaternionToHeading(q), heading);

  // The negative of the quaternion should give the same heading
  q.x() = q.x();
  q.y() = q.y();
  q.z() = q.z();
  q.w() = q.w();
  EXPECT_DOUBLE_EQ(QuaternionToHeading(q), heading);
}

TEST(QuaternionMsgToQuaternion, EquateQuaternions)
{
  const double heading = M_PI_4;

  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = 0;
  q_msg.y = 0;
  q_msg.z = sin(heading / 2);
  q_msg.w = cos(heading / 2);

  auto q_eigen = QuaternionMsgToQuaternion(q_msg);
  EXPECT_DOUBLE_EQ(q_msg.x, q_eigen.x());
  EXPECT_DOUBLE_EQ(q_msg.y, q_eigen.y());
  EXPECT_DOUBLE_EQ(q_msg.z, q_eigen.z());
  EXPECT_DOUBLE_EQ(q_msg.w, q_eigen.w());
}

TEST(QuaternionMsgToHeading, Headings)
{
  const double heading = M_PI_4;

  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = 0;
  q_msg.y = 0;
  q_msg.z = sin(heading / 2);
  q_msg.w = cos(heading / 2);

  EXPECT_DOUBLE_EQ(QuaternionMsgToHeading(q_msg), heading);

  // Multiply by -1 should also be the same
  q_msg.z = -q_msg.z;
  q_msg.w = -q_msg.w;
  EXPECT_DOUBLE_EQ(QuaternionMsgToHeading(q_msg), heading);
}
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
