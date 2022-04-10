/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_ros_utils.cpp
 * @brief Test ros_utils.{hpp/cpp}
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Apr-10
 */
#include <Eigen/Dense>

#include "gtest/gtest.h"
#include "turtle_nav_cpp/ros_utils.hpp"

namespace turtle_nav_cpp
{
namespace ros_utils
{
TEST(PointToPoint32Msg, ConvertEigenVector2dToPoint32Msg)
{
  const double x = 1;
  const double y = 2;
  const Eigen::Vector2d pt{x, y};

  const auto pt_msg = PointToPoint32Msg(pt);

  EXPECT_FLOAT_EQ(pt_msg.x, x);
  EXPECT_FLOAT_EQ(pt_msg.y, y);
  EXPECT_FLOAT_EQ(pt_msg.z, 0);
}
}  // namespace ros_utils
}  // namespace turtle_nav_cpp
