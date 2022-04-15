/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_ros_utils.cpp
 * @brief Test ros_utils.{hpp/cpp}
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Apr-10
 */
#include <Eigen/Dense>
#include <vector>

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

TEST(PointsToPolygon, ConvertVectorOfPointsToPolygon)
{
  std::vector<Eigen::Vector2d> points;
  points.reserve(5);
  for (int i = 0; i < 5; i++) {
    points.push_back({i, i * i});
  }

  const auto polygon = PointsToPolygon(points);

  for (int i = 0; i < 5; i++) {
    EXPECT_FLOAT_EQ(polygon.points[i].x, static_cast<float>(i));
    EXPECT_FLOAT_EQ(polygon.points[i].y, static_cast<float>(i * i));
    EXPECT_FLOAT_EQ(polygon.points[i].z, 0);
  }
}
}  // namespace ros_utils
}  // namespace turtle_nav_cpp
