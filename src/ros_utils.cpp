/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file ros_utils.cpp
 * @brief Implementation of ros_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Apr-10
 */

#include "turtle_nav_cpp/ros_utils.hpp"

#include <algorithm>
#include <vector>

namespace turtle_nav_cpp
{
namespace ros_utils
{
geometry_msgs::msg::Point32 PointToPoint32Msg(const Eigen::Vector2d & point)
{
  geometry_msgs::msg::Point32 point_msg;
  point_msg.x = static_cast<float>(point(0));
  point_msg.y = static_cast<float>(point(1));
  point_msg.z = 0;

  return point_msg;
}

geometry_msgs::msg::Polygon PointsToPolygon(const std::vector<Vector2d> & points)
{
  geometry_msgs::msg::Polygon polygon;
  polygon.points.reserve(points.size());
  std::for_each(points.begin(), points.end(), [&polygon](const Vector2d & v) {
    polygon.points.push_back(PointToPoint32Msg(v));
  });
  return polygon;
}

}  // namespace ros_utils
}  // namespace turtle_nav_cpp
