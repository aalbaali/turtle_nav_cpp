/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file ros_utils.cpp
 * @brief Implementation of ros_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Apr-10
 */

#include "turtle_nav_cpp/ros_utils.hpp"

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

// geometry_msgs::msg::Polygon PointsToPolygon(
//   const std::vector<Vector2d> & points, const int num_points)
// {
//   geometry_msgs::msg::Polygon polygon;
//   polygon.points.back()
// }

}  // namespace ros_utils
}  // namespace turtle_nav_cpp
