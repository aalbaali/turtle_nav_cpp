/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file nav_utils.cpp
 * @brief Implementation of nav_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-22
 */

#include "turtle_nav_cpp/nav_utils.hpp"

#include "turtle_nav_cpp/math_utils.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
Eigen::Quaterniond QuaternionMsgToQuaternion(const geometry_msgs::msg::Quaternion & q_msg)
{
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  q.normalize();
  return q;
}

geometry_msgs::msg::Quaternion QuaternionToQuaternionMsg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();
  return q_msg;
}

double QuaternionMsgToHeading(const geometry_msgs::msg::Quaternion & q)
{
  return WrapToPi(QuaternionToHeading(QuaternionMsgToQuaternion(q)));
}

geometry_msgs::msg::Quaternion HeadingToQuaternionMsg(double heading)
{
  return QuaternionToQuaternionMsg(HeadingToQuaternion(heading));
}
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
