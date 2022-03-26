/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file nav_utils.cpp
 * @brief Implementation of nav_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-22
 */

#include "turtle_nav_cpp/nav_utils.hpp"

#include <queue>

#include "turtle_nav_cpp/math_utils.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
//==================================================================================================
// Heading
//==================================================================================================

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

//==================================================================================================
// Poses
//==================================================================================================

turtlesim::msg::Pose PoseMsgToTurtlePose(const geometry_msgs::msg::Pose & pose)
{
  turtlesim::msg::Pose pose_turtle;
  pose_turtle.x = pose.position.x;
  pose_turtle.y = pose.position.y;
  pose_turtle.theta = Heading(pose.orientation).angle();

  return pose_turtle;
}

geometry_msgs::msg::Pose TurtlePoseToPoseMsg(const turtlesim::msg::Pose & pose)
{
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = pose.x;
  pose_msg.position.y = pose.y;
  pose_msg.orientation = Heading(pose.theta).QuaternionMsg();
  return pose_msg;
}

//==================================================================================================
// Filtering
//==================================================================================================

PoseWithCovarianceStamped AccumOdom(
  const rclcpp::Time & query_at, const PoseWithCovarianceStamped & initial_pose,
  std::queue<TwistWithCovarianceStamped> & vel_history)
{
  rclcpp::Time initial_pose_time = initial_pose.header.stamp;

  // Ensure query time is after initial pose
  if (query_at < initial_pose_time) {
    std::stringstream ss;
    ss << "query time \033[93;1m" << query_at.seconds()
       << "\033[0m is earlier than initial_pose time \033[93;1m" << initial_pose_time.seconds();
    throw std::invalid_argument(ss.str());
  }

  // Need non-empty velocity history to dead-reckon
  if (vel_history.size() == 0) {
    PoseWithCovarianceStamped latest_pose = initial_pose;
    latest_pose.header.stamp = query_at;
    return latest_pose;
  }

  // Latest measurement (start from the earliest measurement)
  TwistWithCovarianceStamped earliest_vel = vel_history.back();

  // Flag to indicate that the pose is successfully predicted to the requested query time
  // Note that this may happen before the velocity history queue is emptied
  bool is_predicted_to_query_time = false;

  // Latest pose, where it's assumed that the latest pose remains unchanged until the latest
  // measurement comes in
  PoseWithCovarianceStamped latest_pose = initial_pose;

  // Deal with the case that query time is between initial_pose and the earliest velocity
  // measurement, in which case the pose remains unchanged
  if (query_at < earliest_vel.header.stamp) {
    latest_pose.header.stamp = query_at;
    return latest_pose;
  }

  // Predict poses until the last velocity time stamp
  while (!vel_history.empty() && !is_predicted_to_query_time) {
    earliest_vel = vel_history.back();
    vel_history.pop();

    // Query time to predict to
    rclcpp::Time time_to_predict_to = earliest_vel.header.stamp;

    // Deal with measurements earlier than latest pose
    if (time_to_predict_to < latest_pose.header.stamp) {
      continue;
    }

    // Use query time if it's earlier than the velocity time
    if (query_at <= time_to_predict_to) {
      time_to_predict_to = query_at;
      is_predicted_to_query_time = true;
    }

    // Time difference between latest velocity and latest pose
    auto duration_latest_pose_to_query_time = time_to_predict_to - latest_pose.header.stamp;

    // * Ignore covariances for now
    // Dead-reckon poses using SE(2) model
    double dt = duration_latest_pose_to_query_time.seconds();
    Pose T_km1 = latest_pose.pose.pose;
    Vector2d linear_vel_km1 =
      Vector2d(earliest_vel.twist.twist.linear.x, earliest_vel.twist.twist.linear.y);
    double angular_vel_km1 = earliest_vel.twist.twist.angular.z;
    Pose T_k = T_km1 * Pose(dt * linear_vel_km1, dt * angular_vel_km1);
    latest_pose.pose.pose = T_k.PoseMsg();
    latest_pose.header.stamp = time_to_predict_to;
  }

  // Now predict pose from latest pose to query time
  if (!is_predicted_to_query_time) {
    auto duration_latest_pose_to_query_time = query_at - latest_pose.header.stamp;
    double dt = duration_latest_pose_to_query_time.seconds();
    Pose T_km1 = latest_pose.pose.pose;
    Vector2d linear_vel_km1{earliest_vel.twist.twist.linear.x, earliest_vel.twist.twist.linear.y};
    double angular_vel_km1 = earliest_vel.twist.twist.angular.z;
    Pose T_k = T_km1 * Pose(dt * linear_vel_km1, dt * angular_vel_km1);
    latest_pose.pose.pose = T_k.PoseMsg();
    latest_pose.header.stamp = query_at;
  }

  return latest_pose;
}

}  // namespace nav_utils
}  // namespace turtle_nav_cpp
