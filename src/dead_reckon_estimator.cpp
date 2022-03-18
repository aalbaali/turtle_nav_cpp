/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file dead_reckon_estimator.cpp
 * @brief Implementation of dead_reckon_estimator.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-18
 */

#include "turtle_nav_cpp/dead_reckon_estimator.hpp"

#include <string>

namespace turtle_nav_cpp
{
DeadReckonEstimator::DeadReckonEstimator() : Node("dead_reckon_estimator"), est_pose_pub_freq_(0)
{
  // -- Declare and acquire parameters
  // Initial pose
  this->declare_parameter<std::string>("initial_pose_topic", "/initialpose");
  this->get_parameter("initial_pose_topic", initial_pose_topic_);

  // Measured cmd vel
  this->declare_parameter<std::string>("cmd_vel_meas_topic", "/meas/cmd_vel");
  this->get_parameter("cmd_vel_meas_topic", cmd_vel_meas_topic_);

  //! TEMPORARY
  // True pose topic
  this->declare_parameter<std::string>("true_pose_topic", "/true_turtle/pose");
  this->get_parameter("true_pose_topic", true_pose_topic_);

  // Estimated pose topic to publish to
  this->declare_parameter<std::string>("est_pose_topic", "/est_turtle/est_pose");
  this->get_parameter("est_pose_topic", est_pose_topic_);

  // est_pose_topic_
}

void DeadReckonEstimator::InitialPoseCallBack(
  const PoseWithCovarianceStamped & pose_with_cov_stamped)
{
}

void DeadReckonEstimator::CmdVelCallBack(
  const geometry_msgs::msg::TwistWithCovarianceStamped & twist_with_cov_stamped)
{
}

void DeadReckonEstimator::EstPosePublisher(
  const PoseWithCovarianceStamped & pose_with_cov_stamped) const
{
}

}  // namespace turtle_nav_cpp

int main() {}
