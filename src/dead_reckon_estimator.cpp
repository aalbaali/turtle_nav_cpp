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
  const PoseWithCovarianceStamped::SharedPtr pose_with_cov_stamped)
{
  latest_est_pose_ = *pose_with_cov_stamped;
  estimator_is_active_ = true;

  std::stringstream ss;
  ss << "Initial pose received on '\033[36;1m" << initial_pose_topic_ << "'\033[0m";
  ss << "with value '\033[36;1m" << pose_with_cov_stamped << "'\033[0m";
  RCLCPP_INFO(this->get_logger(), ss.str());
}

void DeadReckonEstimator::TruePoseCallBack(const turtlesim::msg::Pose::SharedPtr /* pose */)
{
  if (!estimator_is_active_) {
    RCLCPP_DEBUG(this->get_logger(), "dead_reckon_estimator node not active yet");
    return;
  }
}

void DeadReckonEstimator::CmdVelCallBack(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr /* twist_with_cov_stamped */)
{
  if (!estimator_is_active_) {
    RCLCPP_DEBUG(this->get_logger(), "dead_reckon_estimator node not active yet");
    return;
  }
}

void DeadReckonEstimator::EstPosePublisher(
  const PoseWithCovarianceStamped & /* pose_with_cov_stamped */) const
{
}

}  // namespace turtle_nav_cpp

int main() {}
