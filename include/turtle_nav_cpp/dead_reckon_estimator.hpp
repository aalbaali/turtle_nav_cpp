/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file dead_reckon_estimator.hpp
 * @brief Dead-reckoning estimation
 *
* @details Subscribes to `/initial_pose` and `est_turtle/cmd_vel` and updates
 * `/est_turtle/est_pose` accordingly
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-18
 */
#ifndef TURTLE_NAV_CPP_DEAD_RECKON_ESTIMATOR_HPP_
#define TURTLE_NAV_CPP_DEAD_RECKON_ESTIMATOR_HPP_

#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <turtlesim/msg/pose.hpp>

#include "turtle_nav_cpp/pose.hpp"

namespace turtle_nav_cpp
{
using geometry_msgs::msg::PoseWithCovarianceStamped;

class DeadReckonEstimator : public rclcpp::Node
{
public:
  DeadReckonEstimator();

private:
  /**
   * @brief Callback function to messages on `/initialpose`, which is usually set by Rviz
   *
   * @details Receiving the initial message kicks off the estimation, and any consequent message
   * simply updates the pose position
   *
   * @param[in] pose_with_cov_stamped Received pose on the `/initialpose` topic
   */
  void InitialPoseCallBack(const PoseWithCovarianceStamped::SharedPtr pose_with_cov_stamped);

  /**
   * @brief Callback function to wheel encoder measurements
   *
   * @details The message is in 3D but only the 2D portions are populated (e.g., the z-axis is
   * ignored)
   *
   * @param[in] twist_with_cov_stamped 2D noisy speed measurements
   */
  void CmdVelCallBack(
    const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_with_cov_stamped);

  /**
   * ! TEMPORARY
   * @brief Turtlesim true-pose call back
   *
   * @param[in] pose True pose from turtlesim
   */
  void TruePoseCallBack(const turtlesim::msg::Pose::SharedPtr pose);

  /**
   * @brief Publish estimated node
   *
   * @param[in] pose_with_cov_stamped Pose to be published
   */
  void EstPosePublisher(const PoseWithCovarianceStamped & pose_with_cov_stamped) const;

private:
  //////////////////////////////////////////////////////////////////////////////////////////////////
  // -- Topics to subscribe/publish to
  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Initial pose topic to subscribe to
  std::string initial_pose_topic_;

  // Measured velocity topic to subscribe to
  std::string cmd_vel_meas_topic_;

  //! TEMPORARY
  // True turtle pose topic
  std::string true_pose_topic_;

  // Estimated pose topic to publish to
  std::string est_pose_topic_;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // -- Subscribers/publishers
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // Initial-pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscriber_{nullptr};

  // Measured velocity subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    cmd_vel_meas_subscriber_{nullptr};

  //! TEMPORARY
  // true pose subscriber
  rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr true_pose_subscriber_{nullptr};

  // TODO(aalbaali): Publish odometry instead of PoseWithCovariance
  // Estimated pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr est_pose_publisher_{
    nullptr};

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // -- Other vars
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // Flag indicating the estimator is active (this is kicked off in the `InitialPoseCallback`)
  bool estimator_is_active_ = false;

  // TODO(aalbaali): To be replaced with a basic type (pose and covaraince)
  // Latest estimated pose
  PoseWithCovarianceStamped latest_est_pose_msg_;

  // Publishing frequency
  const double est_pose_pub_freq_;
};
}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_DEAD_RECKON_ESTIMATOR_HPP_
