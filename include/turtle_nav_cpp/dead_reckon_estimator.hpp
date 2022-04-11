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

#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <turtlesim/msg/pose.hpp>

#include "turtle_nav_cpp/pose.hpp"

namespace turtle_nav_cpp
{
using geometry_msgs::msg::PolygonStamped;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;

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
   * @brief Publish estimated node
   *
   * @param[in] pose_with_cov_stamped Pose to be published
   */
  void PublishEstimatedPose(const PoseWithCovarianceStamped & pose_with_cov_stamped) const;

  /**
   * @brief Publish uncertainty polygon representing 99% confidence level
   *
   * @param[in] polygon Stamped polygon to publish
   */
  void PublishUncertaintyPolygon(const PolygonStamped & polygon) const;

  /**
   * @brief Dead-reckoning algorithm running on a timer
   *
   * @details When this function is called (by the timer), it checks the velocity queue and does the
   *          following:
   *          - If the queue is empty, then publish the latest stored pose (i.e., the robot didn't
   *            move since the last pose)
   *          - Otherwise, multiply out all previous poses and velocities similar to IMU
   *            pre-integration
   *          - Publish the estimated pose through the estimated pose publisher
   */
  void TimedDeadReckoning();

private:
  //////////////////////////////////////////////////////////////////////////////////////////////////
  // -- Topics to subscribe/publish to
  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Initial pose topic to subscribe to
  std::string initial_pose_topic_;

  // Measured velocity topic to subscribe to
  std::string cmd_vel_meas_topic_;

  // Estimated pose topic to publish to
  std::string est_pose_topic_;

  // Pose uncertainty polygon
  std::string uncertainty_polygon_topic_;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // -- Subscribers/publishers
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // Initial-pose subscriber
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscriber_{nullptr};

  // Measured velocity subscriber
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    cmd_vel_meas_subscriber_{nullptr};

  // TODO(aalbaali): Publish odometry instead of PoseWithCovariance
  // Estimated pose publisher
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr est_pose_publisher_{
    nullptr};

  // Estimated uncertainty publisher
  rclcpp::Publisher<PolygonStamped>::SharedPtr uncertainty_polygon_publisher_{nullptr};

  // Timer for estimated pose publisher
  rclcpp::TimerBase::SharedPtr est_pose_publish_timer_;

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // -- Other vars
  //////////////////////////////////////////////////////////////////////////////////////////////////

  // Flag indicating the estimator is active (this is kicked off in the `InitialPoseCallback`)
  bool estimator_is_active_ = false;

  // TODO(aalbaali): To be replaced with a basic type (pose and covaraince)
  // Latest estimated pose
  PoseWithCovarianceStamped latest_est_pose_msg_;

  // History of cmd_vel messages
  std::queue<TwistWithCovarianceStamped> cmd_vel_history_;

  // Publishing frequency
  const double publishing_freq_;

  // Number of points for the uncertainty polygon
  const int uncertainty_polygon_num_points_;
};
}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_DEAD_RECKON_ESTIMATOR_HPP_
