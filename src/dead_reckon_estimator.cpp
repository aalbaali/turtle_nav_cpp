/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file dead_reckon_estimator.cpp
 * @brief Implementation of dead_reckon_estimator.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-18
 */

#include "turtle_nav_cpp/dead_reckon_estimator.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <array>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "turtle_nav_cpp/nav_utils.hpp"
#include "turtle_nav_cpp/pose.hpp"
#include "turtle_nav_cpp/ros_utils.hpp"

using std::placeholders::_1;

namespace turtle_nav_cpp
{
DeadReckonEstimator::DeadReckonEstimator()
: Node("dead_reckon_estimator"),
  est_pose_pub_freq_(ros_utils::DeclareAndImportParam(this, "est_pose_pub_freq", 10.0))
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

  // Subscribe to the /initialpose topic from rviz
  initial_pose_subscriber_ = this->create_subscription<PoseWithCovarianceStamped>(
    initial_pose_topic_, 10, std::bind(&DeadReckonEstimator::InitialPoseCallBack, this, _1));

  // Measured velocity subscriber
  cmd_vel_meas_subscriber_ = this->create_subscription<TwistWithCovarianceStamped>(
    cmd_vel_meas_topic_, 10, std::bind(&DeadReckonEstimator::CmdVelCallBack, this, _1));

  // Time-based pose estimate publisher
  est_pose_publisher_ = this->create_publisher<PoseWithCovarianceStamped>(est_pose_topic_, 10);
  est_pose_publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1 / est_pose_pub_freq_),
    std::bind(&DeadReckonEstimator::TimedDeadReckoning, this));
}

void DeadReckonEstimator::InitialPoseCallBack(
  const PoseWithCovarianceStamped::SharedPtr pose_with_cov_stamped)
{
  latest_est_pose_msg_ = *pose_with_cov_stamped;
  estimator_is_active_ = true;

  const nav_utils::Pose latest_pose = pose_with_cov_stamped->pose.pose;

  std::stringstream ss;
  ss << "Initial pose received on '\033[36;1m" << initial_pose_topic_ << "'\033[0m ";
  ss << "with value '\033[36;1m(" << latest_pose << ")'\033[0m";
  RCLCPP_INFO(this->get_logger(), ss.str());
}

// !TEMPORARY
void DeadReckonEstimator::TruePoseCallBack(const turtlesim::msg::Pose::SharedPtr pose)
{
  if (!estimator_is_active_) {
    return;
  }

  PoseWithCovarianceStamped pose_with_cov_out;

  // TODO(aalbaali): Get this frame from params
  pose_with_cov_out.header.frame_id = "odom";
  pose_with_cov_out.header.stamp = this->get_clock()->now();
  pose_with_cov_out.pose.pose.position.x = pose->x;
  pose_with_cov_out.pose.pose.position.y = pose->y;

  tf2::Quaternion q;
  q.setRPY(0, 0, pose->theta);
  q.normalize();
  pose_with_cov_out.pose.pose.orientation = tf2::toMsg(q);

  std::array<double, 36> cov;
  cov[0 * 6 + 0] = 1;   // x
  cov[1 * 6 + 1] = 1;   // y
  cov[2 * 6 + 2] = -1;  // z
  cov[3 * 6 + 3] = -1;  // roll
  cov[4 * 6 + 4] = -1;  // pitch
  cov[5 * 6 + 5] = -1;  // yaw

  pose_with_cov_out.pose.covariance = cov;

  latest_est_pose_msg_ = pose_with_cov_out;
}

void DeadReckonEstimator::CmdVelCallBack(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr twist_with_cov_stamped)
{
  cmd_vel_history_.push(*twist_with_cov_stamped);

  double x = twist_with_cov_stamped->twist.twist.linear.x;
  double y = twist_with_cov_stamped->twist.twist.linear.y;
  double th = twist_with_cov_stamped->twist.twist.angular.z;

  std::stringstream ss;
  ss << "Pushed \033[96;1m" << x << ", " << y << ", " << th << "\033[0m. ";
  ss << "cmd_vel.size(): \033[93;1m " << cmd_vel_history_.size() << "\033[0m" << std::endl;

  RCLCPP_INFO(this->get_logger(), ss.str());
}

void DeadReckonEstimator::EstPosePublisher(
  const PoseWithCovarianceStamped & pose_with_cov_stamped) const
{
  est_pose_publisher_->publish(pose_with_cov_stamped);
}

void DeadReckonEstimator::TimedDeadReckoning()
{
  if (!estimator_is_active_) {
    RCLCPP_INFO(this->get_logger(), "dead_reckon_estimator node not active yet");
    return;
  }

  rclcpp::Time now = this->get_clock()->now();

  latest_est_pose_msg_ = nav_utils::AccumOdom(now, latest_est_pose_msg_, cmd_vel_history_);

  // Log info
  std::stringstream ss;
  ss << "Pose at \033[96;1m" << rclcpp::Time(latest_est_pose_msg_.header.stamp).seconds()
     << "\033[0m ";
  ss << "estimated to be \033[96;1m" << nav_utils::Pose(latest_est_pose_msg_.pose.pose)
     << "\033[0m";
  RCLCPP_INFO(this->get_logger(), ss.str());

  EstPosePublisher(latest_est_pose_msg_);
}

}  // namespace turtle_nav_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_nav_cpp::DeadReckonEstimator>());
  rclcpp::shutdown();
}
