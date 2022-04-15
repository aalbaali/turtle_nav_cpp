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
#include <utility>

#include "turtle_nav_cpp/eigen_utils.hpp"
#include "turtle_nav_cpp/nav_utils.hpp"
#include "turtle_nav_cpp/pose.hpp"
#include "turtle_nav_cpp/ros_utils.hpp"

using std::placeholders::_1;

namespace turtle_nav_cpp
{
DeadReckonEstimator::DeadReckonEstimator()
: Node("dead_reckon_estimator"),
  uncertainty_polygon_topic_(ros_utils::DeclareAndImportParam<std::string>(
    this, "uncertainty_polygon_topic", "/est_turtle/uncertainty_bounds")),
  publishing_freq_(ros_utils::DeclareAndImportParam(this, "publishing_freq", 10.0)),
  uncertainty_polygon_num_points_(
    ros_utils::DeclareAndImportParam(this, "uncertainty_polygon_num_points", 30))
{
  // -- Declare and acquire parameters
  // Initial pose
  this->declare_parameter<std::string>("initial_pose_topic", "/initialpose");
  this->get_parameter("initial_pose_topic", initial_pose_topic_);

  // Measured cmd vel
  this->declare_parameter<std::string>("cmd_vel_meas_topic", "/meas/cmd_vel");
  this->get_parameter("cmd_vel_meas_topic", cmd_vel_meas_topic_);

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
  uncertainty_polygon_publisher_ =
    this->create_publisher<PolygonStamped>(uncertainty_polygon_topic_, 10);

  est_pose_publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1 / publishing_freq_),
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
  const std::string& tmp = ss.str();
  const char* cstr = tmp.c_str();
  RCLCPP_INFO(this->get_logger(), cstr);
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

  const std::string& tmp = ss.str();
  const char* cstr = tmp.c_str();
  RCLCPP_INFO(this->get_logger(), cstr);
}

void DeadReckonEstimator::PublishEstimatedPose(
  const PoseWithCovarianceStamped & pose_with_cov_stamped) const
{
  est_pose_publisher_->publish(pose_with_cov_stamped);
}

void DeadReckonEstimator::PublishUncertaintyPolygon(const PolygonStamped & polygon) const
{
  uncertainty_polygon_publisher_->publish(polygon);
}

void DeadReckonEstimator::TimedDeadReckoning()
{
  if (!estimator_is_active_) {
    RCLCPP_INFO(this->get_logger(), "dead_reckon_estimator node not active yet");
    return;
  }

  rclcpp::Time now = this->get_clock()->now();

  try {
    latest_est_pose_msg_ = nav_utils::AccumOdom(now, latest_est_pose_msg_, cmd_vel_history_);
  } catch (const std::string & e) {    
    const char* cstr = e.c_str();
    RCLCPP_INFO(this->get_logger(), cstr);
  }

  // Compute uncertainty polygon
  const nav_utils::Pose pose(latest_est_pose_msg_.pose.pose);
  const auto cov_xi = nav_utils::Cov3dofMsgToCov2dof(latest_est_pose_msg_.pose.covariance);

  PolygonStamped uncertainty_polygon_msg;
  uncertainty_polygon_msg.header = latest_est_pose_msg_.header;

  // Scale ellipse by this number (sqrt(chi2inv(3, 0.99)))
  const double scale = 3.368214175218727;
  const auto points =
    nav_utils::RetractSe2CovarianceEllipse(pose, cov_xi, scale, uncertainty_polygon_num_points_);
  uncertainty_polygon_msg.polygon = std::move(ros_utils::PointsToPolygon(points));

  // Log info
  std::stringstream ss;
  ss << "Pose estimate: \033[96;1m" << nav_utils::Pose(latest_est_pose_msg_.pose.pose)
     << "\033[0m\n";

  // Convert covariance to SE(3) covariance message
  const auto cov_T_k = nav_utils::Cov3dofMsgToCov2dof(latest_est_pose_msg_.pose.covariance);
  ss << "Pose cov:\033[96;1m\n" << cov_T_k << "\033[0m";
  const std::string& tmp = ss.str();
  const char* cstr = tmp.c_str();
  RCLCPP_INFO(this->get_logger(), cstr);

  // Publish messages
  PublishEstimatedPose(latest_est_pose_msg_);
  PublishUncertaintyPolygon(uncertainty_polygon_msg);
}

}  // namespace turtle_nav_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_nav_cpp::DeadReckonEstimator>());
  rclcpp::shutdown();
}
