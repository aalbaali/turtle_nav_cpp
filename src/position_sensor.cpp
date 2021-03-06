/**
 * Copyright 2022 Amro Al-Baali
 * @file position_sensor.cpp
 * @brief Implementation of `position_sensor.hpp` and a node entry point
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Feb-25
 */

#include "turtle_nav_cpp/position_sensor.hpp"

#include <memory>
#include <string>

#include "turtle_nav_cpp/eigen_utils.hpp"
#include "turtle_nav_cpp/math_utils.hpp"
#include "turtle_nav_cpp/ros_utils.hpp"

namespace turtle_nav_cpp
{
using ros_utils::ImportParamAsEigen;
using std::placeholders::_1;

// `noise_biases` and `noise_cov_` are initialized in the initializer list because they are const
// objects
PositionSensor::PositionSensor()
: Node("position_sensor"),
  noise_biases_(ImportParamAsEigen<2, 1>(this, "biases", Vector2d::Zero())),
  noise_cov_(ImportParamAsEigen<2, 2>(this, "covariance", Matrix2d::Identity())),
  noise_cov_chol_L_(eigen_utils::GetCholeskyLower(noise_cov_)),
  publishing_freq_(ros_utils::DeclareAndImportParam(this, "publishing_freq", 1.0))
{
  // Declare and acquire parameters
  //  Topic to subscribe to
  this->declare_parameter<std::string>("true_meas_topic", "true_turtle/pose");
  this->get_parameter("true_meas_topic", true_meas_topic_);

  //  Topic to publish to (noisy measurements with covariance)
  this->declare_parameter<std::string>("noisy_meas_topic", "meas/position");
  this->get_parameter("noisy_meas_topic", noisy_meas_topic_);

  //  Frame of measurement
  this->declare_parameter<std::string>("meas_frame", "map");
  this->get_parameter("meas_frame", meas_frame_);

  // Subscribe to true pose topic
  true_meas_subscriber_ = this->create_subscription<TurtlePose>(
    true_meas_topic_, 10, std::bind(&PositionSensor::GetMeasurement, this, _1));

  // Set up publisher
  noisy_meas_publisher_ = this->create_publisher<Vec3WithCovStamped>(noisy_meas_topic_, 10);

  // Set the random number generators and the randn_ lambda function
  rn_generator_ = std::default_random_engine();
  randn_ = [this]() { return randn_gen(rn_generator_); };

  // Time-based measurement publisher
  meas_publish_timer_ = this->create_wall_timer(
    std::chrono::duration<double>(1 / publishing_freq_),
    std::bind(&PositionSensor::TimedPublisher, this));

  // Only the last measurement is needed; the older measurements will be ignored
  latest_meas_.reserve(1);
}

void PositionSensor::GetMeasurement(const TurtlePose::SharedPtr true_pose)
{
  Vec3WithCovStamped noisy_meas;
  noisy_meas.header.frame_id = meas_frame_;
  noisy_meas.header.stamp = this->get_clock()->now();

  // Set true position
  noisy_meas.vector.vector.x = true_pose->x;
  noisy_meas.vector.vector.y = true_pose->y;
  noisy_meas.vector.vector.z = 0.0;  // z-value is ignored

  // Sample and add noise
  auto noise = noise_biases_ + noise_cov_chol_L_ * Vector2d::NullaryExpr(2, randn_);
  noisy_meas.vector.vector.x += noise(0);
  noisy_meas.vector.vector.y += noise(1);

  // Covariance on x, y
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  cov.block<2, 2>(0, 0) = noise_cov_;
  cov(2, 2) = -1;

  noisy_meas.vector.covariance = eigen_utils::MatrixToStdArray(cov);

  // Store latest measurement
  if (latest_meas_.empty()) {
    latest_meas_.push_back(noisy_meas);
  } else {
    latest_meas_[0] = noisy_meas;
  }
}

void PositionSensor::TimedPublisher()
{
  if (latest_meas_.empty()) {
    return;
  }

  noisy_meas_publisher_->publish(latest_meas_.back());
  latest_meas_.clear();
}
}  // namespace turtle_nav_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_nav_cpp::PositionSensor>());
  rclcpp::shutdown();
  return 0;
}
