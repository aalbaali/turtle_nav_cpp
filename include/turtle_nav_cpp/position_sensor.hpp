/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file position_sensor.hpp
 * @brief Node simulating a position sensor. It subscribes to the true turtlesim
 *        pose and publishes noisy positions
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-17
 */
#ifndef TURTLE_NAV_CPP_POSITION_SENSOR_HPP_
#define TURTLE_NAV_CPP_POSITION_SENSOR_HPP_

#include <Eigen/Dense>
#include <algorithm>
#include <functional>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <turtlesim/msg/pose.hpp>
#include <vector>

#include "turtle_nav_cpp/msg/vector3_with_covariance_stamped.hpp"

namespace turtle_nav_cpp
{
using TurtlePose = turtlesim::msg::Pose;
using Vec3WithCovStamped = turtle_nav_cpp::msg::Vector3WithCovarianceStamped;

using Eigen::Matrix2d;
using Eigen::Vector2d;

class PositionSensor : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Position Sensor object
   *
   */
  PositionSensor();

private:
  /**
   * @brief Measurement call-back function that publishes a noise measured position
   *
   * @param[in] true_pose The true pose from which the true position is extracted
   */
  void MeasCallBack(const TurtlePose::SharedPtr true_pose);

  // Topic to subscribe to
  std::string true_meas_topic_;

  // Topic to publish to
  std::string noisy_meas_topic_;

  // Measurement frame
  std::string meas_frame_;

  // Subscriber
  rclcpp::Subscription<TurtlePose>::SharedPtr true_meas_subscriber_{nullptr};

  // Publisher
  rclcpp::Publisher<Vec3WithCovStamped>::SharedPtr noisy_meas_publisher_{nullptr};

  // Random number generator
  std::default_random_engine rn_generator_;

  // Scalar function that returns a sample from a standard normal distribution
  std::function<double()> randn_;

  // Measurement parameters
  Vector2d noise_biases_;
  Matrix2d noise_cov_;

  // Covariance cholesky decomposition, lower triangular matrix
  Matrix2d noise_cov_chol_L_;
};
}  // namespace turtle_nav_cpp

#endif  // TURTLE_NAV_CPP_POSITION_SENSOR_HPP_
