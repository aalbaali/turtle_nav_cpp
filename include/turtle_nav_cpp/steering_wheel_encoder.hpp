/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file steering_wheel_encoder.hpp
 * @brief The node subscribes to the true `/cmd_vel` topic[Twist] and publishes to a new
 *        topic[TwistWithCovarianceStamped]
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-17
 */

#ifndef TURTLE_NAV_CPP_STEERING_WHEEL_ENCODER_HPP_
#define TURTLE_NAV_CPP_STEERING_WHEEL_ENCODER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace turtle_nav_cpp
{
using Twist = geometry_msgs::msg::Twist;
using TwistWithCovStamped = geometry_msgs::msg::TwistWithCovarianceStamped;

class SteeringWheelEncoder : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Steering Wheel Encoder object
   *
   */
  SteeringWheelEncoder();

private:
  /**
   * @brief Call back function when a cmd_vel measurement is received
   *
   * @param[in] true_meas
   */
  void MeasCallBack(const Twist::SharedPtr true_meas);

  // Topic to subscribe to
  std::string true_meas_topic_;

  // Topic to publish to
  std::string noisy_meas_topic_;

  // Measurement frame
  std::string meas_frame_;

  // Subscriber
  rclcpp::Subscription<Twist>::SharedPtr true_meas_subscriber_{nullptr};

  // Publisher
  rclcpp::Publisher<TwistWithCovStamped>::SharedPtr noisy_meas_publisher_{nullptr};

  // Random number generator
  std::default_random_engine rn_generator_;

  // Measurement noise (Gaussian/normal) distributions
  std::normal_distribution<double> linear_speed_noise_gaussian_;
  std::normal_distribution<double> angular_speed_noise_gaussian_;
};

}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_STEERING_WHEEL_ENCODER_HPP_
