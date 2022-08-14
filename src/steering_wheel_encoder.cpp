/**
 * Copyright Amro Al-Baali 2022
 * @file steering_wheel_encoder.cpp
 * @brief Implementation of steering_wheel_encoder.hpp and a node entry point
 *
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Feb-24
 */

#include "turtle_nav_cpp/steering_wheel_encoder.hpp"

#include <memory>
#include <string>
#include <vector>

#include "turtle_nav_cpp/eigen_utils.hpp"
#include "turtle_nav_cpp/nav_utils.hpp"

namespace turtle_nav_cpp
{
using std::placeholders::_1;

SteeringWheelEncoder::SteeringWheelEncoder() : Node("steering_wheel_encoder")
{
  // Declare and acquire parameters
  //  Topic to subscribe to
  this->declare_parameter<std::string>("true_meas_topic", "true_turtle/cmd_vel");
  this->get_parameter("true_meas_topic", true_meas_topic_);

  //  Topic to publish to (noisy measurements with covariance)
  this->declare_parameter<std::string>("noisy_meas_topic", "meas/wheel_encoder");
  this->get_parameter("noisy_meas_topic", noisy_meas_topic_);

  //  Measurement frame
  this->declare_parameter<std::string>("meas_frame", "est_turtle");
  this->get_parameter("meas_frame", meas_frame_);

  // (mean, std)
  std::vector<double> gaussian_params;

  //  Linear speed Gaussian PDF params (mean, std)
  //  Note that a non-zero mean value implies the sensor is biased
  this->declare_parameter("linear_speed_noise_params", std::vector<double>{0.0, 1.0});
  this->get_parameter("linear_speed_noise_params", gaussian_params);
  linear_speed_noise_gaussian_ =
    std::normal_distribution<double>(gaussian_params[0], gaussian_params[1]);

  //  Angular speed Gaussian PDF params (mean, std)
  //  Note that a non-zero mean value implies the sensor is biased
  gaussian_params.clear();
  this->declare_parameter("angular_speed_noise_params", std::vector<double>{0.0, 1.0});
  this->get_parameter("angular_speed_noise_params", gaussian_params);
  angular_speed_noise_gaussian_ =
    std::normal_distribution<double>(gaussian_params[0], gaussian_params[1]);

  std::stringstream ss;
  ss << "Map params. mu: \033[92;1m" << gaussian_params[0] << "\033[0m"
     << ", std: \033[92;1m" << gaussian_params[1] << "\033[0m";
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());

  rn_generator_ = std::default_random_engine();

  // Subscribe to true measurement topic
  true_meas_subscriber_ = this->create_subscription<Twist>(
    true_meas_topic_, 10, std::bind(&SteeringWheelEncoder::MeasCallBack, this, _1));

  // Set up publisher
  noisy_meas_publisher_ = this->create_publisher<TwistWithCovStamped>(noisy_meas_topic_, 10);
}

void SteeringWheelEncoder::MeasCallBack(const Twist::SharedPtr true_meas)
{
  TwistWithCovStamped noisy_meas;
  noisy_meas.header.frame_id = meas_frame_;
  noisy_meas.header.stamp = this->get_clock()->now();
  noisy_meas.twist.twist = *true_meas.get();

  // Add noise
  noisy_meas.twist.twist.linear.x += linear_speed_noise_gaussian_(rn_generator_);
  noisy_meas.twist.twist.angular.z += angular_speed_noise_gaussian_(rn_generator_);

  // Covariance on x, y, theta
  Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Zero();

  cov(nav_utils::ThreeDof::PoseIdx::x, nav_utils::ThreeDof::PoseIdx::x) =
    std::pow(linear_speed_noise_gaussian_.stddev(), 2);
  cov(nav_utils::ThreeDof::PoseIdx::y, nav_utils::ThreeDof::PoseIdx::y) = -1.0;
  cov(nav_utils::ThreeDof::PoseIdx::th, nav_utils::ThreeDof::PoseIdx::th) =
    std::pow(angular_speed_noise_gaussian_.stddev(), 2);

  noisy_meas.twist.covariance = eigen_utils::MatrixToStdArray(cov);

  noisy_meas_publisher_->publish(noisy_meas);
}
}  // namespace turtle_nav_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_nav_cpp::SteeringWheelEncoder>());
  rclcpp::shutdown();
  return 0;
}
