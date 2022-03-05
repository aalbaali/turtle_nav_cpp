/**
 * Copyright 2022 Amro Al-Baali
 * @file position_sensor.cpp
 * @brief Node simulating a position sensor. It subscribes to the true turtlesim
 * pose and publishes noisy positions
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Feb-25
 */

#include <algorithm>
#include <array>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <map>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <turtlesim/msg/pose.hpp>
#include <vector>

#include "turtle_nav_cpp/msg/vector3_with_covariance_stamped.hpp"
#include "turtle_nav_cpp/utils.hpp"

using std::placeholders::_1;

using Twist = geometry_msgs::msg::Twist;
using TwistWithCovStamped = geometry_msgs::msg::TwistWithCovarianceStamped;
using TurtlePose = turtlesim::msg::Pose;
using Vec3WithCovStamped = turtle_nav_cpp::msg::Vector3WithCovarianceStamped;

// String to double map to extract Gaussian params
using StrToDoubleMap = std::map<std::string, double>;

// (mean, std)
using GaussianParamsVec = std::vector<double>;

namespace turtle_nav_cpp
{
class PositionSensor : public rclcpp::Node
{
public:
  PositionSensor() : Node("position_sensor")
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

    //  Get noise biases
    std::vector<double> input;
    this->declare_parameter<std::vector<double>>("biases", std::vector<double>{0.0, 0.0});
    this->get_parameter("biases", input);
    noise_biases_.reserve(2);
    noise_biases_ = input;

    //  Get noise covariance
    input.clear();
    input.reserve(4);
    this->declare_parameter<std::vector<double>>(
      "covariance", std::vector<double>{1.0, 0.0, 0.0, 1.0});
    this->get_parameter("covariance", input);
    std::copy_n(input.begin(), 4, noise_cov_arr_.begin());

    // TODO(aalbaali): replace array with matrix

    try {
      noise_cov_ = Vec2ToMatrix(input);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), e.what());
    }

    rn_generator_ = std::default_random_engine();

    // Subscribe to true pose topic
    true_meas_subscriber_ = this->create_subscription<TurtlePose>(
      true_meas_topic_, 10, std::bind(&PositionSensor::MeasCallBack, this, _1));

    // Set up publisher
    noisy_meas_publisher_ = this->create_publisher<Vec3WithCovStamped>(noisy_meas_topic_, 10);
  }

private:
  void MeasCallBack(const TurtlePose::SharedPtr true_pose)
  {
    Vec3WithCovStamped noisy_meas;
    noisy_meas.header.frame_id = meas_frame_;
    noisy_meas.header.stamp = this->get_clock()->now();

    // Set true measurements
    noisy_meas.vector.vector.x = true_pose->x;
    noisy_meas.vector.vector.y = true_pose->y;
    noisy_meas.vector.vector.z = 0.0;  // z-value is ignored

    // Add noise
    // For now, assume cross-covariance between x and y is zero
    auto x_meas_noise_gaussian =
      std::normal_distribution<double>(noise_biases_[0], std::sqrt(noise_cov_arr_[0]));
    auto y_meas_noise_gaussian =
      std::normal_distribution<double>(noise_biases_[1], std::sqrt(noise_cov_arr_[3]));

    noisy_meas.vector.vector.x += x_meas_noise_gaussian(rn_generator_);
    noisy_meas.vector.vector.y += y_meas_noise_gaussian(rn_generator_);

    // Variance on x, y, theta
    double var_x = noise_cov_arr_[0];
    double var_y = noise_cov_arr_[3];
    double var_z = -1.0;
    std::array<double, 9> cov{var_x, 0.0, 0.0, 0.0, var_y, 0.0, 0.0, 0.0, var_z};
    noisy_meas.vector.covariance = cov;

    noisy_meas_publisher_->publish(noisy_meas);
  }

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

  // Standard normal distribution
  std::normal_distribution<double> standard_normal_dist_{0.0, 1.0};

  // Measurement parameters
  std::vector<double> noise_biases_;
  Matrix2d noise_cov_;
  std::array<double, 4> noise_cov_arr_;
};
}  // namespace turtle_nav_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_nav_cpp::PositionSensor>());
  rclcpp::shutdown();
  return 0;
}
