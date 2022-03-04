/**
 * Copyright Amro Al-Baali 2022
 * @file steering_wheel_encoder.cpp
 * @brief Node simulating a wheel encoder sensor
 *
 * @details The node subscribes to the true `/cmd_vel` topic[Twist] and publishes to a new
 *          topic[TwistWithCovarianceStamped]
 *
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Feb-24
 */

#include <array>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <map>
#include <memory>
#include <random>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

using std::placeholders::_1;

using Twist = geometry_msgs::msg::Twist;
using TwistWithCovStamped = geometry_msgs::msg::TwistWithCovarianceStamped;

// String to double map to extract Gaussian params
using StrToDoubleMap = std::map<std::string, double>;

// (mean, std)
using GaussianParamsVec = std::vector<double>;

class SteeringWheelEncoder : public rclcpp::Node
{
public:
  SteeringWheelEncoder()
  : Node("steering_wheel_encoder")
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

    GaussianParamsVec gaussian_params;

    //  Linear speed Gaussian PDF params (mean, std)
    //  Note that a non-zero mean value implies the sensor is biased
    this->declare_parameter("linear_speed_noise_params", GaussianParamsVec{0.0, 1.0});
    this->get_parameter("linear_speed_noise_params", gaussian_params);
    linear_speed_noise_gaussian_ =
      std::normal_distribution<double>(gaussian_params[0], gaussian_params[1]);

    //  Angular speed Gaussian PDF params (mean, std)
    //  Note that a non-zero mean value implies the sensor is biased
    gaussian_params.clear();
    this->declare_parameter("angular_speed_noise_params", GaussianParamsVec{0.0, 1.0});
    this->get_parameter("angular_speed_noise_params", gaussian_params);
    angular_speed_noise_gaussian_ =
      std::normal_distribution<double>(gaussian_params[0], gaussian_params[1]);

    std::stringstream ss;
    ss << "Map params. mu: \033[92;1m" << gaussian_params[0] << "\033[0m" <<
      ", std: \033[92;1m" << gaussian_params[1] << "\033[0m";
    RCLCPP_INFO(this->get_logger(), ss.str());

    rn_generator_ = std::default_random_engine();

    // Subscribe to true measurement topic
    true_meas_subscriber_ = this->create_subscription<Twist>(
      true_meas_topic_, 10, std::bind(&SteeringWheelEncoder::MeasCallBack, this, _1));

    // Set up publisher
    noisy_meas_publisher_ = this->create_publisher<TwistWithCovStamped>(noisy_meas_topic_, 10);
  }

private:
  void MeasCallBack(const Twist::SharedPtr true_meas)
  {
    TwistWithCovStamped noisy_meas;
    noisy_meas.header.frame_id = meas_frame_;
    noisy_meas.header.stamp = this->get_clock()->now();
    noisy_meas.twist.twist = *true_meas.get();

    // Add noise
    noisy_meas.twist.twist.linear.x += linear_speed_noise_gaussian_(rn_generator_);
    noisy_meas.twist.twist.angular.z += angular_speed_noise_gaussian_(rn_generator_);

    // Variance on x, y, theta
    double var_x = std::pow(linear_speed_noise_gaussian_.stddev(), 2);
    double var_y = -1.0;
    double var_theta = std::pow(angular_speed_noise_gaussian_.stddev(), 2);
    std::array<double, 36> cov{
      var_x, 0.0, 0.0, 0.0, 0.0, 0.0,             // x
      0.0, var_y, 0.0, 0.0, 0.0, 0.0,             // y
      0.0, 0.0, -1.0, 0.0, 0.0, 0.0,              // z
      0.0, 0.0, 0.0, -1.0, 0.0, 0.0,              // r.x
      0.0, 0.0, 0.0, 0.0, -1.0, 0.0,              // r.y
      0.0, 0.0, 0.0, 0.0, 0.0, var_theta,         // r.z
    };
    noisy_meas.twist.covariance = cov;

    noisy_meas_publisher_->publish(noisy_meas);
  }

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringWheelEncoder>());
  rclcpp::shutdown();
  return 0;
}
