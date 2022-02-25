/**
 * Amro Al-Baali 2022
 * @file steering_wheel_encoder.cpp
 * @brief Node simulating a wheel encoder sensor
 * 
 * @details The node subscribes to the true `/cmd_vel` topic[Twist] and publishes to a new 
 *          topic[TwistWithCovarianceStamped]
 * 
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Feb-24
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include <string>
#include <memory>
#include <functional>

using std::placeholders::_1;

using Twist = geometry_msgs::msg::Twist;
using TwistWithCovStamped = geometry_msgs::msg::TwistWithCovarianceStamped;

class SteeringWheelEncoder : public rclcpp::Node {
 public:
  SteeringWheelEncoder()
  : Node("steering_wheel_encoder") {
    // Declare and acquire parameters
    //  Topic to subscribe to
    this->declare_parameter<std::string>("true_meas_topic", "true_turtle/cmd_vel");
    this->get_parameter("true_meas_topic", true_meas_topic_);

    //  Topic to publish to (noisy measurements with covariance)
    this->declare_parameter<std::string>("noisy_meas_topic", "meas/wheel_encoder");
    this->get_parameter("noisy_meas_topic", noisy_meas_topic_);

    std::stringstream ss;
    ss << "Topics: true_meas_topic: \033[92;1m'" << true_meas_topic_ << "'\033[0m"
        << "noisy_meas_topic: '\033[92;1m" << noisy_meas_topic_ << "'\033[0m";
    RCLCPP_INFO(this->get_logger(), ss.str());

    // Subscribe to true measurement topic
    true_meas_subscriber_ = this->create_subscription<Twist>(
            true_meas_topic_, 10,
            std::bind(&SteeringWheelEncoder::MeasCallBack, this, _1));
    
    // Set up publisher
    noisy_meas_publisher_ = this->create_publisher<TwistWithCovStamped>(noisy_meas_topic_, 10);
  }

 private:
  void MeasCallBack(const Twist::SharedPtr true_meas) {
    // std_msgs::msg::Header header;
    // header.frame_id

    TwistWithCovStamped noisy_meas;
    noisy_meas.header.stamp = this->get_clock()->now();    
    noisy_meas.twist.twist = *true_meas.get();

    // Variance on x, y, theta
    double var_x = 1e-1;
    double var_y = 1e-1;
    double var_theta = 1e-2;
    std::array<double, 36> cov {
        var_x, 0.0, 0.0, 0.0, 0.0, 0.0, // x
        0.0, var_y, 0.0, 0.0, 0.0, 0.0, // y
        0.0, 0.0, -1.0, 0.0, 0.0, 0.0, // z
        0.0, 0.0, 0.0, -1.0, 0.0, 0.0, // r.x
        0.0, 0.0, 0.0, 0.0, -1.0, 0.0, // r.y
        0.0, 0.0, 0.0, 0.0, 0.0, var_theta, // r.z
    };
    noisy_meas.twist.covariance = cov;

    noisy_meas_publisher_->publish(noisy_meas);
  }

  // Topic to subscribe to
  std::string true_meas_topic_;

  // Topic to publish to
  std::string noisy_meas_topic_;

  // Subscriber
  rclcpp::Subscription<Twist>::SharedPtr true_meas_subscriber_{nullptr};

  // Publisher
  rclcpp::Publisher<TwistWithCovStamped>::SharedPtr noisy_meas_publisher_{nullptr};
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SteeringWheelEncoder>());
  rclcpp::shutdown();
  return 0;
}
