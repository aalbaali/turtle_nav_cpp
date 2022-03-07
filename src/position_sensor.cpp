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
using Eigen::Matrix2d;
using Eigen::Vector2d;

class PositionSensor : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Position Sensor object
   *
   * @details `noise_biases` and `noise_cov_` are initialized in the initializer list because they
   * are const objects
   *
   */
  PositionSensor()
  : Node("position_sensor"),
    noise_biases_(ImportNoiseBiases("biases", Vector2d::Zero())),
    noise_cov_(ImportNoiseCovariance("covariance", Matrix2d::Identity())),
    noise_cov_chol_L_(noise_cov_)
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
      true_meas_topic_, 10, std::bind(&PositionSensor::MeasCallBack, this, _1));

    // Set up publisher
    noisy_meas_publisher_ = this->create_publisher<Vec3WithCovStamped>(noisy_meas_topic_, 10);

    // Set the random number generators and the randn_ lambda function
    rn_generator_ = std::default_random_engine();
    randn_ = [this]() { return standard_normal_dist_(rn_generator_); };
  }

private:
  /**
   * @brief Measurement call-back function that publishes a noise measured position
   *
   * @param[in] true_pose The true pose from which the true position is extracted
   */
  void MeasCallBack(const TurtlePose::SharedPtr true_pose)
  {
    Vec3WithCovStamped noisy_meas;
    noisy_meas.header.frame_id = meas_frame_;
    noisy_meas.header.stamp = this->get_clock()->now();

    // Set true position
    noisy_meas.vector.vector.x = true_pose->x;
    noisy_meas.vector.vector.y = true_pose->y;
    noisy_meas.vector.vector.z = 0.0;  // z-value is ignored

    // Sample and add noise
    auto noise = noise_biases_ + noise_cov_chol_L_ * (Vector2d::NullaryExpr(2, randn_));
    noisy_meas.vector.vector.x += noise(0);
    noisy_meas.vector.vector.y += noise(1);

    // Variance on x, y, theta
    double var_x = noise_cov_(0, 0);
    double var_y = noise_cov_(1, 1);
    double var_z = -1.0;
    std::array<double, 9> cov{var_x, 0.0, 0.0, 0.0, var_y, 0.0, 0.0, 0.0, var_z};
    noisy_meas.vector.covariance = cov;

    noisy_meas_publisher_->publish(noisy_meas);
  }

  /**
   * @brief Import measurement noise biases from the config aprams
   *
   * @details This function can be generalized in used in `utils.{hpp, cpp}`
   *
   * @param[in] param_name Parameter name in the config params
   * @param[in] default_val Default value if parameter not found
   * @return Eigen::Vector2d Measurement noise biases
   */
  const Vector2d ImportNoiseBiases(const std::string & param_name, const Vector2d & default_val)
  {
    // Vector size
    const size_t vec_size = default_val.size();

    // The default values take `std::vector<double>`, which is obtained from `Eigen::Vector2d` using
    // the `.data()` method
    std::vector<double> default_val_std_vec;
    default_val_std_vec.assign(default_val.data(), default_val.data() + vec_size);

    // Store the imported biases in a temporary `std::vector` object
    std::vector<double> input;
    input.reserve(vec_size);
    this->declare_parameter<std::vector<double>>(param_name, default_val_std_vec);
    this->get_parameter(param_name, input);
    return Eigen::Map<Vector2d>(input.data());
  }

  const Matrix2d ImportNoiseCovariance(const std::string & param_name, const Matrix2d & default_val)
  {
    // Matrix size
    const size_t mat_size = default_val.size();

    // The default values take `std::vector<double>`, which is obtained from `Eigen::Matrix2d` using
    // the `.data()` method
    std::vector<double> default_val_std_vec;
    default_val_std_vec.assign(default_val.data(), default_val.data() + default_val.size());

    //  Get covariance biases
    std::vector<double> input;
    input.clear();
    input.reserve(mat_size);
    this->declare_parameter<std::vector<double>>(param_name, default_val_std_vec);
    this->get_parameter(param_name, input);

    // Copy to Eigen matrix and return
    return Vec2ToMatrix(input);
  }

  /**
   * @brief Import lower matrix of a Cholesky decomposition and throw an error if the matrix is non
   * semi-positive definite
   *
   * @param[in] matrix Symmetric positive (semi-) definite matrix
   * @return const Matrix2d Lower triangular matrix of a LL^{trans} Cholesky factorization
   */
  const Matrix2d GetCholeskyLower(const Matrix2d & matrix)
  {
    // Cholesky factorization
    const Eigen::LLT<Eigen::Matrix2d> matrix_llt(matrix);

    // Check for covariance positive semi-definiteness
    if (matrix_llt.info() == Eigen::NumericalIssue) {
      throw std::runtime_error("Covariance matrix possibly non semi-positive definite matrix");
    }

    // Return lower triangular part only
    return matrix_llt.matrixL();
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

  // Scalar standard normal distribution (i.e., mean 0 and variance 1)
  std::normal_distribution<double> rn_standard_normal_distribution_;

  // Standard normal distribution
  // TODO(aalbaali): Add this and randn function to `utils.{hpp, cpp}`.
  std::normal_distribution<double> standard_normal_dist_{0.0, 1.0};

  // Scalar function that returns a sample from a standard normal distribution
  std::function<double()> randn_;

  // Measurement parameters
  Vector2d noise_biases_;
  Matrix2d noise_cov_;

  // Covariance cholesky decomposition, lower triangular matrix
  Matrix2d noise_cov_chol_L_;
};
}  // namespace turtle_nav_cpp

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<turtle_nav_cpp::PositionSensor>());
  rclcpp::shutdown();
  return 0;
}
