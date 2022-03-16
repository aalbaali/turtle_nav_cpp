/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file ros_utils.hpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */

#ifndef TURTLE_NAV_CPP_ROS_UTILS_HPP_
#define TURTLE_NAV_CPP_ROS_UTILS_HPP_

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "turtle_nav_cpp/eigen_utils.hpp"

namespace turtle_nav_cpp
{
using Eigen::Matrix2d;
using Eigen::Vector2d;

/**
 * @brief Check if a number is a perfect square (i.e., if there exists an integer i such that x =
 * i*i)
 *
 * @param x Number to check for perfect square
 */
bool IsPerfectSquare(double x);

/**
 * @brief Declare and import a ROS2 parameter as an Eigen type
 *
 * @tparam row Number of rows of a matrix
 * @tparam col Number of columns of a matrix
 * @param nh Node handle for extracting the parameters
 * @param param_name Parameter name to extract
 * @param default_val Default Eigen-type value
 * @return Eigen::Matrix<double, col, 1>
 */
template <int row, int col = 1>
Eigen::Matrix<double, row, col> ImportParamAsEigen(
  rclcpp::Node * nh, const std::string & param_name,
  const Eigen::Matrix<double, row, col> & default_val)
{
  int sz = row * col;

  // The default values take `std::vector<double>`, which is obtained from `Eigen::Vector` using the
  // `.data()` method
  std::vector<double> default_val_std_vec;
  default_val_std_vec.reserve(sz);
  default_val_std_vec.assign(default_val.data(), default_val.data() + sz);

  // Store the imported vector in a temporary `std::vector` object
  std::vector<double> input;
  input.reserve(sz);
  nh->declare_parameter<std::vector<double>>(param_name, default_val_std_vec);
  nh->get_parameter(param_name, input);
  return Eigen::Map<Eigen::Matrix<double, row, col>>(input.data());
}

}  // namespace turtle_nav_cpp

#endif  // TURTLE_NAV_CPP_ROS_UTILS_HPP_
