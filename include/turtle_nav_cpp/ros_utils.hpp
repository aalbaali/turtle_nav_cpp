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
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "turtle_nav_cpp/eigen_utils.hpp"

namespace turtle_nav_cpp
{
namespace ros_utils
{
using Eigen::Matrix2d;
using Eigen::Vector2d;

/**
 * @brief Declare and import ROS2 parameters
 *
 * @details This is especially useful for importing parameters into `const` variables in the
 *          construction initialization list
 *
 * @tparam T                  Parameter type
 * @param[in] nh              Node handle
 * @param[in] param_name      Parameter name from the ROS2 parameter server
 * @param[in] default_value   Default value
 * @return T                  Imported value, if found, or default value otherwise
 */
template <typename T>
T DeclareAndImportParam(
  rclcpp::Node * const nh, const std::string & param_name, const T & default_value)
{
  T imported_value;
  nh->declare_parameter<T>(param_name, default_value);
  nh->get_parameter(param_name, imported_value);

  return imported_value;
}

/**
 * @brief Declare and import a ROS2 parameter as an Eigen type
 *
 * @tparam row Number of rows of a matrix
 * @tparam col Number of columns of a matrix
 * @param nh Node handle for extracting the parameters
 * @param param_name Parameter name to extract
 * @param default_val Default Eigen-type value
 * @return Eigen::Matrix<double, row, col>
 */
template <const int row, int col = 1, int storage_opt = Eigen::StorageOptions::RowMajor>
Eigen::Matrix<double, row, col> ImportParamAsEigen(
  rclcpp::Node * const nh, const std::string & param_name,
  const Eigen::Matrix<double, row, col> & default_val)
{
  const int sz = row * col;

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

  // TODO(aalbaali): Using `eigen_utils::StdVectorToMatrix<row, col, storage_opt>(input)` is causing
  // issues. Specifically, it's the choice of `col`.

  return Eigen::Map<Eigen::Matrix<double, row, col>>(input.data());
}

/**
 * @brief Convert a 2D vector (i.e., a point) into a Point32 message
 *
 * @param[in] point Eigen 2D double vector
 * @return geometry_msgs::msg::Point32 Point geometry message
 */
geometry_msgs::msg::Point32 PointToPoint32Msg(const Eigen::Vector2d & point);

/**
 * @brief Convert a vector of points to a Polygon geometry msg
 *
 * @param[in] points      Points to set to a polygon message
 * @param[in] num_points  Number of points to generate
 * @return geometry_msgs::msg::Polygon
 */
geometry_msgs::msg::Polygon PointsToPolygon(
  const std::vector<Vector2d> & points, const int num_points);

}  // namespace ros_utils
}  // namespace turtle_nav_cpp

#endif  // TURTLE_NAV_CPP_ROS_UTILS_HPP_
