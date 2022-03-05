/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file utils.hpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */

#ifndef TURTLE_NAV_CPP__UTILS_HPP_
#define TURTLE_NAV_CPP__UTILS_HPP_

#include <Eigen/Dense>
#include <vector>

namespace turtle_nav_cpp
{
using Eigen::Matrix2d;

/**
* @brief Convert 2D row-major vector to a matrix
*
* @details The vector is assumed to be row-major to be consistent with ROS2 convention (e.g.,
* http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovariance.html), which
* unfortunately conflicts with Eigen's convention
*
* @param[in] vec Vector to convert. Length must be a squared number (e.g., 1,
* 4, 9, etc.)
* @return Eigen::Matrix2d
*/
Matrix2d Vec2ToMatrix(const std::vector<double> & vec);

}  // namespace turtle_nav_cpp

#endif  // TURTLE_NAV_CPP__UTILS_HPP_
