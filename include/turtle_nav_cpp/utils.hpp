/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file utils.hpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */

#ifndef TURTLE_NAV_CPP_INCLUDE_TURTLE_NAV_CPP_UTILS_HPP_
#define TURTLE_NAV_CPP_INCLUDE_TURTLE_NAV_CPP_UTILS_HPP_

#include <Eigen/Dense>
#include <vector>

namespace turtle_nav_cpp
{
using Eigen::Matrix2d;

/**
* @brief Convert 2D row-major vector to a matrix
*
* @param[in] vec Vector to convert. Length must be a squared number (e.g., 1,
* 4, 9, etc.)
* @return Eigen::Matrix2d
*/
Matrix2d Vec2ToMatrix(const std::vector<double> & vec);

}

#endif // TURTLE_NAV_CPP_INCLUDE_TURTLE_NAV_CPP_UTILS_HPP_
