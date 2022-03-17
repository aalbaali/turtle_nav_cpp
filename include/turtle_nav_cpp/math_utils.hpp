/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file math_utils.hpp
 * @brief Math helper functions. This should be independent of ROS
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-16
 */
#ifndef TURTLE_NAV_CPP_MATH_UTILS_HPP_
#define TURTLE_NAV_CPP_MATH_UTILS_HPP_

#include <random>

namespace turtle_nav_cpp
{
/**
 * @brief Check if a number is a perfect square (i.e., if there exists an integer i such that x =
 * i*i)
 *
 * @param x Number to check for perfect square
 */
bool IsPerfectSquare(double x);

// Single-variable standard normal distribution
std::normal_distribution<double> standard_normal_dist{0.0, 1.0};
}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_MATH_UTILS_HPP_
