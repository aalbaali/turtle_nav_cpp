/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file math_utils.hpp
 * @brief Math helper functions. This should be independent of ROS
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-16
 */
#ifndef TURTLE_NAV_CPP_MATH_UTILS_HPP_
#define TURTLE_NAV_CPP_MATH_UTILS_HPP_

#include <algorithm>
#include <random>
#include <vector>

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
static std::normal_distribution<double> standard_normal_dist{0.0, 1.0};

/**
 * @brief Scalar standard normal distribution generator
 *
 * @details The `rn_generator` must be passed by reference
 *
 * @param[in] rn_generator Random number generator
 * @return double Sampled random number
 */
double randn_gen(std::default_random_engine & rn_generator);

/**
 * @brief Wrap angle to (-pi, pi]
 *
 * @param angle To to wrap to (-pi, pi)
 * @return double
 */
double WrapToPi(double angle);

/**
 * @brief Generate a vector of linearly spaced points
 *
 * @details Uses `std::generate`, which requires C++17
 *
 * @param[in] val_first   First value of the range
 * @param[in] val_last    Last value of the range > val_first
 * @param[in] num_points  Number of points to generate > 1
 * @return std::vector<T> Vector of points
 */
std::vector<double> linspace(
  const double val_first, const double val_last, const size_t num_points);

}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_MATH_UTILS_HPP_
