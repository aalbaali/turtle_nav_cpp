/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file math_utils.cpp
 * @brief Implementation of math_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-16
 */

#include "turtle_nav_cpp/math_utils.hpp"

#include <random>
#include <stdexcept>
#include <vector>

namespace turtle_nav_cpp
{
bool IsPerfectSquare(double x)
{
  double x_sqrt = sqrt(x);
  return (x_sqrt - floor(x_sqrt)) == 0;
}

double randn_gen(std::default_random_engine & rn_generator)
{
  return standard_normal_dist(rn_generator);
}

double WrapToPi(double angle)
{
  // Multiplying 2 by PI loses some precision, thus the `double` output loses some precision
  // The remedy is to use `long double` and return a double
  long double angle_l = angle;
  while (angle_l > M_PI) {
    angle_l -= 2 * M_PIl;
  }
  while (angle_l <= -M_PI) {
    angle_l += 2 * M_PIl;
  }

  return angle_l;
}

std::vector<double> linspace(const double val_first, const double val_last, const size_t num_points)
{
  if (num_points < 2) {
    throw std::invalid_argument("Number of points should be greater than 1");
  }
  if (val_last <= val_first) {
    throw std::invalid_argument("val_last should be greater than the val_first");
  }

  const double dv = (val_last - val_first) / static_cast<double>(num_points - 1);
  std::vector<double> vals(num_points);
  std::generate(
    vals.begin(), vals.end(), [i = 0, dv, val_first]() mutable { return val_first + dv * i++; });

  return vals;
}

}  // namespace turtle_nav_cpp
