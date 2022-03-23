/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file math_utils.cpp
 * @brief Implementation of math_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-16
 */

#include "turtle_nav_cpp/math_utils.hpp"

#include <random>

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

}  // namespace turtle_nav_cpp
