/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file math_utils.cpp
 * @brief Implementation of math_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-16
 */

#include "turtle_nav_cpp/math_utils.hpp"

namespace turtle_nav_cpp
{
bool IsPerfectSquare(double x)
{
  double x_sqrt = sqrt(x);
  return (x_sqrt - floor(x_sqrt)) == 0;
}

}  // namespace turtle_nav_cpp
