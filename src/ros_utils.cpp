/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file ros_utils.cpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */
#include "turtle_nav_cpp/ros_utils.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace turtle_nav_cpp
{
bool IsPerfectSquare(double x)
{
  double x_sqrt = sqrt(x);
  return (x_sqrt - floor(x_sqrt)) == 0;
}

}  // namespace turtle_nav_cpp
