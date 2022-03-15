/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file utils.cpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */
#include "turtle_nav_cpp/utils.hpp"

#include <Eigen/Dense>
#include <iostream>
#include <vector>

namespace turtle_nav_cpp
{
Matrix2d Vec2ToMatrix(const std::vector<double> & vec)
{
  // Length should be 4 to get a 2x2 matrix
  // https://stackoverflow.com/questions/17036818/initialise-eigenvector-with-stdvector
  if (vec.size() != 4) {
    std::stringstream ss;
    ss << "Vector size must be 4. Provided size: " << vec.size();
    throw std::length_error(ss.str());
  }

  return Eigen::Map<const Eigen::Matrix<double, 2, 2>>(vec.data());
}

bool IsPerfectSquare(double x)
{
  double x_sqrt = sqrt(x);
  return (x_sqrt - floor(x_sqrt)) == 0;
}

}  // namespace turtle_nav_cpp
