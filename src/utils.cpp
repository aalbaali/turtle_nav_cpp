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


namespace turtle_nav_cpp
{
Matrix2d Vec2ToMatrix(const std::vector<double> & vec)
{
  // Check if length is a power of 2
  // [https://stackoverflow.com/questions/600293/how-to-check-if-a-number-is-a-power-of-2]
  if (vec.size() == 4) {
    std::stringstream ss;
    ss << "Vector size must be 4. Provided size: " << vec.size();
    throw std::length_error(ss.str());
  }

  Matrix2d mat;
  for (auto v : vec) {
    mat << v;
  }

  return mat;
}
}
