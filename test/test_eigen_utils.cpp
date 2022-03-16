/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_eigen_utils.cpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */

#include <vector>

#include "gtest/gtest.h"
#include "turtle_nav_cpp/eigen_utils.hpp"

namespace eigen_utils
{
TEST(StdVectorToEigenVector, RowColMajor)
{
  // Test for the row/column major of the matrix
  std::vector<double> v{1.0, 2.0, 3.0, 4.0};

  // Row-major
  auto mat_row_major = StdVectorToEigenVector<2, 2, Eigen::StorageOptions::RowMajor>(v);

  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < 2; col++) {
      auto idx = row * 2 + col;
      EXPECT_DOUBLE_EQ(v[idx], mat_row_major(row, col));
    }
  }

  // Column-major
  auto mat_col_major = StdVectorToEigenVector<2, 2, Eigen::StorageOptions::ColMajor>(v);

  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < 2; col++) {
      auto idx = row + col * 2;
      EXPECT_DOUBLE_EQ(v[idx], mat_col_major(row, col));
    }
  }
}

}  // namespace eigen_utils
