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
TEST(StdVectorToMatrix, RowColMajor)
{
  // Test for the row/column major of the matrix
  std::vector<double> v{1.0, 2.0, 3.0, 4.0};

  // Row-major
  auto mat_row_major = StdVectorToMatrix<2, 2, Eigen::StorageOptions::RowMajor>(v);

  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < 2; col++) {
      auto idx = row * 2 + col;
      EXPECT_DOUBLE_EQ(v[idx], mat_row_major(row, col));
    }
  }

  // Column-major
  auto mat_col_major = StdVectorToMatrix<2, 2, Eigen::StorageOptions::ColMajor>(v);

  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < 2; col++) {
      auto idx = row + col * 2;
      EXPECT_DOUBLE_EQ(v[idx], mat_col_major(row, col));
    }
  }
}

/**
 * @brief Array shape should match the row major matrix data type
 *
 */
TEST(MatrixToStdArray, RowMajorMatrix)
{
  // Vector to initialize matrix (using the `Eigen::Map` function)
  std::vector<double> init_vals{1, 2, 3, 4, 5, 6};

  // Matrix size
  const int rows = 2;
  const int cols = 3;

  // Matrix data type. Note that it's row major in this test
  using MatrixType = Eigen::Matrix<double, rows, cols, Eigen::StorageOptions::RowMajor>;

  MatrixType mat = Eigen::Map<MatrixType>(init_vals.data());

  // Get the array
  const auto arr_row_major = MatrixToStdArray(mat);

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      EXPECT_DOUBLE_EQ(arr_row_major[cols * i + j], mat(i, j));
    }
  }
}

/**
 * @brief Array shape should match the column major matrix data type
 *
 */
TEST(MatrixToStdArray, ColMajorMatrix)
{
  // Vector to initialize matrix (using the `Eigen::Map` function)
  std::vector<double> init_vals{1, 2, 3, 4, 5, 6};

  // Matrix size
  const int rows = 2;
  const int cols = 3;

  // Matrix data type. Note that it's row major in this test
  using MatrixType = Eigen::Matrix<double, rows, cols, Eigen::StorageOptions::ColMajor>;

  MatrixType mat = Eigen::Map<MatrixType>(init_vals.data());

  // Get the array
  const auto arr_row_major = MatrixToStdArray(mat);

  for (int i = 0; i < rows; i++) {
    for (int j = 0; j < cols; j++) {
      EXPECT_DOUBLE_EQ(arr_row_major[i + rows * j], mat(i, j));
    }
  }
}

/**
 * @brief Arrays from eigen vectors should be independent of their storage type
 *
 */
TEST(MatrixToStdArray, EigVector)
{
  // Vector to initialize matrix (using the `Eigen::Map` function)
  std::vector<double> init_vals{1, 2, 3, 4, 5, 6};

  // Matrix size
  const int sz = 6;

  // Matrix data type. Note that it's row major in this test
  using MatrixRowMajor = Eigen::Matrix<double, 1, sz, Eigen::StorageOptions::RowMajor>;
  using MatrixColMajor = Eigen::Matrix<double, sz, 1, Eigen::StorageOptions::ColMajor>;

  // Both matrices should be the same (i.e., when accessing coefficients)
  // The difference is in the way the underlying data is stored
  MatrixRowMajor mat_row_major = Eigen::Map<MatrixRowMajor>(init_vals.data());
  MatrixColMajor mat_col_major = Eigen::Map<MatrixColMajor>(init_vals.data());

  // Get the array
  const auto arr_row_major = MatrixToStdArray(mat_row_major);
  const auto arr_col_major = MatrixToStdArray(mat_col_major);

  for (int i = 0; i < sz; i++) {
    EXPECT_DOUBLE_EQ(arr_row_major[i], mat_row_major(i));
    EXPECT_DOUBLE_EQ(arr_row_major[i], mat_col_major(i));
    EXPECT_DOUBLE_EQ(arr_col_major[i], mat_row_major(i));
    EXPECT_DOUBLE_EQ(arr_col_major[i], mat_col_major(i));
  }
}
}  // namespace eigen_utils
