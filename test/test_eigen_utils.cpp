/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_eigen_utils.cpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */

#include <Eigen/Dense>
#include <vector>

#include "gtest/gtest.h"
#include "turtle_nav_cpp/eigen_utils.hpp"

namespace eigen_utils
{
class OrthogonalMatrices : public ::testing::Test
{
protected:
  void SetUp() override
  {
    angle_ = M_PI_4;
    rot_ = Eigen::Rotation2Dd(angle_);
    precision_ = 1e-15;
  }
  double angle_;
  Eigen::Rotation2Dd rot_;
  double precision_;
};

TEST_F(OrthogonalMatrices, IsMatrixSpecialOrthogonal)
{
  // SO(2) matrix
  EXPECT_TRUE(IsMatrixSpecialOrthogonal(rot_.toRotationMatrix(), precision_));

  // Orthogonal but det == -1 (so not SO(n))
  Eigen::Matrix2d mat;
  // clang-format off
  mat << 0, 1,
         1, 0;
  // clang-format on

  EXPECT_FALSE(IsMatrixSpecialOrthogonal(mat, precision_));

  // Random matrix
  mat(0, 0) = 10;
  mat(1, 0) = -20;
  EXPECT_FALSE(IsMatrixSpecialOrthogonal(mat, precision_));
}

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

TEST(StdArrayToMatrix, RowColMajor)
{
  // Test for the row/column major of the matrix
  std::array<double, 4> arr{1.0, 2.0, 3.0, 4.0};

  // Row-major (default storage option)
  auto mat_row_major = StdArrayToMatrix<2, 2>(arr);

  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < 2; col++) {
      auto idx = row * 2 + col;
      EXPECT_DOUBLE_EQ(arr[idx], mat_row_major(row, col));
    }
  }

  // Column-major
  auto mat_col_major = StdArrayToMatrix<2, 2, Eigen::StorageOptions::ColMajor>(arr);

  for (int row = 0; row < 2; row++) {
    for (int col = 0; col < 2; col++) {
      auto idx = row + col * 2;
      EXPECT_DOUBLE_EQ(arr[idx], mat_col_major(row, col));
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

TEST(HeadingToQuaternion, CheckQuaternion)
{
  const double heading = M_PI_4;

  // The online calculator has been used to calculate the quaternion matrix
  // https://www.andre-gaschler.com/rotationconverter/
  auto q = HeadingToQuaternion(heading);

  EXPECT_DOUBLE_EQ(q.x(), 0);
  EXPECT_DOUBLE_EQ(q.y(), 0);
  EXPECT_DOUBLE_EQ(q.z(), sin(heading / 2));
  EXPECT_DOUBLE_EQ(q.w(), cos(heading / 2));
}

TEST(HeadingToQuaternion, CheckRotationMatrix)
{
  const double heading = M_PI_4;

  auto C = HeadingToQuaternion(heading).toRotationMatrix();

  EXPECT_DOUBLE_EQ(C(0, 0), cos(heading));
  EXPECT_DOUBLE_EQ(C(0, 1), -sin(heading));
  EXPECT_DOUBLE_EQ(C(1, 0), sin(heading));
  EXPECT_DOUBLE_EQ(C(1, 1), cos(heading));
  EXPECT_DOUBLE_EQ(C(0, 2), 0);
  EXPECT_DOUBLE_EQ(C(1, 2), 0);
  EXPECT_DOUBLE_EQ(C(2, 0), 0);
  EXPECT_DOUBLE_EQ(C(2, 1), 0);
  EXPECT_DOUBLE_EQ(C(2, 2), 1);
}

TEST(QuaternionToHeading, ZeroHeadingQuaternion)
{
  Eigen::Quaterniond q;
  q.x() = 0;
  q.y() = 0;
  q.z() = 0;
  q.w() = 1;
  q.normalize();

  EXPECT_DOUBLE_EQ(QuaternionToHeading(q), 0);
}

TEST(QuaternionToHeading, NonZeroHeadingQuaternion)
{
  const double heading = M_PI_4;

  Eigen::Quaterniond q(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));

  EXPECT_DOUBLE_EQ(QuaternionToHeading(q), heading);
}

TEST(GetEllipsePoitns, GenerateEllipsePts)
{
  const double radius = 2;

  // The 5 points should be around a circle
  const auto pts = GetEllipsePoints(Eigen::Matrix2d::Identity(), radius, 5);

  // Angles
  const std::vector<double> angles{-M_PI, -M_PI_2, 0, M_PI_2, M_PI};

  for (int i = 0; i < 5; i++) {
    EXPECT_DOUBLE_EQ(pts[i](0), radius * cos(angles[i]));
    EXPECT_DOUBLE_EQ(pts[i](1), radius * sin(angles[i]));
  }
}

TEST(GetEllipsePoitns, DefaultArguments)
{
  // Number of points
  const auto pts_1 = GetEllipsePoints(Eigen::Matrix2d::Identity(), 1);
  EXPECT_EQ(pts_1.size(), static_cast<std::size_t>(100));

  // Scale
  const auto pts_2 = GetEllipsePoints(Eigen::Matrix2d::Identity());
  EXPECT_DOUBLE_EQ(pts_2[0](0), -1);
}

TEST(GetEllipsePoitns, Exceptions)
{
  // Non-positive definite matrix exception
  EXPECT_THROW(GetEllipsePoints(Eigen::Matrix2d::Random(), 1, 5), std::invalid_argument);

  // Wrong number of points
  EXPECT_THROW(GetEllipsePoints(Eigen::Matrix2d::Identity(), 1, 0), std::invalid_argument);

  // Negative scaling value
  EXPECT_THROW(GetEllipsePoints(Eigen::Matrix2d::Identity(), 0, 5), std::invalid_argument);
}
}  // namespace eigen_utils
