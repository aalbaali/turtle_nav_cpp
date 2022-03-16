/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file eigen_utils.cpp
 * @brief Implementation of eigen_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-16
 */

#include "turtle_nav_cpp/eigen_utils.hpp"

#include <vector>

namespace turtle_nav_cpp
{
namespace eigen_utils
{
// TODO(aalbaali): obsolete
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

const Matrix2d GetCholeskyLower(const Matrix2d & matrix)
{
  // Cholesky factorization
  const Eigen::LLT<Eigen::Matrix2d> matrix_llt(matrix);

  // Check for covariance positive semi-definiteness
  if (matrix_llt.info() == Eigen::NumericalIssue) {
    throw std::runtime_error("Covariance matrix possibly non semi-positive definite matrix");
  }

  // Return lower triangular part only
  return matrix_llt.matrixL();
}

}  // namespace eigen_utils
}  // namespace turtle_nav_cpp
