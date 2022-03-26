/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file eigen_utils.cpp
 * @brief Implementation of eigen_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-16
 */

#include "turtle_nav_cpp/eigen_utils.hpp"

#include <vector>

#include "turtle_nav_cpp/math_utils.hpp"

namespace eigen_utils
{
double RotationMatrixToAngle(const Eigen::Matrix2d & rot, double precision /* = 1e-10 */)
{
  if (!IsMatrixSpecialOrthogonal(rot, precision)) {
    throw std::invalid_argument("Matrix is not a SO(2) matrix");
  }

  double angle = atan2(rot(1, 0), rot(0, 0));

  // Ensure it's wrapped to the same angle range
  return turtle_nav_cpp::WrapToPi(angle);
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
