/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file eigen_utils.cpp
 * @brief Implementation of eigen_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-16
 */

#include "turtle_nav_cpp/eigen_utils.hpp"

#include <algorithm>
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

const std::vector<Eigen::Vector2d> GetEllipsePoints(
  const Eigen::Matrix2d & mat, const double scale /* = 1 */, const int num_points /* = 100 */)
{
  // Ensure symmetry
  if (!mat.isApprox(mat.transpose())) {
    throw std::invalid_argument("Non symmetric matrix");
  }

  // Get Cholesky factorization
  const Eigen::LLT<Eigen::Matrix2d> mat_llt = mat.llt();

  // Ensure scale is positive number
  if (scale <= 0) {
    throw std::invalid_argument("Scale should be a positive number");
  }

  // Ensure positive definiteness
  if (mat_llt.info() == Eigen::NumericalIssue) {
    throw std::invalid_argument("Covariance matrix possibly non semi-positive definite matrix");
  }

  // Get lower Cholesky factor
  const Eigen::Matrix2d mat_chol_lower = mat_llt.matrixL();

  // Compute points
  std::vector<Eigen::Vector2d> ellipse_points(num_points);
  const std::vector<double> thetas = turtle_nav_cpp::linspace(-M_PI, M_PI, num_points);
  std::transform(thetas.begin(), thetas.end(), ellipse_points.begin(), [&](const double th) {
    const Eigen::Vector2d v{cos(th), sin(th)};
    Eigen::Vector2d p = scale * mat_chol_lower * v;
    return p;
  });

  return ellipse_points;
}

}  // namespace eigen_utils
