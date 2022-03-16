/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file eigen_utils.hpp
 * @brief Helper functions related to the Eigen library. This translational unit should be
 *        independent of ROS
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-16
 */
#ifndef TURTLE_NAV_CPP_EIGEN_UTILS_HPP_
#define TURTLE_NAV_CPP_EIGEN_UTILS_HPP_

#include <Eigen/Dense>
#include <vector>

namespace turtle_nav_cpp
{
namespace eigen_utils
{
using Eigen::Matrix2d;

// TODO(aalbaali): remove
/**
* @brief Convert 2D row-major vector to a matrix
*
* @details The vector is assumed to be row-major to be consistent with ROS2 convention (e.g.,
* http://docs.ros.org/en/lunar/api/geometry_msgs/html/msg/PoseWithCovariance.html), which
* unfortunately conflicts with Eigen's convention
*
* @param[in] vec Vector to convert. Length must be a squared number (e.g., 1,
* 4, 9, etc.)
* @return Eigen::Matrix2d
*/
Matrix2d Vec2ToMatrix(const std::vector<double> & vec);

/**
 * @brief Convert std::vector<double> to an Eigen type (matrix or vector) with the appropriate
 * dimensions
 *
 * @details The matrix size (i.e., row * col) should match the std::vector size
 *
 * @tparam row      Number of rows of the matrix
 * @tparam col      Number of columns of the matrix
 * @tparam storage_opt Store matrix as column major or row major. Default is row-major to match ROS2
 *         convention
 * @param[in] vec   Vector to convert
 * @return Eigen::Matrix<double, row, col>
 */
template <int row, int col = 1, int storage_opt = Eigen::StorageOptions::RowMajor>
Eigen::Matrix<double, row, col, storage_opt> StdVectorToEigenVector(std::vector<double> & vec)
{
  // Throw error if sizes don't match
  if (vec.size() != row * col) {
    std::stringstream ss;
    ss << "Provided vector size (" << vec.size() << ") does not match Eigen matrix sizes: (" << row
       << ", " << col << ")";
    throw std::length_error(ss.str());
  }

  return Eigen::Map<Eigen::Matrix<double, row, col>>(vec.data());
}

/**
   * @brief Import lower matrix of a Cholesky decomposition and throw an error if the matrix is non
   * semi-positive definite
   *
   * @param[in] matrix Symmetric positive (semi-) definite matrix
   * @return const Matrix2d Lower triangular matrix of a LL^{trans} Cholesky factorization
   */
const Matrix2d GetCholeskyLower(const Matrix2d & matrix);

}  // namespace eigen_utils
}  // namespace turtle_nav_cpp

#endif  // TURTLE_NAV_CPP_EIGEN_UTILS_HPP_
