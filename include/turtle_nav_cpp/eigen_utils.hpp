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
#include <array>
#include <vector>

namespace eigen_utils
{
using Eigen::Matrix2d;

/**
 * @brief Check if a symmetric matrix belongs to the special orthogonal group SO(n)
 *
 * @details Matrix should be unitary (or orthogonal since it's symmetric) and has determinant = 1
 *
 * @tparam sz Dimension of the matrix
 * @param[in] mat Matrix to be assessed
 * @param[in] precision Precision for the comparison
 * @return bool
 */
template <int sz>
bool IsMatrixSpecialOrthogonal(const Eigen::Matrix<double, sz, sz> & mat, double precision = 1e-10)
{
  // Matrix should be unitary
  bool cond_1 = mat.isUnitary(precision);

  // Determinant should be equal to 1 (a counter example is [0 1; 1 0])
  bool cond_2 = abs(mat.determinant() - 1) < precision;

  return cond_1 && cond_2;
}

/**
 * @brief Get angle from a 2x2 rotation matrix
 *
 * @param[in] rot 2x2 matrix that belongs to SO(2)
 * @param[in] precision Precision for the comparison, defaults to 1e-10
 * @return double Angle wrapped to (-pi, pi]
 */
double RotationMatrixToAngle(const Eigen::Matrix2d & rot, double precision = 1e-10);

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
Eigen::Matrix<double, row, col, storage_opt> StdVectorToMatrix(const std::vector<double> & vec)
{
  // Throw error if sizes don't match
  if (vec.size() != row * col) {
    std::stringstream ss;
    ss << "Provided vector size (" << vec.size() << ") does not match Eigen matrix sizes: (" << row
       << ", " << col << ")";
    throw std::length_error(ss.str());
  }

  return Eigen::Map<const Eigen::Matrix<double, row, col, storage_opt>>(vec.data());
}

template <int row, int col = 1, int storage_opt = Eigen::StorageOptions::RowMajor>
Eigen::Matrix<double, row, col, storage_opt> StdArrayToMatrix(
  const std::array<double, row * col> & arr)
{
  return Eigen::Map<const Eigen::Matrix<double, row, col, storage_opt>>(arr.data());
}

/**
 * @brief Convert Eigen matrix to a std::array<double>
 *
 * @tparam row Number of matrix rows
 * @tparam col Number of matrix columns
 * @tparam storage_opt Eigen matrix storate options (row or column major)
 * @param[in] mat Matrix to be converted
 * @return std::array<double, row * col>
 */
template <int row, int col, int storage_opt = Eigen::StorageOptions::RowMajor>
std::array<double, row * col> MatrixToStdArray(
  const Eigen::Matrix<double, row, col, storage_opt> & mat)
{
  // For now, copy the data (since it's a const ref), but create another function with a move
  // reference

  // Answer from
  // https://stackoverflow.com/questions/8443102/convert-eigen-matrix-to-c-array
  std::array<double, row * col> arr;
  Eigen::Map<Eigen::Matrix<double, row, col, storage_opt>>(arr.data()) = mat;
  return arr;
}

/**
 * @brief Import lower matrix of a Cholesky decomposition and throw an error if the matrix is non
 * semi-positive definite
 *
 * @param[in] matrix Symmetric positive (semi-) definite matrix
 * @return const Matrix2d Lower triangular matrix of a LL^{trans} Cholesky factorization
 */
const Matrix2d GetCholeskyLower(const Matrix2d & matrix);

/**
 * @brief Convert heading to Eigen quaternion
 *
 * @tparam T
 * @param[in] heading
 * @return Eigen::Quaternion<T>
 */
template <typename T>
Eigen::Quaternion<T> HeadingToQuaternion(T heading)
{
  return Eigen::Quaternion<T>(Eigen::AngleAxis<T>(heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
}

template <typename T>
T QuaternionToHeading(const Eigen::Quaternion<T> & q)
{
  // For planar rotation, the quaternion should have the form: (0, 0, cos(th/2)) + sin(th/2)
  return atan2(q.z(), q.w()) * 2;
}

}  // namespace eigen_utils

#endif  // TURTLE_NAV_CPP_EIGEN_UTILS_HPP_
