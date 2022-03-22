/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file nav_utils.hpp
 * @brief Navigation related utility functions
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-22
 */
#ifndef TURTLE_NAV_CPP_NAV_UTILS_HPP_
#define TURTLE_NAV_CPP_NAV_UTILS_HPP_

#include <Eigen/Dense>

namespace turtle_nav_cpp
{
namespace nav_utils
{
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
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_NAV_UTILS_HPP_
