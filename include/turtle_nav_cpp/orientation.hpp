/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file orientation.hpp
 * @brief Class for dealing with SO(2) headings
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-21
 */
#ifndef TURTLE_NAV_CPP_ORIENTATION_HPP_
#define TURTLE_NAV_CPP_ORIENTATION_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <iostream>
#include <turtlesim/msg/pose.hpp>

namespace turtle_nav_cpp
{
namespace nav_utils
{
class Orientation
{
public:
  /**
   * @brief Default constructor
   *
   */
  Orientation();

  /**
   * @brief Construct a new Orientation object
   *
   * @param[in] heading Orientation to insert
   *
   * @details The heading is automatically wrapped to (-pi, pi]
   */
  Orientation(double heading);

  /**
   * @brief Construct a new Orientation object from an Eigen quaternion object
   *
   * @param[in] q Eigen quaternion
   */
  Orientation(const Eigen::Quaterniond q);

  /**
   * @brief Construct a new Orientation object
   *
   * @param[in] q ROS `geometry_msgs` quaternion object
   */
  Orientation(const geometry_msgs::msg::Quaternion q);

  /**
   * @brief Return the heading angle wrapped to (-pi, pi]
   *
   * @return double
   */
  double Angle() const;

  /**
   * @brief Return Eigen's rotation object
   *
   * @return Eigen::Rotation2Dd
   */
  Eigen::Rotation2Dd Rotation() const;

  /**
   * @brief Return rotation matrix
   *
   * @return Eigen::Matrix2d
   */
  Eigen::Matrix2d RotationMatrix() const;

  /**
   * @brief Return Eigen quaternion
   *
   * @return Eigen::Quaterniond
   */
  Eigen::Quaterniond Quaternion() const;

  /**
   * @brief Return ROS geometry quaternion msg
   *
   * @return geometry_msgs::msg::Quaternion
   */
  geometry_msgs::msg::Quaternion QuaternionMsg() const;

  /**
   * @brief Wrap heading to pi and store rotation
   *
   * @param[in] heading Input heading
   * @return Orientation&
   */
  Orientation & operator=(double heading);

  /**
   * @brief Assign heading
   *
   * @param[in] q Eigen quaternion object
   * @return Orientation&
   */
  Orientation & operator=(const Eigen::Quaterniond q);

  /**
   * @brief Assign heading
   *
   * @param[in] q ROS `geometry_msgs` quaternion object
   * @return Heading&
   */
  Orientation & operator=(const geometry_msgs::msg::Quaternion q);

  /**
   * @brief Multiply angle by a constant
   *
   * @details The angle is automatically wrapped to (-pi, pi]
   *
   * @param[in] other The heading in rhs
   * @return Heading&
   */
  Orientation operator*(double other) const;

  /**
   * @brief Compoud headings
   *
   * @param[in] other The heading in rhs
   * @return Orientation&
   */
  Orientation operator+(const Orientation & other) const;

  /**
   * @brief Add angle to self
   *
   * @param[in] other Orientation on the RHS
   * @return Orientation&
   */
  Orientation & operator+=(const Orientation & other);

  /**
   * @brief Compoud headings
   *
   * @param[in] other The heading in rhs
   * @return Orientation&
   */
  Orientation operator+(double other) const;

  /**
   * @brief Add angle to self
   *
   * @param[in] other Orientation/angle on the RHS
   * @return Orientation&
   */
  Orientation & operator+=(double other);

  /**
   * @brief Subtract headings
   *
   * @param[in] other The heading in rhs
   * @return Orientation&
   */
  Orientation operator-(const Orientation & other) const;

  /**
   * @brief Subtract headings
   *
   * @param[in] other The heading in rhs
   * @return Orientation&
   */
  Orientation operator-(double other) const;

private:
  Eigen::Rotation2Dd heading_;
};

/**
 * @brief
 *
 * @param[in] os
 * @param[in] heading
 * @return std::ostream&
 */
std::ostream & operator<<(std::ostream & os, const Orientation & heading);
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_ORIENTATION_HPP_
