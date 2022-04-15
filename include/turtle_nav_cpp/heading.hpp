/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file heading.hpp
 * @brief Class for dealing with SO(2) headings
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-21
 */
#ifndef TURTLE_NAV_CPP_HEADING_HPP_
#define TURTLE_NAV_CPP_HEADING_HPP_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <geometry_msgs/msg/quaternion.hpp>
#include <iostream>
#include <turtlesim/msg/pose.hpp>

namespace turtle_nav_cpp
{
namespace nav_utils
{
class Heading
{
public:
  /**
   * @brief Default constructor
   *
   */
  Heading();

  /**
   * @brief Construct a new Heading object
   *
   * @param[in] theta Heading to insert
   *
   * @details The angle is automatically wrapped to (-pi, pi]
   */
  Heading(double theta);

  /**
   * @brief Construct a new Heading object
   *
   * @param[in] rot Eigen rotation object
   */
  Heading(const Eigen::Rotation2Dd & rot);

  /**
   * @brief Construct a new Heading object from a valid rotation matrix
   *
   * @param[in] rot Rotation matrix that belongs to SO(2)
   * @param[in] precision Precision for assessing how close the matrix is to SO(2)
   */
  Heading(const Eigen::Matrix2d & rot, double precision = 1e-10);

  /**
   * @brief Construct a new Heading object from an Eigen quaternion object
   *
   * @param[in] q Eigen quaternion
   */
  Heading(const Eigen::Quaterniond q);

  /**
   * @brief Construct a new Heading object
   *
   * @param[in] q ROS `geometry_msgs` quaternion object
   */
  Heading(const geometry_msgs::msg::Quaternion q);

  // TODO(aalbaali): Construct from `Eigen::Rotation2Dd` and rotation matrix
  /**
   * @brief Return the heading angle wrapped to (-pi, pi]
   *
   * @return double
   */
  double angle() const;

  /**
   * @brief Inverse element
   *
   * @return Heading Inverse heading
   */
  Heading Inverse() const;

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
   * @param[in] theta Input heading
   * @return Heading&
   */
  Heading & operator=(double theta);

  /**
   * @brief Assign heading
   *
   * @param[in] q Eigen quaternion object
   * @return Heading&
   */
  Heading & operator=(const Eigen::Quaterniond q);

  /**
   * @brief Assign heading
   *
   * @param[in] q ROS `geometry_msgs` quaternion object
   * @return Heading&
   */
  Heading & operator=(const geometry_msgs::msg::Quaternion q);

  /**
   * @brief Multiply angle by a constant
   *
   * @details The angle is automatically wrapped to (-pi, pi]
   *
   * @param[in] other The heading in rhs
   * @return Heading&
   */
  Heading operator*(double other) const;

  /**
   * @brief Compoud headings
   *
   * @param[in] other The heading on rhs
   * @return Heading&
   */
  Heading operator+(const Heading & other) const;

  /**
   * @brief Add angle to self
   *
   * @param[in] other Heading on the RHS
   * @return Heading&
   */
  Heading & operator+=(const Heading & other);

  /**
   * @brief Compoud headings
   *
   * @param[in] theta The heading in rhs
   * @return Heading&
   */
  Heading operator+(double theta) const;

  /**
   * @brief Add angle to self
   *
   * @param[in] theta Heading/angle on the RHS
   * @return Heading&
   */
  Heading & operator+=(double theta);

  /**
   * @brief Subtract headings
   *
   * @param[in] theta The heading in rhs
   * @return Heading&
   */
  Heading operator-(const Heading & theta) const;

  /**
   * @brief Subtract headings
   *
   * @param[in] theta The heading in rhs
   * @return Heading&
   */
  Heading operator-(double theta) const;

  /**
   * @brief Compare two headings for angle equality
   *
   * @param[in] other
   * @return bool
   */
  bool operator==(const Heading & other) const;

  bool operator!=(const Heading & other) const;

  /**
   * @brief Cross operator mapping SO(2) Lie algebra coordinates to the Lie algebra
   *
   * @param[in] v Value in the Euclidean space
   * @return Eigen::Matrix2d v^{\cross}
   */
  static Eigen::Matrix2d cross(const double v);

private:
  Eigen::Rotation2Dd rotation_;
};

/**
* @brief Output stream for Heading object
 *
* @param[in/out] os Output stream
 * @param[in] heading Heading to feed into the output stream
 * @return std::ostream& Resulting output stream
 */
std::ostream & operator<<(std::ostream & os, const Heading & heading);
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_HEADING_HPP_
