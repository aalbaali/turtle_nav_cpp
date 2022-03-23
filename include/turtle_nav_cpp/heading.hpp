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
   * @param[in] heading Heading to insert
   *
   * @details The heading is automatically wrapped to (-pi, pi]
   */
  Heading(double heading);

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

  /**
   * @brief Return the heading angle wrapped to (-pi, pi]
   *
   * @return double
   */
  double angle() const;

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
   * @return Heading&
   */
  Heading & operator=(double heading);

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
   * @param[in] other The heading in rhs
   * @return Heading&
   */
  Heading operator+(double other) const;

  /**
   * @brief Add angle to self
   *
   * @param[in] other Heading/angle on the RHS
   * @return Heading&
   */
  Heading & operator+=(double other);

  /**
   * @brief Subtract headings
   *
   * @param[in] other The heading in rhs
   * @return Heading&
   */
  Heading operator-(const Heading & other) const;

  /**
   * @brief Subtract headings
   *
   * @param[in] other The heading in rhs
   * @return Heading&
   */
  Heading operator-(double other) const;

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
