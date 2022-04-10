/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file pose.hpp
 * @brief SE(2) pose class
 *
 * @details The Special Euclidean group, SE(2), is the set of matrices, known as transformation
 * matrices that have the form
 *  T_a_b = [ C_a_b   r_zw_a
 *            0   0      1   ],
 * where C_a_b is the orientation from frame `a` to frame `b`, `r_zw_a` is the translation of point
 * `z` with respect to point `w`, resolved in the frame `a`.

 * When it comes to poses and orientation, reference frames are important to distinguish. In order
 * to minimize ambiguities, the notation the following notation is suggested to be used:
 *
 * r_z_w_a    : Position of point `z` with respect to point `w`, resolved in the frame `a`;
 * C_a_b      : Orientation/rotation from frame `a` to frame `b`;
 * T_a_b      : Pose from reference frame `a` to `b`.
 *
 * Alternatively, the letters `r`, `C`, and `T` can be replaced with `pos`, `heading`, and `pose`,
 * respectively.
 *
 *
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-21
 */
#ifndef TURTLE_NAV_CPP_POSE_HPP_
#define TURTLE_NAV_CPP_POSE_HPP_

#include <Eigen/Dense>
#include <iostream>

#include "geometry_msgs/msg/pose.hpp"
#include "turtle_nav_cpp/heading.hpp"
#include "turtlesim/msg/pose.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
using Eigen::Affine2d;
using Eigen::Vector2d;

class Pose
{
public:
  //================================================================================================
  // Constructors
  //================================================================================================
  /**
   * @brief Default construct a new Pose object
   *
   */
  Pose();

  /**
   * @brief Construct a new Pose object
   *
   * @param[in] affine Eigen affine transformation
   */
  Pose(const Affine2d & affine);

  /**
   * @brief Construct a new Pose object
   *
   * @param[in] pos Translation resolved in the arbitrary frame `a`
   * @param[in] heading Heading of frame `a` to frame `b`
   */
  Pose(const Vector2d & pos, const Heading & heading);

  /**
   * @brief Construct a new Pose object
   *
   * @param[in] pos Translation resolved in the arbitrary frame `a`
   * @param[in] heading angle from frame `a` to frame `b`
   */
  Pose(const Vector2d & pos, double heading);

  /**
   * @brief Construct a new Pose object from individual components
   *
   * @param[in] x x-component of the translation resolved in the arbitrary frame `a`
   * @param[in] y y-component of the translation resolved in the arbitrary frame `a`
   * @param[in] heading angle from frame `a` to frame `b`
   */
  Pose(double x, double y, double heading);

  /**
   * @brief Construct a new Pose object
   *
   * @param[in] pose_msg ROS geometry pose object
   */
  Pose(const geometry_msgs::msg::Pose & pose_msg);

  /**
   * @brief Construct a new Pose object
   *
   * @param[in] pose_in Turtlesim-type pose
   */
  Pose(const turtlesim::msg::Pose & pose_in);

  //================================================================================================
  // Getters
  //================================================================================================

  /**
   * @brief Get ROS geometry type pose
   *
   * @return geometry_msgs::msg::Pose
   */
  geometry_msgs::msg::Pose PoseMsg() const;

  /**
   * @brief Get turtlesim type pose
   *
   * @return turtlesim::msg::Pose
   */
  turtlesim::msg::Pose TurtlePose() const;

  /**
   * @brief Get Eigen Affine2d object
   *
   * @return Affine2d
   */
  Affine2d Affine() const;

  /**
   * @brief Get translational component
   *
   * @return Vector2d Translational component resolved in the `a` frame (i.e., the "from" frame)
   */
  Vector2d translation() const;

  /**
   * @brief Get heading component
   *
   * @return Heading
   */
  Heading heading() const;

  /**
   * @brief Get x component of the translation
   *
   * @return double
   */
  double x() const;

  /**
   * @brief Get the y component of the translation
   *
   * @return double
   */
  double y() const;

  /**
   * @brief Get the wrapped angle
   *
   * @return double angle wrapped to (-pi, pi]
   */
  double angle() const;

  //================================================================================================
  // Operators
  //================================================================================================

  /**
   * @brief Assign pose from affine object
   *
   * @param[in] affine Eigen affine object
   * @return Pose& Assigned pose
   */
  Pose & operator=(const Eigen::Affine2d & affine);

  /**
   * @brief Assign pose
   *
   * @param[in] pose_in ROS geometry pose type
   * @return Pose& Assigned pose
   */
  Pose & operator=(const geometry_msgs::msg::Pose & pose_in);

  /**
   * @brief Assign pose
   *
   * @param[in] pose_in Turtlesim pose type
   * @return Pose& Assigned pose
   */
  Pose & operator=(const turtlesim::msg::Pose & pose_in);

  /**
   * @brief Pose compounding
   *
   * @param[in] other Pose on the rhs
   * @return Pose Compounded pose
   */
  Pose operator*(const Pose & other) const;

  /**
   * @brief Transforming a vector into frame `a`
   *
   * @param[in] v Input vector
   * @return Vector2d Transformed vector
   */
  Vector2d operator*(const Vector2d & v) const;

  /**
   * @brief Self compounding pose
   *
   * @param[in] other Pose on rhs (i.e., T_2 = T_1 * other)
   * @return Pose&
   */
  Pose & operator*=(const Pose & other);

  /**
   * @brief Compare two poses for equality on each of the scalar elements (i.e., x, y, and theta)
   *
   * @param[in] pose_rhs Pose on rhs
   * @return bool
   */
  bool operator==(const Pose & pose_rhs) const;

  /**
   * @brief The negation of `operator==`
   *
   * @param[in] pose_rhs Pose on rhs
   * @return bool True if all scalars are equal
   */
  bool operator!=(const Pose & pose_rhs) const;

  //================================================================================================
  // Lie group operations
  //================================================================================================

  /**
   * @brief Get the pose inverse
   *
   * @return Pose
   */
  Pose Inverse() const;

  /**
   * @brief Return adjoint matrix
   *
   * @return Eigen::Matrix3d Adjoint matrix
   */
  Eigen::Matrix3d Adjoint() const;

private:
  // Translational component of the pose. Named `position` for brevity
  Vector2d position_;

  // Heading component of the pose
  Heading heading_;
};

/**
 * @brief Output stream for Pose object
 *
 * @param[in/out] os Output stream
 * @param[in] pose Pose to feed into the output stream
 * @return std::ostream& Resulting output stream
 */
std::ostream & operator<<(std::ostream & os, const Pose & pose);

}  // namespace nav_utils
}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_POSE_HPP_
