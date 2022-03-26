/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file nav_utils.hpp
 * @brief 2D navigation tools
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-21
 */
#ifndef TURTLE_NAV_CPP_NAV_UTILS_HPP_
#define TURTLE_NAV_CPP_NAV_UTILS_HPP_

#include <Eigen/Dense>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <turtlesim/msg/pose.hpp>

#include "turtle_nav_cpp/heading.hpp"
#include "turtle_nav_cpp/math_utils.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
/**
 * @brief 2D Pose indices in 3D pose
 *
 */
enum TwistIdx { x = 0, y = 1, th = 5 };

/**
 * @brief Convert heading to Eigen quaternion
 *
 * @tparam T Scalar type (e.g., `double` or `float`)
 * @param[in] heading
 * @return Eigen::Quaternion<T>
 */
template <typename T>
Eigen::Quaternion<T> HeadingToQuaternion(T heading)
{
  return Eigen::Quaternion<T>(Eigen::AngleAxis<T>(heading, Eigen::Matrix<T, 3, 1>::UnitZ()));
}

//==================================================================================================
// Heading
//==================================================================================================
/**
 * @brief Convert an Eigen quaternion to a heading
 *
 * @tparam T Scalar type (e.g., `double` or `float`)
 * @param[in] q Eigen quaternion
 * @return T
 */
template <typename T>
T QuaternionToHeading(const Eigen::Quaternion<T> & q)
{
  // For planar rotation, the quaternion should have the form: (0, 0, cos(th/2)) + sin(th/2)
  return WrapToPi(atan2(q.z(), q.w()) * 2);
}

/**
 * @brief Convert ROS geometry quaternion to Eigen quaternion
 *
 * @param[in] q_msg ROS geometry msg quaternion
 * @return Eigen::Quaterniond
 */
Eigen::Quaterniond QuaternionMsgToQuaternion(const geometry_msgs::msg::Quaternion & q_msg);

/**
 * @brief Convert Eigen quaternion to ROS geometry quaternion
 *
 * @param[in] q Eigen quaternion
 * @return geometry_msgs::msg::Quaternion
 */
geometry_msgs::msg::Quaternion QuaternionToQuaternionMsg(const Eigen::Quaterniond & q);

/**
 * @brief Convert a ROS geometry msg to heading
 *
 * @param[in] q ROS geometry msg
 * @return double
 */
double QuaternionMsgToHeading(const geometry_msgs::msg::Quaternion & q);

/**
 * @brief Convert heading to ROS geometry quaternion
 *
 * @param[in] heading Planar heading to convert
 * @return geometry_msgs::msg::Quaternion
 */
geometry_msgs::msg::Quaternion HeadingToQuaternionMsg(double heading);

/**
 * @brief Extract angle from rotation matrix
 *
 * @param[in] rot SO(2) matrix
 * @return double Angle
 */
double RotationMatrixToAngle(const Eigen::Matrix2d & rot);

//==================================================================================================
// Poses
//==================================================================================================

/**
 * @brief Convert ROS geometry msg pose to turtlesim pose
 *
 * @param[in] pose ROS geometry msg pose
 * @return turtlesim::msg::Pose Turtlesim pose
 */
turtlesim::msg::Pose PoseMsgToTurtlePose(const geometry_msgs::msg::Pose & pose);

/**
 * @brief Convert turtlesim pose to geometry msg pose
 *
 * @param[in] pose Turtlesim pose
 * @return geometry_msgs::msg::Pose ROS geometry msg pose
 */
geometry_msgs::msg::Pose TurtlePoseToPoseMsg(const turtlesim::msg::Pose & pose);

}  // namespace nav_utils
}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_INCLUDE_TURTLE_NAV_CPP_NAV_UTILS_HPP_
