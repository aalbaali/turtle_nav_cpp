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
#include <array>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <queue>
#include <rclcpp/time.hpp>
#include <turtlesim/msg/pose.hpp>
#include <vector>

#include "turtle_nav_cpp/eigen_utils.hpp"
#include "turtle_nav_cpp/heading.hpp"
#include "turtle_nav_cpp/math_utils.hpp"
#include "turtle_nav_cpp/pose.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::TwistWithCovarianceStamped;

namespace ThreeDof
/**
 * @brief 2D Pose indices in 3D pose variables
 *
 * @details This is mostly used in to extract SE(2) covariances from planar SE(3) covariances
 *
 */
{
enum PoseIdx { x = 0, y = 1, th = 5 };
}  // namespace ThreeDof

namespace TwoDof
{
/**
 * @brief 2D pose variable indices used in covarainces
 *
 */
enum PoseIdx { x = 0, y = 1, th = 2 };
}  // namespace TwoDof

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

//==================================================================================================
// Covariances
//==================================================================================================

/**
 * @brief Convert covariance on SE(3) poses/twists (i.e., 3 degrees-of-freedom) to covariance on
 * SE(2) poses/twists (i.e., 2 degrees-of-freedom). This is basically marginalizing the variables
 * not available in SE(2).
 *
 * @param[in] cov_3dof      Covariance to convert
 * @return Eigen::Matrix3d  Marginalized covariance
 */
Eigen::Matrix3d Cov3dofToCov2dof(const Eigen::Matrix<double, 6, 6> & cov_3dof);

/**
 * @brief Wrapper around `Cov3dofToCov2dof` for geometry_msgs::msg::Covariance on SE(3) poses/twists
 *
 * @details Converts covariance on SE(3) poses/twists, described using a row-major array into
 * covariance on SE(2) described using matrix
 *
 * @param[in] cov_3dof_msg Covariance on SE(2) poses/twists, as an array (message)
 * @return Eigen::Matrix3d Covariance on SE(2) poses/twists, as a matrix
 */
Eigen::Matrix3d Cov3dofMsgToCov2dof(const std::array<double, 36> & cov_3dof_msg);

/**
 * @brief Convert covariance on SE(2) poses/twists (i.e., 2 degrees-of-freedom) to covariance on
 * SE(3) poses/twists (i.e., 3 degrees-of-freedom).
 *
 * @param[in] cov_2dof
 * @return Eigen::Matrix2d
 */
Eigen::Matrix<double, 6, 6> Cov2dofToCov3dof(const Eigen::Matrix3d & cov_2dof);

/**
 * @brief Wrapper around `Cov2dofToCov3dof` for geometry_msgs::msg::Covariance on SE(3) poses/twists
 *
 * @details Converts covariance on SE(2) poses/twists, described using a matrix into covariance on
 * SE(3) described using a row-major array
 *
 * @param[in] cov_2dof Covariance on SE(2) poses/twists, as a matrix
 * @return std::array<double, 36> Covariance on SE(3) poses/twists, as a row-major array
 */
std::array<double, 36> Cov2dofToCov3dofMsg(const Eigen::Matrix3d & cov_2dof);

/**
 * @brief Retract 3D covariance ellipse points (on \xi from se(2))  onto the SE(2) group, and
 * extract the points from the poses (i.e., ignore the angles).
 *
 * @details Covariance is computed on the Lie group using right-perturbation
 * @param[in] pose        Pose to retract covariance points at
 * @param[in] cov         Covariance on \xi (i.e., Log(pose))
 * @param[in] scale       Factor to scale the covariance points
 * @param[in] num_points  Number of points to generate
 * @return const std::vector<Eigen::Vector2d> 2D points representing the uncertainty ellipse
 */
const std::vector<Eigen::Vector2d> RetractSe2CovarianceEllipse(
  const Pose & pose, const Eigen::Matrix3d & cov, const double scale = 1,
  const double num_points = 100);

//==================================================================================================
// Filtering
//==================================================================================================

// TODO(aalbaali): Test this function
/**
 * @brief Accumulate odometry data using SE(2) process model (similar to IMU pre-integration)
 *
 * @param[in] query_at  Time to query (or predict) the pose
 * @param[in] initial_pose Initial pose to start the dead-reckoning (should have a stamp before one
 *                         of the velocity measurements)
 * @param[in] vel_history History of velocities
 * @return PoseWithCovarianceStamped Final pose
 */
PoseWithCovarianceStamped AccumOdom(
  const rclcpp::Time & query_at, const PoseWithCovarianceStamped & initial_pose,
  std::queue<TwistWithCovarianceStamped> & vel_history);

}  // namespace nav_utils
}  // namespace turtle_nav_cpp
#endif  // TURTLE_NAV_CPP_INCLUDE_TURTLE_NAV_CPP_NAV_UTILS_HPP_
