/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file nav_utils.cpp
 * @brief Implementation of nav_utils.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-22
 */

#include "turtle_nav_cpp/nav_utils.hpp"

#include <algorithm>
#include <limits>
#include <queue>
#include <string>
#include <vector>

#include "turtle_nav_cpp/eigen_utils.hpp"
#include "turtle_nav_cpp/math_utils.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
//==================================================================================================
// Heading
//==================================================================================================

Eigen::Quaterniond QuaternionMsgToQuaternion(const geometry_msgs::msg::Quaternion & q_msg)
{
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  q.normalize();
  return q;
}

geometry_msgs::msg::Quaternion QuaternionToQuaternionMsg(const Eigen::Quaterniond & q)
{
  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = q.x();
  q_msg.y = q.y();
  q_msg.z = q.z();
  q_msg.w = q.w();
  return q_msg;
}

double QuaternionMsgToHeading(const geometry_msgs::msg::Quaternion & q)
{
  return WrapToPi(QuaternionToHeading(QuaternionMsgToQuaternion(q)));
}

geometry_msgs::msg::Quaternion HeadingToQuaternionMsg(double heading)
{
  return QuaternionToQuaternionMsg(HeadingToQuaternion(heading));
}

//==================================================================================================
// Poses
//==================================================================================================

turtlesim::msg::Pose PoseMsgToTurtlePose(const geometry_msgs::msg::Pose & pose)
{
  turtlesim::msg::Pose pose_turtle;
  pose_turtle.x = pose.position.x;
  pose_turtle.y = pose.position.y;
  pose_turtle.theta = Heading(pose.orientation).angle();

  return pose_turtle;
}

geometry_msgs::msg::Pose TurtlePoseToPoseMsg(const turtlesim::msg::Pose & pose)
{
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = pose.x;
  pose_msg.position.y = pose.y;
  pose_msg.orientation = Heading(pose.theta).QuaternionMsg();
  return pose_msg;
}

//==================================================================================================
// Covariances
//==================================================================================================

Eigen::Matrix3d Cov3dofToCov2dof(const Eigen::Matrix<double, 6, 6> & cov_3dof)
{
  // Get the relevant covariances
  Eigen::Matrix3d cov_2dof;
  cov_2dof.block<2, 2>(0, 0) = cov_3dof.block<2, 2>(0, 0);
  cov_2dof.block<2, 2>(TwoDof::PoseIdx::x, TwoDof::PoseIdx::x) =
    cov_3dof.block<2, 2>(ThreeDof::PoseIdx::x, ThreeDof::PoseIdx::x);
  cov_2dof(TwoDof::PoseIdx::th, TwoDof::PoseIdx::th) =
    cov_3dof(ThreeDof::PoseIdx::th, ThreeDof::PoseIdx::th);
  cov_2dof.block<2, 1>(TwoDof::PoseIdx::x, TwoDof::PoseIdx::th) =
    cov_3dof.block<2, 1>(ThreeDof::PoseIdx::x, ThreeDof::PoseIdx::th);
  cov_2dof.block<1, 2>(TwoDof::PoseIdx::th, TwoDof::PoseIdx::x) =
    cov_3dof.block<1, 2>(ThreeDof::PoseIdx::th, ThreeDof::PoseIdx::x);

  // Ensure symmetry
  cov_2dof = 0.5 * (cov_2dof.eval() + cov_2dof.transpose().eval());

  return cov_2dof;
}

Eigen::Matrix3d Cov3dofMsgToCov2dof(const std::array<double, 36> & cov_3dof_msg)
{
  return Cov3dofToCov2dof(eigen_utils::StdArrayToMatrix<6, 6, Eigen::RowMajor>(cov_3dof_msg));
}

Eigen::Matrix<double, 6, 6> Cov2dofToCov3dof(const Eigen::Matrix3d & cov_2dof)
{
  // Get the relevant covariances
  Eigen::Matrix<double, 6, 6> cov_3dof;
  cov_3dof.setZero();
  cov_3dof.block<2, 2>(0, 0) = cov_2dof.block<2, 2>(0, 0);
  cov_3dof.block<2, 2>(ThreeDof::PoseIdx::x, ThreeDof::PoseIdx::x) =
    cov_2dof.block<2, 2>(TwoDof::PoseIdx::x, TwoDof::PoseIdx::x);
  cov_3dof(ThreeDof::PoseIdx::th, ThreeDof::PoseIdx::th) =
    cov_2dof(TwoDof::PoseIdx::th, TwoDof::PoseIdx::th);
  cov_3dof.block<2, 1>(ThreeDof::PoseIdx::x, ThreeDof::PoseIdx::th) =
    cov_2dof.block<2, 1>(TwoDof::PoseIdx::x, TwoDof::PoseIdx::th);
  cov_3dof.block<1, 2>(ThreeDof::PoseIdx::th, ThreeDof::PoseIdx::x) =
    cov_2dof.block<1, 2>(TwoDof::PoseIdx::th, TwoDof::PoseIdx::x);

  // Ensure symmetry
  cov_3dof = 0.5 * (cov_3dof.eval() + cov_3dof.transpose().eval());

  return cov_3dof;
}

std::array<double, 36> Cov2dofToCov3dofMsg(const Eigen::Matrix3d & cov_2dof)
{
  return eigen_utils::MatrixToStdArray<6, 6, Eigen::RowMajor>(Cov2dofToCov3dof(cov_2dof));
}
//==================================================================================================
// Filtering
//==================================================================================================

PoseWithCovarianceStamped AccumOdom(
  const rclcpp::Time & query_at, const PoseWithCovarianceStamped & initial_pose,
  std::queue<TwistWithCovarianceStamped> & vel_history)
{
  // TODO(aalbaali): Add a velocity threshold under which the velocity variance is ignored.
  // For now, it's assumed to be zero

  rclcpp::Time initial_pose_time = initial_pose.header.stamp;

  // Ensure query time is after initial pose
  if (query_at < initial_pose_time) {
    std::stringstream ss;
    ss << "query time \033[93;1m" << query_at.seconds()
       << "\033[0m is earlier than initial_pose time \033[93;1m" << initial_pose_time.seconds();
    throw std::invalid_argument(ss.str());
  }

  // Need non-empty velocity history to dead-reckon
  if (vel_history.size() == 0) {
    PoseWithCovarianceStamped latest_pose = initial_pose;
    latest_pose.header.stamp = query_at;
    return latest_pose;
  }

  // Latest measurement (start from the earliest measurement)
  TwistWithCovarianceStamped earliest_vel = vel_history.back();

  // Flag to indicate that the pose is successfully predicted to the requested query time
  // Note that this may happen before the velocity history queue is emptied
  bool is_predicted_to_query_time = false;

  // Latest pose, where it's assumed that the latest pose remains unchanged until the latest
  // measurement comes in
  PoseWithCovarianceStamped latest_pose = initial_pose;

  // Deal with the case that query time is between initial_pose and the earliest velocity
  // measurement, in which case the pose remains unchanged
  if (query_at < earliest_vel.header.stamp) {
    latest_pose.header.stamp = query_at;
    return latest_pose;
  }

  // Predict poses and covariances until the last velocity time stamp
  // Each pose is predicted to the next time stamp. That is, pose at T(t_km1) is predicted until the
  // time of the nearest velocity measurement that is after t_km1
  while (!vel_history.empty() && !is_predicted_to_query_time) {
    earliest_vel = vel_history.back();
    vel_history.pop();

    // Query time to predict to
    rclcpp::Time time_to_predict_to = earliest_vel.header.stamp;

    // Ignore measurements earlier than latest pose
    if (time_to_predict_to < latest_pose.header.stamp) {
      continue;
    }

    // Use query time if it's earlier than the velocity time
    if (query_at <= time_to_predict_to) {
      time_to_predict_to = query_at;
      is_predicted_to_query_time = true;
    }

    // Time difference from the latest pose to the time to predict to (basically dt)
    auto duration_latest_pose_to_query_time = time_to_predict_to - latest_pose.header.stamp;
    double dt = duration_latest_pose_to_query_time.seconds();

    // Dead-reckon poses using SE(2) model
    Pose T_km1 = latest_pose.pose.pose;
    Vector2d linear_vel_km1 =
      Vector2d(earliest_vel.twist.twist.linear.x, earliest_vel.twist.twist.linear.y);
    double angular_vel_km1 = earliest_vel.twist.twist.angular.z;
    Pose dT_km1(dt * linear_vel_km1, dt * angular_vel_km1);
    Pose T_k = T_km1 * dT_km1;
    latest_pose.pose.pose = T_k.PoseMsg();
    latest_pose.header.stamp = time_to_predict_to;

    //==============================================================================================
    // Covariance propagation
    //==============================================================================================
    // Covariance on latest pose
    Eigen::Matrix3d cov_T_km1 = Cov3dofMsgToCov2dof(latest_pose.pose.covariance);
    Eigen::Matrix3d cov_v_km1 = Cov3dofMsgToCov2dof(earliest_vel.twist.covariance);

    // Check positive definiteness
    if (Eigen::LLT<Eigen::Matrix3d>(cov_T_km1).info() == Eigen::NumericalIssue) {
      std::stringstream ss;
      ss << "State covariance is not symmetric positive (semi) definite" << cov_T_km1;
      throw ss.str();
    }

    // If covariance on the y-component of the velocity is negative (which is used to imply that
    // there's NO variance on the variable), then replace it with process noise
    if (cov_v_km1(TwoDof::y, TwoDof::y) < 0) {
      cov_v_km1(1, 1) = 1e-10;
    }

    if (Eigen::LLT<Eigen::Matrix3d>(cov_v_km1).info() == Eigen::NumericalIssue) {
      std::stringstream ss;
      ss << "Measurement covariance is not symmetric positive definite: " << cov_v_km1;
      throw ss.str();
    }

    // Jacobian of the process model w.r.t. the state (T_km1 = Exp(xi_km1))
    // Adj_{Exp(-u_j)}. Check (100) and (161) from Sola.
    Eigen::Matrix3d jac_xi_km1 = dT_km1.Inverse().Adjoint();

    // Jacobian of the process model w.r.t. the control input
    // J_r_{u_j} from (163) in Sola.
    // For now, assume it's an identity matrix, which is not far off.
    Eigen::Matrix3d jac_v_km1 = dt * Eigen::Matrix3d::Identity();

    // Compute the covariance
    auto cov_xi_k_xi_km1 = jac_xi_km1 * cov_T_km1 * jac_xi_km1.transpose();
    auto cov_xi_k_v_km1 = jac_v_km1 * cov_v_km1 * jac_v_km1.transpose();
    auto cov_xi_k = cov_xi_k_xi_km1 + cov_xi_k_v_km1;

    // Convert the covariance matrix on SE(2) to covariance matrix on SE(3)
    Eigen::Matrix<double, 6, 6> cov_msg;
    cov_msg.setZero();
    cov_msg.block<2, 2>(0, 0) = cov_xi_k.block<2, 2>(0, 0);
    cov_msg(5, 5) = cov_xi_k(2, 2);
    cov_msg.block<2, 1>(0, 5) = cov_xi_k.block<2, 1>(0, 2);
    cov_msg.block<1, 2>(5, 0) = cov_xi_k.block<1, 2>(2, 0);

    // Assign the covariance to the pose message
    latest_pose.pose.covariance = Cov2dofToCov3dofMsg(cov_xi_k);
  }

  // Now predict pose from latest pose to query time
  if (!is_predicted_to_query_time) {
    auto duration_latest_pose_to_query_time = query_at - latest_pose.header.stamp;
    double dt = duration_latest_pose_to_query_time.seconds();
    Pose T_km1 = latest_pose.pose.pose;
    Vector2d linear_vel_km1{earliest_vel.twist.twist.linear.x, earliest_vel.twist.twist.linear.y};
    double angular_vel_km1 = earliest_vel.twist.twist.angular.z;
    Pose T_k = T_km1 * Pose(dt * linear_vel_km1, dt * angular_vel_km1);
    latest_pose.pose.pose = T_k.PoseMsg();
    latest_pose.header.stamp = query_at;
  }

  return latest_pose;
}

const std::vector<Eigen::Vector2d> RetractSe2CovarianceEllipse(
  const Pose & pose, const Eigen::Matrix3d & cov, const double scale /* = 1 */,
  const double num_points /* = 100 */)
{
  if (scale <= 0) {
    throw std::invalid_argument("Scale must be a positive number");
  }

  if (num_points < 2) {
    throw std::invalid_argument("Number of points must be greater than 1");
  }

  // Get poitns of a circle
  const std::vector<Eigen::Vector2d> unit_circle_pts =
    eigen_utils::GetEllipsePoints(Eigen::Matrix2d::Identity(), 1, num_points);

  // Cholesky factor and lower matrix
  const auto cov_llt = cov.llt();
  if (cov_llt.info() == Eigen::NumericalIssue) {
    throw std::invalid_argument("Provided matrix is not positive definite");
  }
  const Eigen::Matrix3d cov_L = cov_llt.matrixL();

  // Retracted points
  std::vector<Eigen::Vector2d> retracted_pts(num_points);
  std::transform(
    unit_circle_pts.begin(), unit_circle_pts.end(), retracted_pts.begin(),
    [&scale, &cov_L, &pose](const Eigen::Vector2d & pt_2d) -> Eigen::Vector2d {
      const Eigen::Vector3d ell_se2_vec = scale * cov_L * Eigen::Vector3d{pt_2d(0), pt_2d(1), 0};
      Pose T_pt_global = pose * Pose::Exp(ell_se2_vec.block<2, 1>(0, 0), ell_se2_vec(2));
      return Eigen::Vector2d{T_pt_global.x(), T_pt_global.y()};
    });

  return retracted_pts;
}

}  // namespace nav_utils
}  // namespace turtle_nav_cpp
