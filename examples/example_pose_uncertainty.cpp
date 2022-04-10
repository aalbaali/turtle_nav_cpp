/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file example_pose_uncertainty.cpp
 * @brief Example of propagating uncertainties through SE(2) poses
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Apr-01
 */

#include <matplot/matplot.h>

#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <random>
#include <string>
#include <tuple>
#include <unsupported/Eigen/MatrixFunctions>
#include <utility>
#include <vector>

#include "turtle_nav_cpp/math_utils.hpp"
#include "turtle_nav_cpp/pose.hpp"

using turtle_nav_cpp::nav_utils::Pose;
using Trajectory = std::vector<Pose>;
using StdVector2Double = std::vector<std::vector<double>>;

namespace turtle_nav_cpp
{
/**
 * @brief Parameters of a Gaussian random variable
 *
 */
struct GaussianRV
{
  // Mean of the RV
  double mean;

  // Standard deviation of the RV
  double stddev;
};

/**
 * @brief Cross operator mapping SO(2) Lie algebra coordinates to the Lie algebra
 *
 * @param[in] v Value in the Euclidean space
 * @return Eigen::Matrix2d v^{\cross}
 */
Eigen::Matrix2d cross(const double v)
{
  Eigen::Matrix2d mat;

  // clang-format off
  mat << 0, -v,
       v, 0;
  // clang-format on

  return mat;
}

/**
 * @brief SE(2) exponential map
 *
 * @param[in] lie_alg_translation Translational component of the *Lie algebra* vector
 * @param[in] theta               Heading
 * @return Pose Exp([rho; theta])
 */
Pose Exp(const Eigen::Vector2d & lie_alg_translation, const double theta)
{
  // From (156) and (158) of Sola
  const Eigen::Matrix2d V =
    sin(theta) / theta * Eigen::Matrix2d::Identity() + (1 - cos(theta)) / theta * cross(1);

  return Pose(V * lie_alg_translation, theta);
}

/**
 * @brief Plot x-y positions of a vector of pose
 *
 * @param[in] poses Vector of poses to plot
 */
void PlotTrajectory(const Trajectory & poses)
{
  // Get x and y values
  std::vector<double> xvals;
  std::vector<double> yvals;

  std::for_each(poses.begin(), poses.end(), [&](const nav_utils::Pose & pose) {
    xvals.push_back(pose.translation().x());
    yvals.push_back(pose.translation().y());
  });

  matplot::plot(xvals, yvals)->line_width(1.5);
  matplot::grid(matplot::on);
}

/**
 * @brief Plot full trajectories
 *
 * @param[in] trajectories
 */
void PlotTrajectories(const std::vector<Trajectory> & trajectories)
{
  matplot::figure();
  matplot::hold(true);
  for (const auto & traj : trajectories) {
    turtle_nav_cpp::PlotTrajectory(traj);
  }
  matplot::xlabel("x [m]");
  matplot::ylabel("y [m]");
  matplot::title("Robot trajectory");
  matplot::grid(true);
  matplot::show();
}

template <typename... Args>
void PlotTrajectoryEndPoints(const std::vector<Trajectory> & trajectories, Args... args)
{
  // Get the x-y points of the end point of each trajectory
  std::vector<double> xvals;
  std::vector<double> yvals;
  xvals.reserve(trajectories.size());
  yvals.reserve(trajectories.size());
  for (const auto & traj : trajectories) {
    const auto last_pose = traj.back();
    xvals.push_back(last_pose.x());
    yvals.push_back(last_pose.y());
  }

  matplot::hold(true);
  auto l = matplot::scatter(xvals, yvals, args...);
  // l->marker_style((matplot::line_spec::marker_style::cross));
  l->marker_face_color({0, 0.5, 0.5});
  matplot::xlabel("x [m]");
  matplot::ylabel("y [m]");
  matplot::title("Trajectory end points");
}

/**
 * @brief Plot a 2D ellipse from a symmetric positive definite matrix
 *
 * @param[in] mat         Square symmetric positive definite matrix
 * @param[in] scale       Number to scale the ellipse
 * @param[in] num_points  Number of points to plot on the ellipse
 * @return std::vector<Eigen::Vector2d> Points of 2D ellipse
 */
std::vector<Eigen::Vector2d> GetEllipsePoints(
  const Eigen::Matrix2d & mat, const double scale = 1, const int num_points = 100)
{
  // Ensure symmetry
  if (!mat.isApprox(mat.transpose())) {
    throw std::runtime_error("Non symmetric matrix");
  }

  // Get Cholesky factorization
  const Eigen::LLT<Eigen::Matrix2d> mat_llt = mat.llt();

  // Check for covariance positive semi-definiteness
  if (mat_llt.info() == Eigen::NumericalIssue) {
    throw std::runtime_error("Covariance matrix possibly non semi-positive definite matrix");
  }

  // Get lower Cholesky factor
  const Eigen::Matrix2d mat_chol_lower = mat_llt.matrixL();

  // Compute points
  std::vector<Eigen::Vector2d> ellipse_points(num_points);
  const std::vector<double> thetas = matplot::linspace(-M_PI, M_PI, num_points);
  std::transform(thetas.begin(), thetas.end(), ellipse_points.begin(), [&](const double th) {
    const Eigen::Vector2d v{cos(th), sin(th)};
    Eigen::Vector2d p = scale * mat_chol_lower * v;
    return p;
  });

  return ellipse_points;
}

/**
 * @brief Get the Ellipse (not ellipsoid) points object in 3D space
 *
 * @param[in] mat 3x3 matrix
 * @param[in] scale
 * @param[in] num_points
 * @return std::vector<Eigen::Vector3d>
 */
std::vector<Eigen::Vector3d> GetEllipsePoints(
  const Eigen::Matrix3d & mat, const double scale = 1, const int num_points = 100)
{
  // Ensure symmetry
  if (!mat.isApprox(mat.transpose())) {
    throw std::runtime_error("Non symmetric matrix");
  }

  // Check for covariance positive semi-definiteness
  if (Eigen::LLT<Eigen::Matrix3d>(mat).info() == Eigen::NumericalIssue) {
    throw std::runtime_error("Covariance matrix possibly non semi-positive definite matrix");
  }

  // Get eigenvalues and eigenvectors
  const Eigen::EigenSolver<Eigen::Matrix3d> eigs(mat);
  const Eigen::Matrix3d eig_vectors = eigs.eigenvectors().real();
  const Eigen::Matrix3d eig_sqrt_vals = eigs.eigenvalues().real().unaryExpr(&sqrt).asDiagonal();

  // Compute points
  std::vector<Eigen::Vector3d> ellipse_points(num_points);
  const std::vector<double> thetas = matplot::linspace(0, 2 * M_PI, num_points);
  std::transform(thetas.begin(), thetas.end(), ellipse_points.begin(), [&](const double th) {
    const Eigen::Vector3d v{cos(th), sin(th), 0};
    return Eigen::Vector3d(scale * eig_vectors * eig_sqrt_vals * v);
  });

  return ellipse_points;
}

/**
 * @brief Get x and y vectors from a vector of Eigen::Vector2d
 *
 * @param[in] points Vector of Eigen::Vector2d
 * @return auto Pair of x and y vectors
 */
std::pair<std::vector<double>, std::vector<double>> GetXYPoints(
  const std::vector<Eigen::Vector2d> & points)
{
  // Get x-y points
  std::vector<double> pts_x(points.size());
  std::vector<double> pts_y(points.size());
  std::transform(
    points.begin(), points.end(), pts_x.begin(), [](const Eigen::Vector2d & v) { return v(0); });
  std::transform(
    points.begin(), points.end(), pts_y.begin(), [](const Eigen::Vector2d & v) { return v(1); });

  return {pts_x, pts_y};
}

/**
 * @brief Generate a dead-reckoning trejectory from speed and yaw measurements
 *
 * @param[in] T_0       Initial pose
 * @param[in] dt        Sampling period [s]
 * @param[in] speeds    Vector of speeds [m/s]
 * @param[in] yaw_rates Vector of yaw rates [rad/s]
 * @return Trajectory Vector of poses (i.e., the trajectory)
 */
std::pair<Trajectory, std::vector<Eigen::Matrix3d>> DeadReckonTrajectory(
  const nav_utils::Pose & T_0, const double dt, const std::vector<double> & speeds,
  const std::vector<double> & yaw_rates, const Eigen::Matrix3d & cov_T_0,
  const GaussianRV & speed_rv, const GaussianRV & yaw_rate_rv)
{
  const size_t num_poses = speeds.size() + 1;
  Trajectory poses;
  poses.reserve(num_poses);
  poses.push_back(T_0);

  // Process model Jacobian w.r.t. error (161 in Sola)
  auto jac_A = [](const Pose & /* T_km1 */, const Pose & dT_km1) {
    const Pose dT_km1_inv = dT_km1.Inverse();
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    A.block<2, 2>(0, 0) = dT_km1_inv.heading().RotationMatrix();
    A(0, 2) = dT_km1_inv.y();
    A(1, 2) = -dT_km1_inv.x();
    A(2, 2) = 1;

    return A;
  };

  // Process model Jacobian w.r.t. measurement
  auto jac_L = [](const double dt, const Pose & /* T_km1 */, const Pose & /* dT_km1 */) {
    return -dt * Eigen::Matrix3d::Identity();
  };

  // Covariances on the left-invariant errors
  std::vector<Eigen::Matrix3d> covs;
  covs.reserve(num_poses);
  covs.push_back(cov_T_0);

  // Measurement covariances
  //  Let the measurements be [x; y; theta]
  Eigen::Matrix3d cov_meas = Eigen::Matrix3d::Zero();
  cov_meas(0, 0) = speed_rv.stddev * speed_rv.stddev;
  cov_meas(1, 1) = 1e-6;
  cov_meas(2, 2) = yaw_rate_rv.stddev * yaw_rate_rv.stddev;

  for (size_t i = 1; i < speeds.size(); i++) {
    nav_utils::Pose dT_km1(dt * speeds[i - 1], 0, dt * yaw_rates[i - 1]);
    const auto T_k = poses[i - 1] * dT_km1;
    poses.push_back(T_k);

    // Compute left-invariant covariance
    const Eigen::Matrix3d A = jac_A(poses[i - 1], dT_km1);
    const Eigen::Matrix3d L = jac_L(dt, poses[i - 1], dT_km1);
    const Eigen::Matrix3d cov_pose_k =
      A * covs.back() * A.transpose() + L * cov_meas * L.transpose();
    covs.push_back(cov_pose_k);
  }

  // Generate trajectory
  return {poses, covs};
}

/**
 * @brief Generate a vector<double> of a samples of a Gaussian RV for a given mean and standard
 *        deviation
 *
 * @param[in] num_vals          Number of values/samples to generate
 * @param[in] rv_params         Gaussian random variable parameters (i.e., mean and std deviation)
 * @param[in] rn_generator      Random number generator
 * @return std::vector<double>  Vector of realizations
 */
std::vector<double> GenerateNoisyVector(
  const size_t num_vals, const GaussianRV & rv_params, std::default_random_engine & rn_generator)
{
  auto randn = [&rn_generator]() { return turtle_nav_cpp::randn_gen(rn_generator); };

  std::vector<double> noisy_vals(num_vals);
  std::generate(noisy_vals.begin(), noisy_vals.end(), [&]() {
    return rv_params.mean + rv_params.stddev * randn();
  });

  return noisy_vals;
}

std::tuple<std::vector<Trajectory>, std::vector<std::vector<Eigen::Matrix3d>>> GenerateTrajectories(
  const size_t num_trajectories, const size_t num_poses, const Pose & T_0, const double dt,
  const Eigen::Matrix3d & cov_T_0, const GaussianRV & speed_rv, const GaussianRV & yaw_rate_rv,
  std::default_random_engine & rn_generator)
{
  std::vector<Trajectory> trajectories;
  std::vector<std::vector<Eigen::Matrix3d>> pose_covariances;
  std::vector<std::vector<double>> speeds_bundle;
  std::vector<std::vector<double>> yaw_rates_bundle;

  for (size_t i = 0; i < num_trajectories; i++) {
    // Sample odometry measurements
    // Noisy speeds
    speeds_bundle.push_back(
      turtle_nav_cpp::GenerateNoisyVector(num_poses - 1, speed_rv, rn_generator));
    yaw_rates_bundle.push_back(
      turtle_nav_cpp::GenerateNoisyVector(num_poses - 1, yaw_rate_rv, rn_generator));

    // Noisy initial pose
    auto randn = [&rn_generator]() { return turtle_nav_cpp::randn_gen(rn_generator); };
    const Eigen::Matrix3d cov_T_0_cholL = cov_T_0.llt().matrixL();
    const Eigen::Vector3d xi_T_0 = cov_T_0_cholL * Eigen::Vector3d::NullaryExpr(3, randn);
    const Pose T_0_sample = T_0 * Exp(xi_T_0.block<2, 1>(0, 0), xi_T_0(2));

    // Generate straight-line trajectory
    const auto [traj, covs] = DeadReckonTrajectory(
      T_0_sample, dt, speeds_bundle.back(), yaw_rates_bundle.back(), cov_T_0, speed_rv,
      yaw_rate_rv);
    trajectories.push_back(traj);
    pose_covariances.push_back(covs);
  }

  return {trajectories, pose_covariances};
}
}  // namespace turtle_nav_cpp

int main()
{
  // Number of trajectgories to generate
  const size_t num_trajs = 1000;

  // Numbers of poses per trajectory
  const int num_poses = 300;

  // Dead-reckoning parameters
  Pose T_0(0, 0, 0);
  const Eigen::Matrix3d cov_T_0 = Eigen::Vector3d{1e-5, 1e-5, 1e-5}.asDiagonal();
  const double dt = 0.1;                                 // [sec]
  const turtle_nav_cpp::GaussianRV speed_rv{0.1, 0.01};  // [m/s]
  const turtle_nav_cpp::GaussianRV yaw_rate_rv{0, 0.1};  // [rad/s]

  // Random number generator
  std::default_random_engine rn_generator = std::default_random_engine();

  const auto [trajectories, pose_covariances] = turtle_nav_cpp::GenerateTrajectories(
    num_trajs, num_poses, T_0, dt, cov_T_0, speed_rv, yaw_rate_rv, rn_generator);

  matplot::figure();
  turtle_nav_cpp::PlotTrajectoryEndPoints(trajectories, 5);
  matplot::axis(matplot::square);
  matplot::grid(true);

  // Get confidence intervals only for the last pose:
  // - Get Chol lower triangular matrix
  // - Get circular points
  // - Get ellipse points in the Lie group
  const int num_ellipse_pts = 100;
  const Eigen::Matrix2d I_2 = Eigen::Matrix2d::Identity();
  const auto circular_points_2d = turtle_nav_cpp::GetEllipsePoints(I_2, 1, num_ellipse_pts);

  // Get Cholesky lower triangular matrix of the last matrix of the last trajectory
  const Eigen::Matrix3d cov_L = pose_covariances.back().back().llt().matrixL();

  // Get the last pose of trajectory
  const auto T_end = trajectories.back().back();

  // Points to plot
  matplot::vector_1d points_x;
  matplot::vector_1d points_y;
  points_x.reserve(num_ellipse_pts);
  points_y.reserve(num_ellipse_pts);

  // Scale ellipse by this number (sqrt(chi2inv(3, 0.999)))
  const double scale = 4.0331422236561565;

  for (const auto & pt : circular_points_2d) {
    // Get circular point in 3D space
    Eigen::Vector3d pt_3d = Eigen::Vector3d::Zero();
    pt_3d(0) = pt(0);
    pt_3d(1) = pt(1);

    // Retract to the group
    const Eigen::Vector3d ell_se2_vec = scale * cov_L * pt_3d;
    const auto T_pt_global =
      T_end * turtle_nav_cpp::Exp(ell_se2_vec.block<2, 1>(0, 0), ell_se2_vec(2));
    ;

    // Append points
    points_x.push_back(T_pt_global.x());
    points_y.push_back(T_pt_global.y());
  }

  matplot::hold(true);
  matplot::plot(points_x, points_y)->line_width(1.5);
  matplot::axis(matplot::square);

  matplot::show();

  return 0;
}
