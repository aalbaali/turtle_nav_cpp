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
#include <random>
#include <string>
#include <tuple>
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
Trajectory DeadReckonTrajectory(
  const nav_utils::Pose & T_0, const double dt, const std::vector<double> & speeds,
  const std::vector<double> & yaw_rates)
{
  const size_t num_poses = speeds.size() + 1;
  Trajectory poses;
  poses.reserve(num_poses);
  poses.push_back(T_0);

  Trajectory dT_vecs(num_poses);
  for (size_t i = 1; i < speeds.size(); i++) {
    nav_utils::Pose dT_km1(dt * speeds[i - 1], 0, dt * yaw_rates[i - 1]);
    auto T_k = poses[i - 1] * dT_km1;
    poses.push_back(T_k);
  }

  // Generate trajectory
  return poses;
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

auto GenerateTrajectories(
  const size_t num_trajectories, const size_t num_poses, const Pose & T_0, const double dt,
  const GaussianRV & speed_rv, const GaussianRV & yaw_rate_rv,
  std::default_random_engine & rn_generator)
{
  std::vector<Trajectory> trajectories;
  std::vector<std::vector<double>> speeds_bundle;
  std::vector<std::vector<double>> yaw_rates_bundle;

  for (size_t i = 0; i < num_trajectories; i++) {
    // Sample odometry measurements
    // Noisy speeds
    speeds_bundle.push_back(
      turtle_nav_cpp::GenerateNoisyVector(num_poses - 1, speed_rv, rn_generator));
    yaw_rates_bundle.push_back(
      turtle_nav_cpp::GenerateNoisyVector(num_poses - 1, yaw_rate_rv, rn_generator));

    // Generate straight-line trajectory
    trajectories.push_back(
      DeadReckonTrajectory(T_0, dt, speeds_bundle.back(), yaw_rates_bundle.back()));
  }

  return std::tuple<std::vector<Trajectory>, StdVector2Double, StdVector2Double>(
    trajectories, speeds_bundle, yaw_rates_bundle);
}

}  // namespace turtle_nav_cpp

int main()
{
  // Number of trajectgories to generate
  const size_t num_trajs = 1000;

  // Numbers of poses per trajectory
  const int num_poses = 100;

  // Dead-reckoning parameters
  Pose T_0(0, 0, 0);
  const double dt = 0.01;                                 // [sec]
  const turtle_nav_cpp::GaussianRV speed_rv{1, 0.0};      // [m/s]
  const turtle_nav_cpp::GaussianRV yaw_rate_rv{0, 10.0};  // [rad/s]

  // Random number generator
  std::default_random_engine rn_generator = std::default_random_engine();

  const auto [trajectories, speeds_bundle, yaw_rates_bundle] = turtle_nav_cpp::GenerateTrajectories(
    num_trajs, num_poses, T_0, dt, speed_rv, yaw_rate_rv, rn_generator);

  matplot::figure();
  turtle_nav_cpp::PlotTrajectoryEndPoints(trajectories, 5);
  matplot::axis(matplot::square);
  matplot::grid(true);

  // Generate and plot ellipse
  Eigen::Matrix2d A;
  A << 5, 3, 3, 4;

  // Check for positive definiteness
  const Eigen::LLT<Eigen::Matrix2d> matrix_llt(A);
  // Check for covariance positive semi-definiteness
  if (matrix_llt.info() == Eigen::NumericalIssue) {
    throw std::runtime_error("Covariance matrix possibly non semi-positive definite matrix");
  }

  // Get eigenvalues and eigenvectors
  const Eigen::EigenSolver<Eigen::Matrix2d> eigs(A);
  const Eigen::Matrix2d eig_vectors = eigs.eigenvectors().real();
  Eigen::Matrix2d eig_sqrt_vals = eigs.eigenvalues().real().unaryExpr(&sqrt).asDiagonal();

  // Compute contour
  const size_t num_points = 1e3;
  std::vector<Eigen::Vector2d> ellipse_points(num_points);
  std::vector<double> thetas = matplot::linspace(0, 2 * M_PI, num_points);
  std::transform(thetas.begin(), thetas.end(), ellipse_points.begin(), [&](const double th) {
    const Eigen::Vector2d v{cos(th), sin(th)};
    Eigen::Vector2d p = eig_vectors * eig_sqrt_vals * v;
    return p;
  });

  auto [ellipse_points_x, ellipse_points_y] = turtle_nav_cpp::GetXYPoints(ellipse_points);

  // Plot ellipse
  matplot::figure();
  matplot::plot(ellipse_points_x, ellipse_points_y)->line_width(1.5);
  matplot::grid(true);

  matplot::show();

  return 0;
}
