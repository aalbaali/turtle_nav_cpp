/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file example_pose_uncertainty.cpp
 * @brief Example of propagating uncertainties through SE(2) poses
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Apr-01
 */

#include <matplot/matplot.h>

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <random>
#include <string>
#include <vector>

#include "turtle_nav_cpp/math_utils.hpp"
#include "turtle_nav_cpp/pose.hpp"

using turtle_nav_cpp::nav_utils::Pose;
using Trajectory = std::vector<Pose>;

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

std::vector<Trajectory> GenerateTrajectories(
  const size_t num_trajectories, const size_t num_poses, const Pose & T_0, const double dt,
  const GaussianRV & speed_rv, const GaussianRV & yaw_rate_rv,
  std::default_random_engine & rn_generator)
{
  std::vector<Trajectory> trajectories;
  for (size_t i = 0; i < num_trajectories; i++) {
    // Sample odometry measurements
    // Noisy speeds
    auto speeds = turtle_nav_cpp::GenerateNoisyVector(num_poses - 1, speed_rv, rn_generator);
    auto yaw_rates = turtle_nav_cpp::GenerateNoisyVector(num_poses - 1, yaw_rate_rv, rn_generator);

    // Generate straight-line trajectory
    trajectories.push_back(DeadReckonTrajectory(T_0, dt, speeds, yaw_rates));
  }

  return trajectories;
}

}  // namespace turtle_nav_cpp

int main()
{
  // Number of trajectgories to generate
  const size_t num_trajs = 10;

  // Numbers of poses per trajectory
  const int num_poses = 100;

  // Dead-reckoning parameters
  Pose T_0(0, 0, 0);
  const double dt = 0.1;                                 // sec
  const turtle_nav_cpp::GaussianRV speed_rv{1, 0.001};   // [m/s]
  const turtle_nav_cpp::GaussianRV yaw_rate_rv{0, 0.1};  // [rad/s]

  // Random number generator
  std::default_random_engine rn_generator = std::default_random_engine();

  auto trajectories = turtle_nav_cpp::GenerateTrajectories(
    num_trajs, num_poses, T_0, dt, speed_rv, yaw_rate_rv, rn_generator);

  matplot::figure();
  turtle_nav_cpp::PlotTrajectoryEndPoints(trajectories, 5);
  matplot::xlim(matplot::gca(), {0.0, dt * num_poses * speed_rv.mean});
  matplot::axis(matplot::square);
  matplot::grid(true);
  matplot::show();

  return 0;
}
