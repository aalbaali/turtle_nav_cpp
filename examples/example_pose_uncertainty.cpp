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
#include <vector>

#include "turtle_nav_cpp/math_utils.hpp"
#include "turtle_nav_cpp/pose.hpp"

namespace turtle_nav_cpp
{
/**
 * @brief Plot x-y positions of a vector of pose
 *
 * @param[in] poses Vector of poses to plot
 */
void PlotPoses(const std::vector<nav_utils::Pose> & poses)
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
 * @brief Generate a dead-reckoning trejectory from speed and yaw measurements
 *
 * @param[in] T_0       Initial pose
 * @param[in] dt        Sampling period [s]
 * @param[in] speeds    Vector of speeds [m/s]
 * @param[in] yaw_rates Vector of yaw rates [rad/s]
 * @return std::vector<nav_utils::Pose> Vector of poses (i.e., the trajectory)
 */
std::vector<nav_utils::Pose> DeadReckonTrajectory(
  const nav_utils::Pose & T_0, const double dt, const std::vector<double> & speeds,
  const std::vector<double> & yaw_rates)
{
  const size_t num_poses = speeds.size() + 1;
  std::vector<nav_utils::Pose> poses;
  poses.reserve(num_poses);
  poses.push_back(T_0);

  std::vector<nav_utils::Pose> dT_vecs(num_poses);
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
 * @param[in] mean              Mean of the Gaussian random variable
 * @param[in] stddev            Standard deviation of the Gaussian random variable
 * @param[in] rn_generator      Random number generator
 * @return std::vector<double>  Vector of realizations
 */
std::vector<double> GenerateNoisyVector(
  const size_t num_vals, const double mean, const double stddev,
  std::default_random_engine & rn_generator)
{
  auto randn = [&rn_generator]() { return turtle_nav_cpp::randn_gen(rn_generator); };

  std::vector<double> noisy_vals(num_vals);
  std::generate(noisy_vals.begin(), noisy_vals.end(), [&]() { return mean + stddev * randn(); });

  return noisy_vals;
}
}  // namespace turtle_nav_cpp

using turtle_nav_cpp::nav_utils::Pose;

int main()
{
  Pose T_0(0, 0, 0);

  // Number of states
  const int num_poses = 100;

  // Let the robot drive in a straight line with constant speed
  const double dt = 0.1;      // sec
  const double speed = 1;     // m/s
  const double yaw_rate = 0;  // rad/s

  std::default_random_engine rn_generator = std::default_random_engine();

  // Noisy speeds
  auto speeds = turtle_nav_cpp::GenerateNoisyVector(num_poses - 1, speed, 0.01, rn_generator);
  auto yaw_rates = turtle_nav_cpp::GenerateNoisyVector(num_poses - 1, yaw_rate, 1, rn_generator);

  // Generate straight-line trajectory
  auto poses = turtle_nav_cpp::DeadReckonTrajectory(T_0, dt, speeds, yaw_rates);

  for (const auto & p : poses) {
    std::cout << p << std::endl;
  }

  matplot::figure();
  turtle_nav_cpp::PlotPoses(poses);
  matplot::xlabel("x [m]");
  matplot::ylabel("y [m]");
  matplot::title("Robot trajectory");
  matplot::grid(true);
  matplot::show();

  return 0;
}
