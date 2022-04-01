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
#include <iostream>
#include <random>
#include <vector>

#include "turtle_nav_cpp/math_utils.hpp"
#include "turtle_nav_cpp/pose.hpp"

namespace turtle_nav_cpp
{
void PlotPoses(const std::vector<nav_utils::Pose> & poses)
{
  // Get x and y values
  std::vector<double> xvals;
  std::vector<double> yvals;

  std::for_each(poses.begin(), poses.end(), [&](const nav_utils::Pose & pose) {
    xvals.push_back(pose.translation().x());
    yvals.push_back(pose.translation().y());
  });

  matplot::plot(xvals, yvals);
  matplot::grid(matplot::on);
}

std::vector<nav_utils::Pose> GenerateTrajectory(
  const nav_utils::Pose & T_0, const std::vector<nav_utils::Pose> & dT_vecs)
{
  std::vector<nav_utils::Pose> poses;
  poses.reserve(dT_vecs.size() + 1);
  poses.assign(100, nav_utils::Pose(0, 0, 0));
  poses[0] = T_0;
  for (size_t i = 1; i < dT_vecs.size(); i++) {
    poses[i] = poses[i - 1] * dT_vecs[i - 1];
  }

  return poses;
}
}  // namespace turtle_nav_cpp

using turtle_nav_cpp::nav_utils::Pose;

int main()
{
  Pose T_0(0, 0, 0);

  // Number of states
  const int num_poses = 100;

  // Let the robot drive in a straight line with constant speed
  const double dt = 0.01;     // sec
  const double speed = 1;     // m/s
  const double yaw_rate = 0;  // rad/s

  // The incremental change transformation matrix
  const Pose dT(dt * speed, 0, dt * yaw_rate);
  std::vector<Pose> dT_vecs;
  dT_vecs.assign(num_poses - 1, dT);

  const auto poses = turtle_nav_cpp::GenerateTrajectory(T_0, dT_vecs);

  for (const auto & p : poses) {
    std::cout << p << std::endl;
  }

  matplot::figure();
  turtle_nav_cpp::PlotPoses(poses);
  matplot::xlabel("x [m]");
  matplot::ylabel("y [m]");
  matplot::title("Robot trajectory");
  matplot::show();

  return 0;
}
