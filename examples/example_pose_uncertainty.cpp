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
}  // namespace turtle_nav_cpp

using turtle_nav_cpp::nav_utils::Pose;

int main()
{
  Pose T_1(0, 0, 0);

  // Let the robot drive in a straight line with constant speed
  const double dt = 0.01;     // sec
  const double speed = 1;     // m/s
  const double yaw_rate = 0;  // rad/s

  // The incremental change transformation matrix
  const Pose dT(dt * speed, 0, dt * yaw_rate);

  // Number of states
  const int num_poses = 100;

  std::vector<Pose> poses;
  poses.reserve(num_poses);
  poses.assign(100, Pose(0, 0, 0));
  poses[0] = T_1;
  for (int i = 1; i < num_poses; i++) {
    poses[i] = poses[i - 1] * dT;
  }

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
