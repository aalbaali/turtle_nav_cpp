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
#include <vector>

int main()
{
  std::vector<double> x = matplot::linspace(0, 2 * M_PI);
  std::vector<double> y = matplot::transform(x, [](auto x) { return sin(x); });

  matplot::plot(x, y, "-o");
  matplot::hold(matplot::on);
  matplot::plot(x, matplot::transform(y, [](auto y) { return -y; }), "--xr")->line_width(2.5);
  matplot::plot(x, matplot::transform(x, [](auto x) { return x / M_PI - 1.; }), "-:gs");
  matplot::plot({1.0, 0.7, 0.4, 0.0, -0.4, -0.7, -1}, "k");
  matplot::xlabel("x^2 vals");
  matplot::ylabel("y vals");
  matplot::grid(matplot::on);
  matplot::gca()->minor_grid(true);
  // matplot::show();

  // Attempting to plot ellipse
  matplot::figure();
  // matplot::gca()->clear();
  matplot::fimplicit([](double x, double y) { return pow(x, 2) + pow(y, 2) - 1; });
  matplot::grid(matplot::on);
  matplot::axis(matplot::equal);

  matplot::figure();
  matplot::scatter(x, y);
  show();

  // Eigen::Matrix2

  return 0;
}
