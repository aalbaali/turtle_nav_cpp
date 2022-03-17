/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_math_utils.cpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */

#include <functional>
#include <vector>

#include "gtest/gtest.h"
#include "turtle_nav_cpp/math_utils.hpp"

using ::testing::DoubleLE;

namespace turtle_nav_cpp
{
TEST(IsPerfectSquare, DoubleSquareNumber)
{
  EXPECT_TRUE(IsPerfectSquare(0.0));
  EXPECT_TRUE(IsPerfectSquare(4.0));
  EXPECT_FALSE(IsPerfectSquare(3.0));
}

TEST(IsPerfectSquare, IntSquareNumber)
{
  EXPECT_TRUE(IsPerfectSquare(0));
  EXPECT_TRUE(IsPerfectSquare(4));
  EXPECT_FALSE(IsPerfectSquare(3));
}

class RandnTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Set a random number generator
    rn_generator_ = std::default_random_engine();

    // Random number generator
    randn_ = [this]() { return randn_gen(rn_generator_); };
  }

  std::default_random_engine rn_generator_;
  std::function<double()> randn_;
};

TEST_F(RandnTest, DifferentNumbers)
{  // Expect different numbers for different randn() calls
  auto x1 = randn_();
  auto x2 = randn_();

  EXPECT_NE(x1, x2);
}

TEST_F(RandnTest, StatisticalTest)
{
  // Number of samples
  const int N = 1e5;

  // Sample and sum
  double x_sum = 0;
  double var_sum = 0;
  for (int i = 0; i < N; i++) {
    auto x = randn_();
    x_sum += x;
    var_sum += x * x;
  }

  // Get the means
  const double x_mean = x_sum / N;
  const double var_mean = var_sum / N;

  // Mean should be within 4 sigma bounds should be almost 100%
  EXPECT_TRUE(x_mean <= 4 / sqrt(N));
  EXPECT_PRED_FORMAT2(DoubleLE, x_mean, 4 / sqrt(N));
  EXPECT_PRED_FORMAT2(DoubleLE, -x_mean, 4 / sqrt(N));

  // Sample variance 99% confidence intervals
  // Checkout https://www.vrcbuzz.com/confidence-interval-variance-calculator/
  // chi2_quantile bounds computed using Julia and Distributions package. Used the Chisq
  // distribution and quantile function (N = 1e5)
  const double chi2_quantile_upper = 98851.81093911003;
  const double chi2_quantile_lower = 101155.70224030754;

  EXPECT_PRED_FORMAT2(DoubleLE, 1, (N - 1) * var_mean / chi2_quantile_upper);
  EXPECT_PRED_FORMAT2(DoubleLE, -1, -(N - 1) * var_mean / chi2_quantile_lower);
}
}  // namespace turtle_nav_cpp
