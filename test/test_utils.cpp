/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_utils.cpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */

#include <vector>

#include "gtest/gtest.h"
#include "turtle_nav_cpp/ros_utils.hpp"

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

}  // namespace turtle_nav_cpp
