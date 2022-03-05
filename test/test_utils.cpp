/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_utils.cpp
 * @brief
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-02
 */

#include "gtest/gtest.h"
#include "turtle_nav_cpp/utils.hpp"

namespace turtle_nav_cpp
{

TEST(Vec2ToMatrix, WrongVectorSize) {
  EXPECT_ANY_THROW(Vec2ToMatrix(std::vector<double>{1.0}));
}

TEST(Vec2ToMatrix, CheckMatrixEntries) {
  auto v = std::vector<double>{0.0, 1.0, 2.0, 3.0};

  // Matrix is inserted in row-major format
  auto mat = Vec2ToMatrix(v);

  for (int col = 0; col < 2; col++) {
    for (int row = 0; row < 2; row++) {
      auto expected_val = static_cast<double>(col * 2 + row);
      auto matrix_val = mat(row, col);
      EXPECT_DOUBLE_EQ(expected_val, matrix_val);
    }
  }
}
}  // turtle_nav_cpp
