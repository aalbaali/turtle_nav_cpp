/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file test_nav_utils.cpp
 * @brief Test nav_utils.cpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-22
 */

#include <array>
#include <geometry_msgs/msg/pose.hpp>
#include <turtlesim/msg/pose.hpp>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "gtest/gtest.h"
#include "turtle_nav_cpp/nav_utils.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
//==================================================================================================
// Heading
//==================================================================================================

TEST(HeadingToQuaternion, CheckQuaternion)
{
  const double heading = M_PI_4;

  // The online calculator has been used to calculate the quaternion matrix
  // https://www.andre-gaschler.com/rotationconverter/
  auto q = HeadingToQuaternion(heading);

  EXPECT_DOUBLE_EQ(q.x(), 0);
  EXPECT_DOUBLE_EQ(q.y(), 0);
  EXPECT_DOUBLE_EQ(q.z(), sin(heading / 2));
  EXPECT_DOUBLE_EQ(q.w(), cos(heading / 2));
}

TEST(HeadingToQuaternion, CheckRotationMatrix)
{
  const double heading = M_PI_4;

  auto C = HeadingToQuaternion(heading).toRotationMatrix();

  EXPECT_DOUBLE_EQ(C(0, 0), cos(heading));
  EXPECT_DOUBLE_EQ(C(0, 1), -sin(heading));
  EXPECT_DOUBLE_EQ(C(1, 0), sin(heading));
  EXPECT_DOUBLE_EQ(C(1, 1), cos(heading));
  EXPECT_DOUBLE_EQ(C(0, 2), 0);
  EXPECT_DOUBLE_EQ(C(1, 2), 0);
  EXPECT_DOUBLE_EQ(C(2, 0), 0);
  EXPECT_DOUBLE_EQ(C(2, 1), 0);
  EXPECT_DOUBLE_EQ(C(2, 2), 1);
}

TEST(QuaternionToHeading, ZeroHeadingQuaternion)
{
  Eigen::Quaterniond q;
  q.x() = 0;
  q.y() = 0;
  q.z() = 0;
  q.w() = 1;
  q.normalize();

  EXPECT_DOUBLE_EQ(QuaternionToHeading(q), 0);
}

TEST(QuaternionToHeading, NonZeroHeadingQuaternion)
{
  const double heading = M_PI_4;

  Eigen::Quaterniond q(Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));

  EXPECT_DOUBLE_EQ(QuaternionToHeading(q), heading);

  // The negative of the quaternion should give the same heading
  q.x() = q.x();
  q.y() = q.y();
  q.z() = q.z();
  q.w() = q.w();
  EXPECT_DOUBLE_EQ(QuaternionToHeading(q), heading);
}

TEST(QuaternionMsgToQuaternion, EquateQuaternions)
{
  const double heading = M_PI_4;

  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = 0;
  q_msg.y = 0;
  q_msg.z = sin(heading / 2);
  q_msg.w = cos(heading / 2);

  auto q_eigen = QuaternionMsgToQuaternion(q_msg);
  EXPECT_DOUBLE_EQ(q_msg.x, q_eigen.x());
  EXPECT_DOUBLE_EQ(q_msg.y, q_eigen.y());
  EXPECT_DOUBLE_EQ(q_msg.z, q_eigen.z());
  EXPECT_DOUBLE_EQ(q_msg.w, q_eigen.w());
}

TEST(QuaternionToQuaternionMsg, EquateQuaternions)
{
  const double heading = M_PI_4;
  Eigen::Quaterniond q(cos(heading / 2), 0, 0, sin(heading / 2));

  auto q_msg = QuaternionToQuaternionMsg(q);

  EXPECT_DOUBLE_EQ(q_msg.x, q.x());
  EXPECT_DOUBLE_EQ(q_msg.y, q.y());
  EXPECT_DOUBLE_EQ(q_msg.z, q.z());
  EXPECT_DOUBLE_EQ(q_msg.w, q.w());
}

TEST(QuaternionMsgToHeading, Headings)
{
  const double heading = M_PI_4;

  geometry_msgs::msg::Quaternion q_msg;
  q_msg.x = 0;
  q_msg.y = 0;
  q_msg.z = sin(heading / 2);
  q_msg.w = cos(heading / 2);

  EXPECT_DOUBLE_EQ(QuaternionMsgToHeading(q_msg), heading);

  // Multiply by -1 should also be the same
  q_msg.z = -q_msg.z;
  q_msg.w = -q_msg.w;
  EXPECT_DOUBLE_EQ(QuaternionMsgToHeading(q_msg), heading);
}

TEST(HeadingToQuaternionMsg, Headings)
{
  const double heading = M_PI_4;

  auto q_msg = HeadingToQuaternionMsg(heading);

  EXPECT_DOUBLE_EQ(q_msg.x, 0);
  EXPECT_DOUBLE_EQ(q_msg.y, 0);
  EXPECT_DOUBLE_EQ(q_msg.z, sin(heading / 2));
  EXPECT_DOUBLE_EQ(q_msg.w, cos(heading / 2));
}

//==================================================================================================
// Poses
//==================================================================================================
class TestPoseFuncs : public ::testing::Test
{
protected:
  void SetUp() override
  {
    x = 1.0;
    y = 2.0;
    heading = M_PI_4;
  }

  double x;
  double y;
  double heading;
};

TEST_F(TestPoseFuncs, PoseMsgToTurtlePose)
{
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = x;
  pose_msg.position.y = y;
  pose_msg.position.z = 0;
  pose_msg.orientation.x = 0;
  pose_msg.orientation.y = 0;
  pose_msg.orientation.z = sin(heading / 2);
  pose_msg.orientation.w = cos(heading / 2);

  turtlesim::msg::Pose pose_turtle = PoseMsgToTurtlePose(pose_msg);

  EXPECT_FLOAT_EQ(pose_turtle.x, x);
  EXPECT_FLOAT_EQ(pose_turtle.y, y);
  EXPECT_FLOAT_EQ(pose_turtle.theta, heading);
}

TEST_F(TestPoseFuncs, TurtlePoseToPoseMsg)
{
  turtlesim::msg::Pose pose_turtle;
  pose_turtle.x = x;
  pose_turtle.y = y;
  pose_turtle.theta = heading;

  auto pose_msg = TurtlePoseToPoseMsg(pose_turtle);

  EXPECT_FLOAT_EQ(pose_msg.position.x, x);
  EXPECT_FLOAT_EQ(pose_msg.position.y, y);
  EXPECT_FLOAT_EQ(pose_msg.position.z, 0);
  EXPECT_FLOAT_EQ(pose_msg.orientation.x, 0);
  EXPECT_FLOAT_EQ(pose_msg.orientation.y, 0);
  EXPECT_FLOAT_EQ(pose_msg.orientation.z, sin(heading / 2));
  EXPECT_FLOAT_EQ(pose_msg.orientation.w, cos(heading / 2));
}

TEST_F(TestPoseFuncs, RetractSe2CovarianceEllipse)
{
  // When theta is nonzero, the mapped points for a "banana shape", which is nontrivial to compute,
  // thus difficult to test for.
  // Therefore, a heading angle of 0 will be used, which results in a regular ellipse shape, except
  // that it's centered at the pose location instead of the origin (0, 0).
  const Pose pose(x, y, 0.0);

  const auto retracted_pts = RetractSe2CovarianceEllipse(pose, Eigen::Matrix3d::Identity(), 1, 5);

  // Angles
  const std::vector<double> angles{-M_PI, -M_PI_2, 0, M_PI_2, M_PI};

  for (int i = 0; i < 5; i++) {
    EXPECT_DOUBLE_EQ(retracted_pts[i](0), x + cos(angles[i]));
    EXPECT_DOUBLE_EQ(retracted_pts[i](1), y + sin(angles[i]));
  }
}

//==================================================================================================
// Covarainces
//==================================================================================================
class TestPoseCov : public ::testing::Test
{
protected:
  void SetUp() override
  {
    var_x_ = 1.0;
    var_y_ = 2.0;
    var_th_ = 3.0;
    cov_xy_ = 0.1;
    cov_xth_ = 0.2;
    cov_yth_ = 0.2;
  }

  double var_x_;
  double var_y_;
  double var_th_;

  double cov_xy_;
  double cov_xth_;
  double cov_yth_;
};

TEST_F(TestPoseCov, ThreeDofToTwoDof)
{
  Eigen::Matrix<double, 6, 6> cov_3dof;
  // clang-format off
  cov_3dof << var_x_  , cov_xy_ , 0, 0, 0, cov_xth_,
              cov_xy_ , var_y_  , 0, 0, 0, cov_yth_,
                  0   ,   0     , 0, 0, 0,    0    ,
                  0   ,   0     , 0, 0, 0,    0    ,
                  0   ,   0     , 0, 0, 0,    0    ,
              cov_xth_, cov_yth_, 0, 0, 0, var_th_ ;
  // clang-format on

  const auto cov_2dof = Cov3dofToCov2dof(cov_3dof);

  // Ensure symmetry
  EXPECT_TRUE(cov_2dof.isApprox(cov_2dof.transpose()));

  // Elements
  EXPECT_DOUBLE_EQ(cov_2dof(0, 0), var_x_);
  EXPECT_DOUBLE_EQ(cov_2dof(0, 1), cov_xy_);
  EXPECT_DOUBLE_EQ(cov_2dof(0, 2), cov_xth_);
  EXPECT_DOUBLE_EQ(cov_2dof(1, 0), cov_xy_);
  EXPECT_DOUBLE_EQ(cov_2dof(1, 1), var_y_);
  EXPECT_DOUBLE_EQ(cov_2dof(1, 2), cov_yth_);
  EXPECT_DOUBLE_EQ(cov_2dof(2, 0), cov_xth_);
  EXPECT_DOUBLE_EQ(cov_2dof(2, 1), cov_yth_);
  EXPECT_DOUBLE_EQ(cov_2dof(2, 2), var_th_);
}

TEST_F(TestPoseCov, ThreeDofMsgToTwoDof)
{
  // clang-format off
  std::array<double, 36> cov_3dof_msg = {
    var_x_  , cov_xy_ , 0, 0, 0, cov_xth_,
    cov_xy_ , var_y_  , 0, 0, 0, cov_yth_,
        0   ,   0     , 0, 0, 0,    0    ,
        0   ,   0     , 0, 0, 0,    0    ,
        0   ,   0     , 0, 0, 0,    0    ,
    cov_xth_, cov_yth_, 0, 0, 0, var_th_
  };
  // clang-format on

  const auto cov_2dof = Cov3dofMsgToCov2dof(cov_3dof_msg);

  // Ensure symmetry
  EXPECT_TRUE(cov_2dof.isApprox(cov_2dof.transpose()));

  // Elements
  EXPECT_DOUBLE_EQ(cov_2dof(0, 0), var_x_);
  EXPECT_DOUBLE_EQ(cov_2dof(0, 1), cov_xy_);
  EXPECT_DOUBLE_EQ(cov_2dof(0, 2), cov_xth_);
  EXPECT_DOUBLE_EQ(cov_2dof(1, 0), cov_xy_);
  EXPECT_DOUBLE_EQ(cov_2dof(1, 1), var_y_);
  EXPECT_DOUBLE_EQ(cov_2dof(1, 2), cov_yth_);
  EXPECT_DOUBLE_EQ(cov_2dof(2, 0), cov_xth_);
  EXPECT_DOUBLE_EQ(cov_2dof(2, 1), cov_yth_);
  EXPECT_DOUBLE_EQ(cov_2dof(2, 2), var_th_);
}

TEST_F(TestPoseCov, TwoDofToThreeDof)
{
  Eigen::Matrix3d cov_2dof;
  // clang-format off
  cov_2dof << var_x_  , cov_xy_ , cov_xth_,
              cov_xy_ , var_y_  , cov_yth_,
              cov_xth_, cov_yth_, var_th_ ;
  // clang-format on

  const auto cov_3dof = Cov2dofToCov3dof(cov_2dof);

  // Ensure symmetry
  EXPECT_TRUE(cov_3dof.isApprox(cov_3dof.transpose()));

  // Elements
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::x, 0), var_x_);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::x, 1), cov_xy_);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::x, 2), 0);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::x, 3), 0);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::x, 4), 0);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::x, 5), cov_xth_);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::y, 0), cov_xy_);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::y, 1), var_y_);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::y, 2), 0);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::y, 3), 0);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::y, 4), 0);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::y, 5), cov_yth_);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::th, 0), cov_xth_);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::th, 1), cov_yth_);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::th, 2), 0);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::th, 3), 0);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::th, 4), 0);
  EXPECT_DOUBLE_EQ(cov_3dof(ThreeDof::th, 5), var_th_);
}

TEST_F(TestPoseCov, TwoDofToThreeDofMsg)
{
  Eigen::Matrix3d cov_2dof;
  // clang-format off
  cov_2dof << var_x_  , cov_xy_ , cov_xth_,
              cov_xy_ , var_y_  , cov_yth_,
              cov_xth_, cov_yth_, var_th_ ;
  // clang-format on

  const auto cov_3dof_msg = Cov2dofToCov3dofMsg(cov_2dof);

  // Elements
  // The `6` multiplier is to skip each row
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::x + 0], var_x_);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::x + 1], cov_xy_);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::x + 2], 0);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::x + 3], 0);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::x + 4], 0);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::y + 5], cov_xth_);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::y + 0], cov_xy_);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::y + 1], var_y_);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::y + 2], 0);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::y + 3], 0);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::y + 4], 0);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::x + 5], cov_yth_);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::th + 0], cov_xth_);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::th + 1], cov_yth_);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::th + 2], 0);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::th + 3], 0);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::th + 4], 0);
  EXPECT_DOUBLE_EQ(cov_3dof_msg[6 * ThreeDof::th + 5], var_th_);
}
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
