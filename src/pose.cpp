/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file pose.cpp
 * @brief Implementation of pose.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-21
 */

#include "turtle_nav_cpp/pose.hpp"

#include <limits>

#include "turtle_nav_cpp/heading.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
//==================================================================================================
// Constructors
//==================================================================================================
Pose::Pose() : position_{0, 0}, heading_(0) {}

Pose::Pose(const Affine2d & affine) : position_(affine.translation()), heading_(affine.linear()) {}

Pose::Pose(const Vector2d & pos, const Heading & heading) : position_(pos), heading_(heading) {}

Pose::Pose(const Vector2d & pos, double heading) : position_(pos), heading_(heading) {}

Pose::Pose(double x, double y, double heading) : position_{x, y}, heading_(heading) {}

Pose::Pose(const geometry_msgs::msg::Pose & pose)
: position_{pose.position.x, pose.position.y}, heading_(pose.orientation)
{
}

Pose::Pose(const turtlesim::msg::Pose & pose) : position_{pose.x, pose.y}, heading_(pose.theta) {}

//==================================================================================================
// Getters
//==================================================================================================

geometry_msgs::msg::Pose Pose::PoseMsg() const
{
  geometry_msgs::msg::Pose pose_msg;
  pose_msg.position.x = x();
  pose_msg.position.y = y();
  pose_msg.position.z = 0;
  pose_msg.orientation = heading_.QuaternionMsg();

  return pose_msg;
}

turtlesim::msg::Pose Pose::TurtlePose() const
{
  turtlesim::msg::Pose pose_turtle;
  pose_turtle.x = x();
  pose_turtle.y = y();
  pose_turtle.theta = angle();

  return pose_turtle;
}

Affine2d Pose::Affine() const
{
  Affine2d T;
  T.translation() = position_;

  // Confusingly enough, this is the rotation part
  T.linear() = heading_.RotationMatrix();

  return T;
}

Vector2d Pose::translation() const { return position_; }

Heading Pose::heading() const { return heading_; }

double Pose::x() const { return position_(0); }

double Pose::y() const { return position_(1); }

double Pose::angle() const { return heading_.angle(); }

//==================================================================================================
// Operators
//==================================================================================================
Pose & Pose::operator=(const Eigen::Affine2d & affine)
{
  heading_ = Heading(affine.linear());
  position_ = affine.translation();

  return *this;
}

Pose & Pose::operator=(const geometry_msgs::msg::Pose & pose)
{
  heading_ = pose.orientation;
  position_ = Vector2d{pose.position.x, pose.position.y};

  return *this;
}

Pose & Pose::operator=(const turtlesim::msg::Pose & pose_in)
{
  heading_ = pose_in.theta;
  position_ = Vector2d{pose_in.x, pose_in.y};

  return *this;
}

Pose Pose::operator*(const Pose & other) const { return this->Affine() * other.Affine(); }

Vector2d Pose::operator*(const Vector2d & v) const { return Affine() * v; }

Pose & Pose::operator*=(const Pose & other)
{
  Pose pose_out = *this * other;
  position_ = pose_out.translation();
  heading_ = pose_out.heading();
  return *this;
}

bool Pose::operator==(const Pose & pose_rhs) const
{
  return (this->translation() == pose_rhs.translation()) && (this->angle() == pose_rhs.angle());
}

bool Pose::operator!=(const Pose & pose_rhs) const { return !(*this == pose_rhs); }

//================================================================================================
// Lie group operations
//================================================================================================

Pose Pose::Inverse() const
{
  const auto heading_inverse = this->heading().Inverse();
  const Vector2d position_inverse = -heading_inverse.RotationMatrix() * this->translation();

  return Pose(position_inverse, heading_inverse);
}

Eigen::Matrix3d Pose::Adjoint() const
{
  return Pose(this->y(), -this->x(), this->angle()).Affine().matrix();
}

Pose Pose::Exp(const Eigen::Vector2d & rho, const double theta)
{
  // If theta == 0, then rho == r
  if (abs(theta) <= std::numeric_limits<double>::min()) {
    return Pose(rho, theta);
  }

  // From (156) and (158) of Sola
  const Eigen::Matrix2d V =
    sin(theta) / theta * Eigen::Matrix2d::Identity() + (1 - cos(theta)) / theta * Heading::cross(1);

  return Pose(V * rho, theta);
}

std::ostream & operator<<(std::ostream & os, const Pose & pose)
{
  return os << "x: " << pose.x() << ", y: " << pose.y() << ", theta: " << pose.angle();
}

}  // namespace nav_utils
}  // namespace turtle_nav_cpp
