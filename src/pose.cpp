/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file pose.cpp
 * @brief Implementation of pose.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-21
 */

#include "turtle_nav_cpp/pose.hpp"

#include "turtle_nav_cpp/heading.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
//==================================================================================================
// Constructors
//==================================================================================================
Pose::Pose() : position_{0, 0}, heading_(0) {}

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

Pose Pose::Inverse() const
{
  const auto heading_inverse = this->heading().Inverse();
  const Vector2d position_inverse = -heading_inverse.RotationMatrix() * this->translation();

  return Pose(position_inverse, heading_inverse);
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

Pose Pose::operator*(const Pose & other) const
{
  Heading heading_2 = heading_ + other.heading();
  Vector2d position_2 = position_ + other.translation();

  return Pose(position_2, heading_2);
}

Vector2d Pose::operator*(const Vector2d & v) const { return Affine() * v; }

Pose & Pose::operator*=(const Pose & other)
{
  heading_ += other.heading();
  position_ += other.translation();

  return *this;
}

bool Pose::operator==(const Pose & pose_rhs) const
{
  return (this->translation() == pose_rhs.translation()) && (this->angle() == pose_rhs.angle());
}

bool Pose::operator!=(const Pose & pose_rhs) const { return !(*this == pose_rhs); }

std::ostream & operator<<(std::ostream & os, const Pose & pose)
{
  return os << "x: " << pose.x() << ", y: " << pose.y() << ", theta: " << pose.angle();
}

}  // namespace nav_utils
}  // namespace turtle_nav_cpp
