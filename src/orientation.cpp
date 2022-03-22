/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file heading.cpp
 * @brief Implementation of heading.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-21
 */

#include "turtle_nav_cpp/orientation.hpp"

#include <Eigen/Dense>

#include "turtle_nav_cpp/math_utils.hpp"
#include "turtle_nav_cpp/nav_utils.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
Orientation::Orientation() : heading_(0) {}

Orientation::Orientation(double heading) : heading_(WrapToPi(heading)) {}

Orientation::Orientation(const Eigen::Quaterniond q) : heading_(QuaternionToHeading(q)) {}

Orientation::Orientation(const geometry_msgs::msg::Quaternion q)
{
  heading_ = Eigen::Rotation2Dd(QuaternionMsgToHeading(q));
}

double Orientation::Angle() const { return heading_.angle(); }

Eigen::Rotation2Dd Orientation::Rotation() const { return heading_; }

Eigen::Matrix2d Orientation::RotationMatrix() const { return heading_.toRotationMatrix(); }

Orientation & Orientation::operator=(double heading)
{
  heading_ = Eigen::Rotation2Dd(WrapToPi(heading));
  return *this;
}

Orientation & Orientation::operator=(const Eigen::Quaterniond q)
{
  heading_ = Eigen::Rotation2Dd(QuaternionToHeading(q));
  return *this;
}

Orientation & Orientation::operator=(const geometry_msgs::msg::Quaternion q_msg)
{
  auto q = QuaternionMsgToQuaternion(q_msg);
  q.normalize();
  heading_ = Eigen::Rotation2Dd(QuaternionToHeading(q));
  return *this;
}

Orientation Orientation::operator*(double other) const
{
  return Orientation(this->Angle() * other);
}

Orientation Orientation::operator+(const Orientation & other) const
{
  return Orientation(this->Angle() + other.Angle());
}

Orientation & Orientation::operator+=(const Orientation & other)
{
  *this = *this + other;
  return *this;
}

Orientation Orientation::operator+(double other) const
{
  return Orientation(this->Angle() + other);
}

Orientation & Orientation::operator+=(double other)
{
  *this = *this + other;
  return *this;
}

Orientation Orientation::operator-(const Orientation & other) const
{
  return Orientation(this->Angle() - other.Angle());
}

Orientation Orientation::operator-(double other) const
{
  return Orientation(this->Angle() - other);
}
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
