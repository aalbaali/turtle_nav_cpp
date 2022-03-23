/**
 * Copyright 2022 Ⓒ Amro Al-Baali
 * @file heading.cpp
 * @brief Implementation of Heading.hpp
 * @author Amro Al-Baali (albaalia@live.com)
 * @date 2022-Mar-21
 */

#include "turtle_nav_cpp/heading.hpp"

#include <Eigen/Dense>

#include "turtle_nav_cpp/math_utils.hpp"
#include "turtle_nav_cpp/nav_utils.hpp"

namespace turtle_nav_cpp
{
namespace nav_utils
{
Heading::Heading() : heading_(0) {}

Heading::Heading(double heading) : heading_(WrapToPi(heading)) {}

Heading::Heading(const Eigen::Quaterniond q) : heading_(QuaternionToHeading(q)) {}

Heading::Heading(const geometry_msgs::msg::Quaternion q)
{
  heading_ = Eigen::Rotation2Dd(QuaternionMsgToHeading(q));
}

double Heading::Angle() const { return heading_.angle(); }

Eigen::Rotation2Dd Heading::Rotation() const { return heading_; }

Eigen::Matrix2d Heading::RotationMatrix() const { return heading_.toRotationMatrix(); }

Eigen::Quaterniond Heading::Quaternion() const { return HeadingToQuaternion(this->Angle()); }

geometry_msgs::msg::Quaternion Heading::QuaternionMsg() const
{
  return HeadingToQuaternionMsg(this->Angle());
}

Heading & Heading::operator=(double heading)
{
  heading_ = Eigen::Rotation2Dd(WrapToPi(heading));
  return *this;
}

Heading & Heading::operator=(const Eigen::Quaterniond q)
{
  heading_ = Eigen::Rotation2Dd(QuaternionToHeading(q));
  return *this;
}

Heading & Heading::operator=(const geometry_msgs::msg::Quaternion q_msg)
{
  auto q = QuaternionMsgToQuaternion(q_msg);
  q.normalize();
  heading_ = Eigen::Rotation2Dd(QuaternionToHeading(q));
  return *this;
}

Heading Heading::operator*(double other) const { return Heading(this->Angle() * other); }

Heading Heading::operator+(const Heading & other) const
{
  return Heading(this->Angle() + other.Angle());
}

Heading & Heading::operator+=(const Heading & other)
{
  *this = *this + other;
  return *this;
}

Heading Heading::operator+(double other) const { return Heading(this->Angle() + other); }

Heading & Heading::operator+=(double other)
{
  *this = *this + other;
  return *this;
}

Heading Heading::operator-(const Heading & other) const
{
  return Heading(this->Angle() - other.Angle());
}

Heading Heading::operator-(double other) const { return Heading(this->Angle() - other); }

std::ostream & operator<<(std::ostream & os, const Heading & heading)
{
  return os << heading.Angle();
}
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
