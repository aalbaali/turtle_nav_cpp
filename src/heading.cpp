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
Heading::Heading() : rotation_(0) {}

Heading::Heading(double theta) : rotation_(WrapToPi(theta)) {}

Heading::Heading(const Eigen::Quaterniond q) : rotation_(QuaternionToHeading(q)) {}

Heading::Heading(const geometry_msgs::msg::Quaternion q)
{
  rotation_ = Eigen::Rotation2Dd(QuaternionMsgToHeading(q));
}

double Heading::angle() const { return rotation_.angle(); }

Heading Heading::Inverse() const { return Heading(-this->angle()); }

Eigen::Rotation2Dd Heading::Rotation() const { return rotation_; }

Eigen::Matrix2d Heading::RotationMatrix() const { return rotation_.toRotationMatrix(); }

Eigen::Quaterniond Heading::Quaternion() const { return HeadingToQuaternion(this->angle()); }

geometry_msgs::msg::Quaternion Heading::QuaternionMsg() const
{
  return HeadingToQuaternionMsg(this->angle());
}

Heading & Heading::operator=(double theta)
{
  rotation_ = Eigen::Rotation2Dd(WrapToPi(theta));
  return *this;
}

Heading & Heading::operator=(const Eigen::Quaterniond q)
{
  rotation_ = Eigen::Rotation2Dd(QuaternionToHeading(q));
  return *this;
}

Heading & Heading::operator=(const geometry_msgs::msg::Quaternion q_msg)
{
  auto q = QuaternionMsgToQuaternion(q_msg);
  q.normalize();
  rotation_ = Eigen::Rotation2Dd(QuaternionToHeading(q));
  return *this;
}

Heading Heading::operator*(double other) const { return Heading(this->angle() * other); }

Heading Heading::operator+(const Heading & other) const
{
  return Heading(this->angle() + other.angle());
}

Heading & Heading::operator+=(const Heading & other)
{
  *this = *this + other;
  return *this;
}

Heading Heading::operator+(double other) const { return Heading(this->angle() + other); }

Heading & Heading::operator+=(double other)
{
  *this = *this + other;
  return *this;
}

Heading Heading::operator-(const Heading & other) const
{
  return Heading(this->angle() - other.angle());
}

Heading Heading::operator-(double other) const { return Heading(this->angle() - other); }

bool Heading::operator==(const Heading & other) const { return (this->angle() == other.angle()); }

bool Heading::operator!=(const Heading & other) const { return !(*this == other); }

std::ostream & operator<<(std::ostream & os, const Heading & heading)
{
  return os << heading.angle();
}
}  // namespace nav_utils
}  // namespace turtle_nav_cpp
