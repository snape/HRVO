/*
 * Vector2.cc
 * HRVO Library
 *
 * SPDX-FileCopyrightText: 2009 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/HRVO/>
 */

/**
 * @file  Vector2.cc
 * @brief Defines the Vector2 class.
 */

#include "Vector2.h"

#include <cmath>
#include <ostream>

namespace hrvo {
Vector2::Vector2() : x_(0.0F), y_(0.0F) {}

Vector2::Vector2(float x, float y) : x_(x), y_(y) {}

Vector2 Vector2::operator-() const { return Vector2(-x_, -y_); }

float Vector2::operator*(const Vector2 &other) const {
  return x_ * other.x_ + y_ * other.y_;
}

Vector2 Vector2::operator*(float scalar) const {
  return Vector2(x_ * scalar, y_ * scalar);
}

Vector2 Vector2::operator/(float scalar) const {
  const float invScalar = 1.0F / scalar;

  return Vector2(x_ * invScalar, y_ * invScalar);
}

Vector2 Vector2::operator+(const Vector2 &other) const {
  return Vector2(x_ + other.x_, y_ + other.y_);
}

Vector2 Vector2::operator-(const Vector2 &other) const {
  return Vector2(x_ - other.x_, y_ - other.y_);
}

bool Vector2::operator==(const Vector2 &other) const {
  return x_ == other.x_ && y_ == other.y_;
}

bool Vector2::operator!=(const Vector2 &other) const {
  return !(*this == other);
}

Vector2 &Vector2::operator*=(float scalar) {
  x_ *= scalar;
  y_ *= scalar;

  return *this;
}

Vector2 &Vector2::operator/=(float scalar) {
  const float invScalar = 1.0F / scalar;

  x_ *= invScalar;
  y_ *= invScalar;

  return *this;
}

Vector2 &Vector2::operator+=(const Vector2 &other) {
  x_ += other.x_;
  y_ += other.y_;

  return *this;
}

Vector2 &Vector2::operator-=(const Vector2 &other) {
  x_ -= other.x_;
  y_ -= other.y_;

  return *this;
}

float abs(const Vector2 &vector) { return std::sqrt(vector * vector); }

float absSq(const Vector2 &vector) { return vector * vector; }

float atan(const Vector2 &vector) {
  return std::atan2(vector.getY(), vector.getX());
}

float det(const Vector2 &vector1, const Vector2 &vector2) {
  return vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();
}

Vector2 normalize(const Vector2 &vector) { return vector / abs(vector); }

Vector2 normal(const Vector2 &vector1, const Vector2 &vector2) {
  return normalize(Vector2(vector2.getY() - vector1.getY(),
                           vector1.getX() - vector2.getX()));
}

Vector2 operator*(float scalar, const Vector2 &vector) {
  return Vector2(scalar * vector.getX(), scalar * vector.getY());
}

std::ostream &operator<<(std::ostream &stream, const Vector2 &vector) {
  stream << vector.getX() << " " << vector.getY();

  return stream;
}
} /* namespace hrvo */
