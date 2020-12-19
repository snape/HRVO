/*
 * Vector2.h
 * HRVO Library
 *
 * Copyright 2009 University of North Carolina at Chapel Hill
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
 * \file   Vector2.h
 * \brief  Declares and defines the Vector2 class.
 */

#ifndef HRVO_VECTOR2_H_
#define HRVO_VECTOR2_H_

#include "Export.h"

#include <cmath>
#include <iosfwd>

namespace hrvo {
	/**
	 * \class  Vector2
	 * \brief  A vector in two dimensions.
	 */
	class HRVO_EXPORT Vector2 {
	public:
		/**
		 * \brief  Constructor.
		 */
		Vector2() : x_(0.0f), y_(0.0f) { }

		/**
		 * \brief  Constructor.
		 * \param[in]  x  The x-coordinate of the vector.
		 * \param[in]  y  The y-coordinate of the vector.
		 */
		Vector2(float x, float y) : x_(x), y_(y) { }

		/**
		 * \brief   Returns the x-coordinate of this vector.
		 * \return  The x-coordinate of the vector.
		 */
		float getX() const { return x_; }

		/**
		 * \brief   Returns the y-coordinate of this vector.
		 * \return  The y-coordinate of the vector.
		 */
		float getY() const { return y_; }

		/**
		 * \brief      Sets the x-coordinate of this vector.
		 * \param[in]  x  The replacement x-coordinate.
		 */
		void setX(float x) { x_ = x; }

		/**
		 * \brief      Sets the y-coordinate of this vector.
		 * \param[in]  y  The replacement y-coordinate.
		 */
		void setY(float y) { y_ = y; }

		/**
		 * \brief   Computes the negation of this vector.
		 * \return  The negation of this vector.
		 */
		Vector2 operator-() const
		{
			return Vector2(-x_, -y_);
		}

		/**
		 * \brief      Computes the dot product of this vector with the specified vector.
		 * \param[in]  other  The vector with which the dot product should be computed.
		 * \return     The dot product of this vector with a specified vector.
		 */
		float operator*(const Vector2 &other) const
		{
			return x_ * other.x_ + y_ * other.y_;
		}

		/**
		 * \brief      Computes the scalar multiplication of this vector with the specified scalar value.
		 * \param[in]  scalar  The scalar value with which the scalar multiplication should be computed.
		 * \return     The scalar multiplication of this vector with a specified scalar value.
		 */
		Vector2 operator*(float scalar) const
		{
			return Vector2(x_ * scalar, y_ * scalar);
		}

		/**
		 * \brief      Computes the scalar division of this vector with the specified scalar value.
		 * \param[in]  scalar  The scalar value with which the scalar division should be computed.
		 * \return     The scalar division of this vector with a specified scalar value.
		 */
		Vector2 operator/(float scalar) const
		{
			const float invScalar = 1.0f / scalar;

			return Vector2(x_ * invScalar, y_ * invScalar);
		}

		/**
		 * \brief      Computes the vector sum of this vector with the specified vector.
		 * \param[in]  other  The vector with which the vector sum should be computed.
		 * \return     The vector sum of this vector with a specified vector.
		 */
		Vector2 operator+(const Vector2 &other) const
		{
			return Vector2(x_ + other.x_, y_ + other.y_);
		}

		/**
		 * \brief      Computes the vector difference of this vector with the specified vector.
		 * \param[in]  other  The vector with which the vector difference should be computed.
		 * \return     The vector difference of this vector with a specified vector.
		 */
		Vector2 operator-(const Vector2 &other) const
		{
			return Vector2(x_ - other.x_, y_ - other.y_);
		}

		/**
		 * \brief      Tests this vector for equality with the specified vector.
		 * \param[in]  other  The vector with which to test for equality.
		 * \return     True if the vectors are equal.
		 */
		bool operator==(const Vector2 &other) const
		{
			return x_ == other.x_ && y_ == other.y_;
		}

		/**
		 * \brief      Tests this vector for inequality with the specified vector.
		 * \param[in]  other  The vector with which to test for inequality
		 * \return     True if the vectors are not equal.
		 */
		bool operator!=(const Vector2 &other) const
		{
			return !(*this == other);
		}

		/**
		 * \brief      Sets the value of this vector to the scalar multiplication of itself with the specified scalar value.
		 * \param[in]  scalar  The scalar value with which the scalar multiplication should be computed.
		 * \return     A reference to this vector.
		 */
		Vector2 &operator*=(float scalar)
		{
			x_ *= scalar;
			y_ *= scalar;

			return *this;
		}

		/**
		 * \brief      Sets the value of this vector to the scalar division of itself with the specified scalar value.
		 * \param[in]  scalar  The scalar value with which the scalar division should be computed.
		 * \return     A reference to this vector.
		 */
		Vector2 &operator/=(float scalar)
		{
			const float invScalar = 1.0f / scalar;

			x_ *= invScalar;
			y_ *= invScalar;

			return *this;
		}

		/**
		 * \brief      Sets the value of this vector to the vector sum of itself with the specified vector.
		 * \param[in]  other  The vector with which the vector sum should be computed.
		 * \return     A reference to this vector.
		 */
		Vector2 &operator+=(const Vector2 &other)
		{
			x_ += other.x_;
			y_ += other.y_;

			return *this;
		}

		/**
		 * \brief      Sets the value of this vector to the vector difference of itself with the specified vector.
		 * \param[in]  other  The vector with which the vector difference should be computed.
		 * \return     A reference to this vector.
		 */
		Vector2 &operator-=(const Vector2 &other)
		{
			x_ -= other.x_;
			y_ -= other.y_;

			return *this;
		}

	private:
		float x_;
		float y_;
	};

	/**
	 * \relates    Vector2
	 * \brief      Computes the length of a specified vector.
	 * \param[in]  vector  The vector whose length is to be computed.
	 * \return     The length of the vector.
	 */
	HRVO_EXPORT inline float abs(const Vector2 &vector)
	{
		return std::sqrt(vector * vector);
	}

	/**
	 * \relates    Vector2
	 * \brief      Computes the squared length of a specified vector.
	 * \param[in]  vector  The vector whose squared length is to be calculated.
	 * \return     The squared length of the vector.
	 */
	HRVO_EXPORT inline float absSq(const Vector2 &vector)
	{
		return vector * vector;
	}

	/**
	 * \relates    Vector2
	 * \brief      Computes the angle between a specified vector and the positive x-axis.
	 * \param[in]  vector  The vector whose angle with the positive x-axis is to be calculated.
	 * \return     The angle in radians between the vector and the positive x-axis in the range [-PI, PI].
	 */
	HRVO_EXPORT inline float atan(const Vector2 &vector)
	{
		return std::atan2(vector.getY(), vector.getX());
	}

	/**
	 * \relates    Vector2
	 * \brief      Computes the determinant of a square matrix with rows consisting of the specified vectors.
	 * \param[in]  vector1  The top row of the square matrix.
	 * \param[in]  vector2  The bottom row of the square matrix.
	 * \return     The determinant of the square matrix.
	 */
	HRVO_EXPORT inline float det(const Vector2 &vector1, const Vector2 &vector2)
	{
		return vector1.getX() * vector2.getY() - vector1.getY() * vector2.getX();
	}

	/**
	 * \relates    Vector2
	 * \brief      Computes the normalization of a specified vector.
	 * \param[in]  vector  The vector whose normalization is to be calculated.
	 * \return     The normalization of the vector.
	 */
	HRVO_EXPORT inline Vector2 normalize(const Vector2 &vector)
	{
		return vector / abs(vector);
	}

	/**
	 * \relates    Vector2
	 * \brief      Computes the normal to a line segment with the specified end points.
	 * \param[in]  vector1  The first end point of the line segment.
	 * \param[in]  vector2  The second end point of the line segment.
	 * \return     The normal vector of the line segment.
	 */
	HRVO_EXPORT inline Vector2 normal(const Vector2 &vector1, const Vector2 &vector2)
	{
		return normalize(Vector2(vector2.getY() - vector1.getY(), vector1.getX() - vector2.getX()));
	}

	/**
	 * \relates    Vector2
	 * \brief      Computes the scalar multiplication of the specified vector with the specified scalar value.
	 * \param[in]  scalar  The scalar value with which the scalar multiplication should be computed.
	 * \param[in]  vector  The vector with which the scalar multiplication should be computed.
	 * \return     The scalar multiplication of the vector with the scalar value.
	 */
	HRVO_EXPORT inline Vector2 operator*(float scalar, const Vector2 &vector)
	{
		return Vector2(scalar * vector.getX(), scalar * vector.getY());
	}

	/**
	 * \relates    Vector2
	 * \brief      Inserts the specified two-dimensional vector into the specified output stream.
	 * \param[in]  stream  The output stream into which the two-dimensional vector should be inserted.
	 * \param[in]  vector  The two-dimensional vector which to insert into the output stream.
	 * \return     A reference to the output stream.
	 */
	HRVO_EXPORT std::ostream &operator<<(std::ostream &stream, const Vector2 &vector);
}

#endif /* HRVO_VECTOR2_H_ */
