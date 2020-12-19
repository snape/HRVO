/*
 * Definitions.h
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
 * \file   Definitions.h
 * \brief  Declares and defines internal functions.
 */

#ifndef HRVO_DEFINITIONS_H_
#define HRVO_DEFINITIONS_H_

namespace hrvo {
	/**
	 * \brief  A sufficiently small positive float.
	 */
	const float HRVO_EPSILON = 0.00001f;

	/**
	 * \brief      Computes the square of a float.
	 * \param[in]  scalar  The float to be squared.
	 * \return     The square of the float.
	 */
	inline float sqr(float scalar)
	{
		return scalar * scalar;
	}
}

#endif /* HRVO_DEFINITIONS_H_ */
