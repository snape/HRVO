/*
 * HRVO.h
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
 * \file   HRVO.h
 * \brief  Includes all public headers.
 */

#ifndef HRVO_HRVO_H_
#define HRVO_HRVO_H_

/**
 * \namespace  hrvo
 * \brief      Contains all classes and functions.
 */

/**
 * \mainpage   HRVO Library Documentation
 * \authors    Jamie Snape, Jur van den Berg, Stephen J. Guy, and Dinesh Manocha
 * \copyright  2009 University of North Carolina at Chapel Hill
 *
 * \details    We present the hybrid reciprocal velocity obstacle for
 *             collision-free and oscillation-free navigation of multiple mobile
 *             robots or virtual agents. Each robot or virtual agents senses its
 *             surroundings and acts independently without central coordination
 *             or communication with other robots or virtual agents. Our
 *             approach uses both the current position and the velocity of other
 *             robots or virtual agents to compute their future trajectories in
 *             order to avoid collisions. Moreover, our approach is reciprocal
 *             and avoids oscillations by explicitly taking into account that
 *             the other robots or virtual agents also sense their surroundings
 *             and change their trajectories accordingly.
 */

#include "Export.h"
#include "Simulator.h"
#include "Vector2.h"

#endif
