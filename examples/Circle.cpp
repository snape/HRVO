/*
 * Circle.cpp
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
 * \file   Circle.cpp
 * \brief  Example with 250 agents navigating through a circular environment.
 */

#ifndef HRVO_OUTPUT_TIME_AND_POSITIONS
#define HRVO_OUTPUT_TIME_AND_POSITIONS 1
#endif

#include <cmath>

#if HRVO_OUTPUT_TIME_AND_POSITIONS
#include <iostream>
#endif

#include <HRVO.h>

using namespace hrvo;

const float HRVO_TWO_PI = 6.283185307179586f;

int main()
{
	Simulator simulator;

	simulator.setTimeStep(0.25f);
	simulator.setAgentDefaults(15.0f, 10, 1.5f, 1.5f, 1.0f, 2.0f);

	for (std::size_t i = 0; i < 250; ++i) {
		const Vector2 position = 200.0f * Vector2(std::cos(0.004f * i * HRVO_TWO_PI), std::sin(0.004f * i * HRVO_TWO_PI));
		simulator.addAgent(position, simulator.addGoal(-position));
	}

	do {
#if HRVO_OUTPUT_TIME_AND_POSITIONS
		std::cout << simulator.getGlobalTime();

		for (std::size_t i = 0; i < simulator.getNumAgents(); ++i) {
			std::cout << " " << simulator.getAgentPosition(i);
		}

		std::cout << std::endl;
#endif /* HRVO_OUTPUT_TIME_AND_POSITIONS */

		simulator.doStep();
	}
	while (!simulator.haveReachedGoals());

	return 0;
}
