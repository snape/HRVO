/*
 * Agent.h
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
 * \file   Agent.h
 * \brief  Declares the Agent class.
 */

#ifndef HRVO_AGENT_H_
#define HRVO_AGENT_H_

#include <cstddef>
#include <map>
#include <set>
#include <utility>
#include <vector>

#include "Simulator.h"
#include "Vector2.h"

namespace hrvo {
	/**
	 * \class  Agent
	 * \brief  An agent in the simulation.
	 */
	class Agent {
	private:
		/**
		 * \class  Candidate
		 * \brief  A candidate point.
		 */
		class Candidate {
		public:
			/**
			 * \brief  Constructor.
			 */
			Candidate() : velocityObstacle1_(0), velocityObstacle2_(0) { }

			/**
			 * \brief  The position of the candidate point.
			 */
			Vector2 position_;

			/**
			 * \brief  The number of the first velocity obstacle.
			 */
			int velocityObstacle1_;

			/**
			 * \brief  The number of the second velocity obstacle.
			 */
			int velocityObstacle2_;
		};

		/**
		 * \class  VelocityObstacle
		 * \brief  A hybrid reciprocal velocity obstacle.
		 */
		class VelocityObstacle {
		public:
			/**
			 * \brief  Constructor.
			 */
			VelocityObstacle() { }
			/**
			 * \brief  The position of the apex of the hybrid reciprocal velocity obstacle.
			 */
			Vector2 apex_;

			/**
			 * \brief  The direction of the first side of the hybrid reciprocal velocity obstacle.
			 */
			Vector2 side1_;

			/**
			 * \brief  The direction of the second side of the hybrid reciprocal velocity obstacle.
			 */
			Vector2 side2_;
		};

		/**
		 * \brief      Constructor.
		 * \param[in]  simulator  The simulation.
		 */
		explicit Agent(Simulator *simulator);

		/**
		 * \brief      Constructor.
		 * \param[in]  simulator  The simulation.
		 * \param[in]  position   The starting position of this agent.
		 * \param[in]  goalNo     The goal number of this agent.
		 */
		Agent(Simulator *simulator, const Vector2 &position, std::size_t goalNo);

		/**
		 * \brief      Constructor.
		 * \param[in]  simulator          The simulation.
		 * \param[in]  position           The starting position of this agent.
		 * \param[in]  goalNo             The goal number of this agent.
		 * \param[in]  neighborDist       The maximum neighbor distance of this agent.
		 * \param[in]  maxNeighbors       The maximum neighbor count of this agent.
		 * \param[in]  radius             The radius of this agent.
		 * \param[in]  goalRadius         The goal radius of this agent.
		 * \param[in]  prefSpeed          The preferred speed of this agent.
		 * \param[in]  maxSpeed           The maximum speed of this agent.
		 * \param[in]  uncertaintyOffset  The uncertainty offset of this agent.
		 * \param[in]  maxAccel           The maximum acceleration of this agent.
		 * \param[in]  velocity           The initial velocity of this agent.
		 * \param[in]  orientation        The initial orientation (in radians) of this agent.
		 */
		Agent(Simulator *simulator, const Vector2 &position, std::size_t goalNo, float neighborDist, std::size_t maxNeighbors, float radius, const Vector2 &velocity, float maxAccel, float goalRadius, float prefSpeed, float maxSpeed, float orientation,
#if HRVO_DIFFERENTIAL_DRIVE
			float timeToOrientation, float wheelTrack,
#endif /* HRVO_DIFFERENTIAL_DRIVE */
			float uncertaintyOffset);

		/**
		 * \brief  Computes the neighbors of this agent.
		 */
		void computeNeighbors();

		/**
		 * \brief  Computes the new velocity of this agent.
		 */
		void computeNewVelocity();

		/**
		 * \brief  Computes the preferred velocity of this agent.
		 */
		void computePreferredVelocity();

#if HRVO_DIFFERENTIAL_DRIVE
		/**
		 * \brief  Computes the wheel speeds of this agent.
		 */
		void computeWheelSpeeds();
#endif /* HRVO_DIFFERENTIAL_DRIVE */

		/**
		 * \brief          Inserts a neighbor into the set of neighbors of this agent.
		 * \param[in]      agentNo  The number of the agent to be inserted.
		 * \param[in,out]  rangeSq  The squared range around this agent.
		 */
		void insertNeighbor(std::size_t agentNo, float &rangeSq);

		/**
		 * \brief  Updates the orientation, position, and velocity of this agent.
		 */
		void update();

		Simulator *const simulator_;
		Vector2 newVelocity_;
		Vector2 position_;
		Vector2 prefVelocity_;
		Vector2 velocity_;
		std::size_t goalNo_;
		std::size_t maxNeighbors_;
		float goalRadius_;
		float maxAccel_;
		float maxSpeed_;
		float neighborDist_;
		float orientation_;
		float prefSpeed_;
		float radius_;
		float uncertaintyOffset_;
#if HRVO_DIFFERENTIAL_DRIVE
		float leftWheelSpeed_;
		float rightWheelSpeed_;
		float timeToOrientation_;
		float wheelTrack_;
#endif /* HRVO_DIFFERENTIAL_DRIVE */
		bool reachedGoal_;
		std::multimap<float, Candidate> candidates_;
		std::set<std::pair<float, std::size_t> > neighbors_;
		std::vector<VelocityObstacle> velocityObstacles_;

		friend class KdTree;
		friend class Simulator;
	};
}

#endif /* HRVO_AGENT_H_ */
