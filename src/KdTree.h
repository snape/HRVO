/*
 * KdTree.h
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
 * \file   KdTree.h
 * \brief  Declares the KdTree class.
 */

#ifndef HRVO_KD_TREE_H_
#define HRVO_KD_TREE_H_

#include <cstddef>
#include <vector>

#include "Vector2.h"

namespace hrvo {
	class Agent;
	class Simulator;

	/**
	 * \class  KdTree
	 * \brief  k-D trees for agents in the simulation.
	 */
	class KdTree {
	private:
		/**
		 * \class  Node
		 * \brief  Defines a k-D tree node.
		 */
		class Node {
		public:
			/**
			 * \brief  Constructor.
			 */
			Node() : begin_(0), end_(0), left_(0), right_(0), maxX_(0.0f), maxY_(0.0f), minX_(0.0f), minY_(0.0f) { }

			/**
			 * \brief  The beginning node number.
			 */
			std::size_t begin_;

			/**
			 * \brief  The ending node number.
			 */
			std::size_t end_;

			/**
			 * \brief  The left node number.
			 */
			std::size_t left_;

			/**
			 * \brief  The right node number.
			 */
			std::size_t right_;

			/**
			 * \brief  The maximum x-coordinate.
			 */
			float maxX_;

			/**
			 * \brief  The maximum y-coordinate.
			 */
			float maxY_;

			/**
			 * \brief  The minimum x-coordinate.
			 */
			float minX_;

			/**
			 * \brief  The minimum y-coordinate.
			 */
			float minY_;
		};

		/**
		 * \brief  The maximum leaf size of a k-D tree.
		 */
		static const std::size_t HRVO_MAX_LEAF_SIZE = 10;

		/**
		 * \brief      Constructor.
		 * \param[in]  simulator  The simulation.
		 */
		explicit KdTree(Simulator *simulator);

		/**
		 * \brief  Builds an agent k-D tree.
		 */
		void build();

		/**
		 * \brief  Recursive function to build a k-D tree.
		 * \param  begin  The beginning k-D tree node.
		 * \param  end    The ending k-D tree node.
		 * \param  node   The current k-D tree node.
		 */
		void buildRecursive(std::size_t begin, std::size_t end, std::size_t node);

		/**
		 * \brief      Computes the neighbors of the specified agent.
		 * \param[in]  agent    A pointer to the agent for which neighbors are to be computed.
		 * \param[in]  rangeSq  The squared range around the agent.
		 */
		void query(Agent *agent, float rangeSq) const
		{
			queryRecursive(agent, rangeSq, 0);
		}

		/**
		 * \brief          Recursive function to compute the neighbors of the specified agent.
		 * \param[in]      agent    A pointer to the agent for which neighbors are to be computed.
		 * \param[in,out]  rangeSq  The squared range around the agent.
		 * \param[in]      node     The current k-D tree node.
		 */
		void queryRecursive(Agent *agent, float &rangeSq, std::size_t node) const;

		Simulator *const simulator_;
		std::vector<std::size_t> agents_;
		std::vector<Node> nodes_;

		friend class Agent;
		friend class Simulator;
	};
}

#endif /* HRVO_KD_TREE_H_ */
