/*
 * KdTree.cpp
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
 * \file   KdTree.cpp
 * \brief  Defines the KdTree class.
 */

#include "KdTree.h"

#include <algorithm>
#include <limits>

#include "Agent.h"
#include "Definitions.h"
#include "Simulator.h"

namespace hrvo {
	KdTree::KdTree(Simulator *simulator) : simulator_(simulator) { }

	void KdTree::build()
	{
		agents_.reserve(simulator_->agents_.size());

		for (std::size_t i = agents_.size(); i < simulator_->agents_.size(); ++i) {
			agents_.push_back(i);
		}

		nodes_.resize(2 * simulator_->agents_.size() - 1);

		if (!agents_.empty()) {
			buildRecursive(0, agents_.size(), 0);
		}
	}

	void KdTree::buildRecursive(std::size_t begin, std::size_t end, std::size_t node)
	{
		nodes_[node].begin_ = begin;
		nodes_[node].end_ = end;
		nodes_[node].minX_ = nodes_[node].maxX_ = simulator_->agents_[agents_[begin]]->position_.getX();
		nodes_[node].minY_ = nodes_[node].maxY_ = simulator_->agents_[agents_[begin]]->position_.getY();

		for (std::size_t i = begin + 1; i < end; ++i) {
			if (simulator_->agents_[agents_[i]]->position_.getX() > nodes_[node].maxX_) {
				nodes_[node].maxX_ = simulator_->agents_[agents_[i]]->position_.getX();
			}
			else if (simulator_->agents_[agents_[i]]->position_.getX() < nodes_[node].minX_) {
				nodes_[node].minX_ = simulator_->agents_[agents_[i]]->position_.getX();
			}

			if (simulator_->agents_[agents_[i]]->position_.getY() > nodes_[node].maxY_) {
				nodes_[node].maxY_ = simulator_->agents_[agents_[i]]->position_.getY();
			}
			else if (simulator_->agents_[agents_[i]]->position_.getY() < nodes_[node].minY_) {
				nodes_[node].minY_ = simulator_->agents_[agents_[i]]->position_.getY();
			}
		}

		if (end - begin > HRVO_MAX_LEAF_SIZE) {
			const bool vertical = nodes_[node].maxX_ - nodes_[node].minX_ > nodes_[node].maxY_ - nodes_[node].minY_;
			const float split = 0.5f * (vertical ?  nodes_[node].maxX_ + nodes_[node].minX_ : nodes_[node].maxY_ + nodes_[node].minY_);

			std::size_t left = begin;
			std::size_t right = end - 1;

			while (true) {
				while (left <= right && (vertical ? simulator_->agents_[agents_[left]]->position_.getX()
										 : simulator_->agents_[agents_[left]]->position_.getY()) < split) {
					++left;
				}

				while (right >= left && (vertical ? simulator_->agents_[agents_[right]]->position_.getX()
										 : simulator_->agents_[agents_[right]]->position_.getY()) >= split) {
					--right;
				}

				if (left > right) {
					break;
				}
				else {
					std::swap(agents_[left], agents_[right]);
					++left;
					--right;
				}
			}

			if (left == begin) {
				++left;
				++right;
			}

			nodes_[node].left_ = node + 1;
			nodes_[node].right_ = 2 * (left - begin) + node;

			buildRecursive(begin, left, nodes_[node].left_);
			buildRecursive(left, end, nodes_[node].right_);
		}
	}

	void KdTree::queryRecursive(Agent *agent, float &rangeSq, std::size_t node) const
	{
		if (nodes_[node].end_ - nodes_[node].begin_ <= HRVO_MAX_LEAF_SIZE) {
			for (std::size_t i = nodes_[node].begin_; i < nodes_[node].end_; ++i) {
				agent->insertNeighbor(agents_[i], rangeSq);
			}
		}
		else {
			float distSqLeft = 0.0f;
			float distSqRight = 0.0f;

			if (agent->position_.getX() < nodes_[nodes_[node].left_].minX_) {
				distSqLeft += sqr(nodes_[nodes_[node].left_].minX_ - agent->position_.getX());
			}
			else if (agent->position_.getX() > nodes_[nodes_[node].left_].maxX_) {
				distSqLeft += sqr(agent->position_.getX() - nodes_[nodes_[node].left_].maxX_);
			}

			if (agent->position_.getY() < nodes_[nodes_[node].left_].minY_) {
				distSqLeft += sqr(nodes_[nodes_[node].left_].minY_ - agent->position_.getY());
			}
			else if (agent->position_.getY() > nodes_[nodes_[node].left_].maxY_) {
				distSqLeft += sqr(agent->position_.getY() - nodes_[nodes_[node].left_].maxY_);
			}

			if (agent->position_.getX() < nodes_[nodes_[node].right_].minX_) {
				distSqRight += sqr(nodes_[nodes_[node].right_].minX_ - agent->position_.getX());
			}
			else if (agent->position_.getX() > nodes_[nodes_[node].right_].maxX_) {
				distSqRight += sqr(agent->position_.getX() - nodes_[nodes_[node].right_].maxX_);
			}

			if (agent->position_.getY() < nodes_[nodes_[node].right_].minY_) {
				distSqRight += sqr(nodes_[nodes_[node].right_].minY_ - agent->position_.getY());
			}
			else if (agent->position_.getY() > nodes_[nodes_[node].right_].maxY_) {
				distSqRight += sqr(agent->position_.getY() - nodes_[nodes_[node].right_].maxY_);
			}

			if (distSqLeft < distSqRight) {
				if (distSqLeft < rangeSq) {
					queryRecursive(agent, rangeSq, nodes_[node].left_);

					if (distSqRight < rangeSq) {
						queryRecursive(agent, rangeSq, nodes_[node].right_);
					}
				}
			}
			else {
				if (distSqRight < rangeSq) {
					queryRecursive(agent, rangeSq, nodes_[node].right_);

					if (distSqLeft < rangeSq) {
						queryRecursive(agent, rangeSq, nodes_[node].left_);
					}
				}
			}
		}
	}
}
