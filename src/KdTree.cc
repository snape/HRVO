/*
 * KdTree.cc
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
 * @file  KdTree.cc
 * @brief Defines the KdTree class.
 */

#include "KdTree.h"

#include <algorithm>
#include <utility>

#include "Agent.h"
#include "Simulator.h"
#include "Vector2.h"

namespace hrvo {
namespace {
const std::size_t HRVO_MAX_LEAF_SIZE = 10U;
} /* namespace */

class KdTree::Node {
 public:
  Node();

  std::size_t begin_;
  std::size_t end_;
  std::size_t left_;
  std::size_t right_;
  float maxX_;
  float maxY_;
  float minX_;
  float minY_;
};

KdTree::Node::Node()
    : begin_(0U),
      end_(0U),
      left_(0U),
      right_(0U),
      maxX_(0.0F),
      maxY_(0.0F),
      minX_(0.0F),
      minY_(0.0F) {}

KdTree::KdTree(Simulator *simulator) : simulator_(simulator) {}

KdTree::~KdTree() {}

void KdTree::build() {
  agents_.reserve(simulator_->agents_.size());

  for (std::size_t i = agents_.size(); i < simulator_->agents_.size(); ++i) {
    agents_.push_back(i);
  }

  nodes_.resize(2U * simulator_->agents_.size() - 1U);

  if (!agents_.empty()) {
    buildRecursive(0U, agents_.size(), 0U);
  }
}

void KdTree::buildRecursive(std::size_t begin, std::size_t end,
                            std::size_t node) {
  nodes_[node].begin_ = begin;
  nodes_[node].end_ = end;
  nodes_[node].minX_ = nodes_[node].maxX_ =
      simulator_->agents_[agents_[begin]]->position_.getX();
  nodes_[node].minY_ = nodes_[node].maxY_ =
      simulator_->agents_[agents_[begin]]->position_.getY();

  for (std::size_t i = begin + 1U; i < end; ++i) {
    if (simulator_->agents_[agents_[i]]->position_.getX() >
        nodes_[node].maxX_) {
      nodes_[node].maxX_ = simulator_->agents_[agents_[i]]->position_.getX();
    } else if (simulator_->agents_[agents_[i]]->position_.getX() <
               nodes_[node].minX_) {
      nodes_[node].minX_ = simulator_->agents_[agents_[i]]->position_.getX();
    }

    if (simulator_->agents_[agents_[i]]->position_.getY() >
        nodes_[node].maxY_) {
      nodes_[node].maxY_ = simulator_->agents_[agents_[i]]->position_.getY();
    } else if (simulator_->agents_[agents_[i]]->position_.getY() <
               nodes_[node].minY_) {
      nodes_[node].minY_ = simulator_->agents_[agents_[i]]->position_.getY();
    }
  }

  if (end - begin > HRVO_MAX_LEAF_SIZE) {
    const bool vertical = nodes_[node].maxX_ - nodes_[node].minX_ >
                          nodes_[node].maxY_ - nodes_[node].minY_;
    const float split =
        0.5F * (vertical ? nodes_[node].maxX_ + nodes_[node].minX_
                         : nodes_[node].maxY_ + nodes_[node].minY_);

    std::size_t left = begin;
    std::size_t right = end - 1U;

    while (true) {
      while (left <= right &&
             (vertical ? simulator_->agents_[agents_[left]]->position_.getX()
                       : simulator_->agents_[agents_[left]]->position_.getY()) <
                 split) {
        ++left;
      }

      while (right >= left &&
             (vertical
                  ? simulator_->agents_[agents_[right]]->position_.getX()
                  : simulator_->agents_[agents_[right]]->position_.getY()) >=
                 split) {
        --right;
      }

      if (left > right) {
        break;
      }
      std::swap(agents_[left], agents_[right]);
      ++left;
      --right;
    }

    if (left == begin) {
      ++left;
      ++right;
    }

    nodes_[node].left_ = node + 1U;
    nodes_[node].right_ = 2U * (left - begin) + node;

    buildRecursive(begin, left, nodes_[node].left_);
    buildRecursive(left, end, nodes_[node].right_);
  }
}

void KdTree::query(Agent *agent, float rangeSq) const {
  queryRecursive(agent, rangeSq, 0U);
}

void KdTree::queryRecursive(Agent *agent, float &rangeSq,
                            std::size_t node) const {
  if (nodes_[node].end_ - nodes_[node].begin_ <= HRVO_MAX_LEAF_SIZE) {
    for (std::size_t i = nodes_[node].begin_; i < nodes_[node].end_; ++i) {
      agent->insertNeighbor(agents_[i], rangeSq);
    }
  } else {
    const float distLeftMinX = std::max(
        0.0F, nodes_[nodes_[node].left_].minX_ - agent->position_.getX());
    const float distLeftMaxX = std::max(
        0.0F, agent->position_.getX() - nodes_[nodes_[node].left_].maxX_);
    const float distLeftMinY = std::max(
        0.0F, nodes_[nodes_[node].left_].minY_ - agent->position_.getY());
    const float distLeftMaxY = std::max(
        0.0F, agent->position_.getY() - nodes_[nodes_[node].left_].maxY_);

    const float distSqLeft =
        distLeftMinX * distLeftMinX + distLeftMaxX * distLeftMaxX +
        distLeftMinY * distLeftMinY + distLeftMaxY * distLeftMaxY;

    const float distRightMinX = std::max(
        0.0F, nodes_[nodes_[node].right_].minX_ - agent->position_.getX());
    const float distRightMaxX = std::max(
        0.0F, agent->position_.getX() - nodes_[nodes_[node].right_].maxX_);
    const float distRightMinY = std::max(
        0.0F, nodes_[nodes_[node].right_].minY_ - agent->position_.getY());
    const float distRightMaxY = std::max(
        0.0F, agent->position_.getY() - nodes_[nodes_[node].right_].maxY_);

    const float distSqRight =
        distRightMinX * distRightMinX + distRightMaxX * distRightMaxX +
        distRightMinY * distRightMinY + distRightMaxY * distRightMaxY;

    if (distSqLeft < distSqRight) {
      if (distSqLeft < rangeSq) {
        queryRecursive(agent, rangeSq, nodes_[node].left_);

        if (distSqRight < rangeSq) {
          queryRecursive(agent, rangeSq, nodes_[node].right_);
        }
      }
    } else {
      if (distSqRight < rangeSq) {
        queryRecursive(agent, rangeSq, nodes_[node].right_);

        if (distSqLeft < rangeSq) {
          queryRecursive(agent, rangeSq, nodes_[node].left_);
        }
      }
    }
  }
}
} /* namespace hrvo */
