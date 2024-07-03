#include "ECSKDTree.h"

#include "ORCAMath.h"
#include "ORCA/ORCAEntry.h"
#include "ORCA/ecs/component/Components.h"

namespace ecs
{
	ECSKDTree::~ECSKDTree() { }

#pragma region "Agents"

	void ECSKDTree::BuildAgentTree(entt::DefaultRegistry * reg)
	{
		if (reg == nullptr)
			return;
		
		auto unitView = reg->view<ComTransform, ComNavigationAgent>();
		// 更新自身存储的 agent 位置信息
		for (auto agent : agents_) {
			agent->agentPos = unitView.get<ComTransform>(agent->agentEntity).location;
		}
		
		if (!agents_.empty()) {
			BuildAgentTreeRecursive(0, agents_.size(), 0);
		}
	}
	
	void ECSKDTree::BuildAgentTreeRecursive(size_t begin, size_t end, size_t node)
	{
		agentTree_[node].begin = begin;
		agentTree_[node].end = end;
		agentTree_[node].minX = agentTree_[node].maxX = agents_[begin]->x();
		agentTree_[node].minY = agentTree_[node].maxY = agents_[begin]->y();

		for (size_t i = begin + 1; i < end; ++i) {
			agentTree_[node].maxX = std::max(agentTree_[node].maxX, agents_[i]->x());
			agentTree_[node].minX = std::min(agentTree_[node].minX, agents_[i]->x());
			agentTree_[node].maxY = std::max(agentTree_[node].maxY, agents_[i]->y());
			agentTree_[node].minY = std::min(agentTree_[node].minY, agents_[i]->y());
		}
	
		if (end - begin > MAX_LEAF_SIZE) {
			/* No leaf node. */
			const bool isVertical = (agentTree_[node].maxX - agentTree_[node].minX > agentTree_[node].maxY - agentTree_[node].minY);
			const float splitValue = (isVertical ? 0.5f * (agentTree_[node].maxX + agentTree_[node].minX) : 0.5f * (agentTree_[node].maxY + agentTree_[node].minY));
	
			size_t left = begin;
			size_t right = end;
	
			while (left < right) {
				while (left < right && (isVertical ? agents_[left]->x() : agents_[left]->y()) < splitValue) {
					++left;
				}
	
				while (right > left && (isVertical ? agents_[right - 1]->x() : agents_[right - 1]->y()) >= splitValue) {
					--right;
				}
	
				if (left < right) {
					std::swap(agents_[left], agents_[right - 1]);
					++left;
					--right;
				}
			}
	
			if (left == begin) {
				++left;
				++right;
			}
	
			agentTree_[node].left = node + 1;
			agentTree_[node].right = node + 2 * (left - begin);
	
			BuildAgentTreeRecursive(begin, left, agentTree_[node].left);
			BuildAgentTreeRecursive(left, end, agentTree_[node].right);
		}
	}
	
	void ECSKDTree::ComputeAgentNeighbors(const FVector pos, float &rangeSq, std::vector<uint32_t>* neighbours) const
	{
		QueryAgentTreeRecursive(pos, rangeSq, 0, neighbours);
	}
	
	void ECSKDTree::QueryAgentTreeRecursive(const FVector pos, float &rangeSq, size_t node, std::vector<uint32_t>* neighbours) const
	{
		if (agentTree_[node].end - agentTree_[node].begin <= MAX_LEAF_SIZE) {
			for (size_t i = agentTree_[node].begin; i < agentTree_[node].end; ++i) {
				auto dis = FVector::DistSquaredXY(pos, agents_[i]->agentPos);
				if (dis < rangeSq)
				{
					neighbours->push_back(agents_[i]->agentEntity);
				}
			}
		}
		else {
			const float distSqLeft = ORCAMath::Sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minX - pos.X)) + ORCAMath::Sqr(std::max(0.0f, pos.X - agentTree_[agentTree_[node].left].maxX))
			+ ORCAMath::Sqr(std::max(0.0f, agentTree_[agentTree_[node].left].minY - pos.Y)) + ORCAMath::Sqr(std::max(0.0f, pos.Y - agentTree_[agentTree_[node].left].maxY));
	
			const float distSqRight = ORCAMath::Sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minX - pos.X)) + ORCAMath::Sqr(std::max(0.0f, pos.X - agentTree_[agentTree_[node].right].maxX))
			+ ORCAMath::Sqr(std::max(0.0f, agentTree_[agentTree_[node].right].minY - pos.Y)) + ORCAMath::Sqr(std::max(0.0f, pos.Y - agentTree_[agentTree_[node].right].maxY));
	
			if (distSqLeft < distSqRight) {
				if (distSqLeft < rangeSq) {
					QueryAgentTreeRecursive(pos, rangeSq, agentTree_[node].left, neighbours);
	
					if (distSqRight < rangeSq) {
						QueryAgentTreeRecursive(pos, rangeSq, agentTree_[node].right, neighbours);
					}
				}
			}
			else {
				if (distSqRight < rangeSq) {
					QueryAgentTreeRecursive(pos, rangeSq, agentTree_[node].right, neighbours);
	
					if (distSqLeft < rangeSq) {
						QueryAgentTreeRecursive(pos, rangeSq, agentTree_[node].left, neighbours);
					}
				}
			}
		}
	}
	
#pragma endregion "Agents"

#pragma region "Obstacles"

	size_t ECSKDTree::AddObstacle(const std::vector<FVector> &vertices)
	{
		if (vertices.size() < 2) {
			return ORCAMath::RVO_ERROR;
		}

		const size_t obstacleNo = obstacles_.size();

		for (size_t i = 0; i < vertices.size(); ++i) {
			ECSObstacle *obstacle = new ECSObstacle();
			obstacle->point_ = vertices[i];

			if (i != 0) {
				obstacle->prevObstacle_ = obstacles_.back();
				obstacle->prevObstacle_->nextObstacle_ = obstacle;
			}

			if (i == vertices.size() - 1) {
				obstacle->nextObstacle_ = obstacles_[obstacleNo];
				obstacle->nextObstacle_->prevObstacle_ = obstacle;
			}

			obstacle->unitDir_ = ORCAMath::Normalize(vertices[(i == vertices.size() - 1 ? 0 : i + 1)] - vertices[i]);
			if (vertices.size() == 2) {
				obstacle->isConvex_ = true;
			}
			else {
				obstacle->isConvex_ = ORCAMath::LeftOf(vertices[(i == 0 ? vertices.size() - 1 : i - 1)], vertices[i], vertices[(i == vertices.size() - 1 ? 0 : i + 1)]) >= 0.0f;
			}

			obstacle->id_ = obstacles_.size();

			obstacles_.push_back(obstacle);
		}

		return obstacleNo;
	}

	void ECSKDTree::BuildObstacleTree()
	{
		DeleteObstacleTree(obstacleTree_);
		obstacleTree_ = BuildObstacleTreeRecursive(obstacles_);
	}

	ECSKDTree::ObstacleTreeNode * ECSKDTree::BuildObstacleTreeRecursive(const std::vector<ECSObstacle *> & obstacles)
	{
		if (obstacles.empty()) {
			return nullptr;
		}
		
		ObstacleTreeNode *const node = new ObstacleTreeNode;

		size_t optimalSplit = 0;
		size_t minLeft = obstacles.size();
		size_t minRight = obstacles.size();

		for (size_t i = 0; i < obstacles.size(); ++i) {
			size_t leftSize = 0;
			size_t rightSize = 0;

			const ECSObstacle *const obstacleI1 = obstacles[i];
			const ECSObstacle *const obstacleI2 = obstacleI1->nextObstacle_;

			/* Compute optimal split node. */
			for (size_t j = 0; j < obstacles.size(); ++j) {
				if (i == j) {
					continue;
				}

				const ECSObstacle *const obstacleJ1 = obstacles[j];
				const ECSObstacle *const obstacleJ2 = obstacleJ1->nextObstacle_;

				const float j1LeftOfI = ORCAMath::LeftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ1->point_);
				const float j2LeftOfI = ORCAMath::LeftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ2->point_);

				if (j1LeftOfI >= -ORCAMath::RVO_EPSILON && j2LeftOfI >= -ORCAMath::RVO_EPSILON) {
					++leftSize;
				}
				else if (j1LeftOfI <= ORCAMath::RVO_EPSILON && j2LeftOfI <= ORCAMath::RVO_EPSILON) {
					++rightSize;
				}
				else {
					++leftSize;
					++rightSize;
				}

				if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) >= std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) {
					break;
				}
			}

			if (std::make_pair(std::max(leftSize, rightSize), std::min(leftSize, rightSize)) < std::make_pair(std::max(minLeft, minRight), std::min(minLeft, minRight))) {
				minLeft = leftSize;
				minRight = rightSize;
				optimalSplit = i;
			}
		}

		/* Build split node. */
		std::vector<ECSObstacle *> leftObstacles(minLeft);
		std::vector<ECSObstacle *> rightObstacles(minRight);

		size_t leftCounter = 0;
		size_t rightCounter = 0;
		const size_t i = optimalSplit;

		ECSObstacle *const obstacleI1 = obstacles[i];
		ECSObstacle *const obstacleI2 = obstacleI1->nextObstacle_;

		for (size_t j = 0; j < obstacles.size(); ++j) {
			if (i == j) {
				continue;
			}

			ECSObstacle *const obstacleJ1 = obstacles[j];
			ECSObstacle *const obstacleJ2 = obstacleJ1->nextObstacle_;

			const float j1LeftOfI = ORCAMath::LeftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ1->point_);
			const float j2LeftOfI = ORCAMath::LeftOf(obstacleI1->point_, obstacleI2->point_, obstacleJ2->point_);

			if (j1LeftOfI >= -ORCAMath::RVO_EPSILON && j2LeftOfI >= -ORCAMath::RVO_EPSILON) {
				leftObstacles[leftCounter++] = obstacles[j];
			}
			else if (j1LeftOfI <= ORCAMath::RVO_EPSILON && j2LeftOfI <= ORCAMath::RVO_EPSILON) {
				rightObstacles[rightCounter++] = obstacles[j];
			}
			else {
				/* Split obstacle j. */
				const float t = ORCAMath::Det2D(obstacleI2->point_ - obstacleI1->point_, obstacleJ1->point_ - obstacleI1->point_) / ORCAMath::Det2D(obstacleI2->point_ - obstacleI1->point_, obstacleJ1->point_ - obstacleJ2->point_);

				const FVector splitpoint = obstacleJ1->point_ + t * (obstacleJ2->point_ - obstacleJ1->point_);

				ECSObstacle *const newObstacle = new ECSObstacle();
				newObstacle->point_ = splitpoint;
				newObstacle->prevObstacle_ = obstacleJ1;
				newObstacle->nextObstacle_ = obstacleJ2;
				newObstacle->isConvex_ = true;
				newObstacle->unitDir_ = obstacleJ1->unitDir_;
				newObstacle->id_ = obstacles_.size();

				obstacles_.push_back(newObstacle);

				obstacleJ1->nextObstacle_ = newObstacle;
				obstacleJ2->prevObstacle_ = newObstacle;

				if (j1LeftOfI > 0.0f) {
					leftObstacles[leftCounter++] = obstacleJ1;
					rightObstacles[rightCounter++] = newObstacle;
				}
				else {
					rightObstacles[rightCounter++] = obstacleJ1;
					leftObstacles[leftCounter++] = newObstacle;
				}
			}
		}

		node->obstacle = obstacleI1;
		node->left = BuildObstacleTreeRecursive(leftObstacles);
		node->right = BuildObstacleTreeRecursive(rightObstacles);
		return node;
	}

	void ECSKDTree::DeleteObstacleTree(ObstacleTreeNode *node)
	{
		if (node != nullptr) {
			DeleteObstacleTree(node->left);
			DeleteObstacleTree(node->right);
			delete node;
		}
	}

	void ECSKDTree::ComputeObstacleNeighbors(const FVector & pos, float &rangeSq, std::vector<std::pair<float, ECSObstacle *>> & neighbours) const
	{
		QueryObstacleTreeRecursive(pos, rangeSq, obstacleTree_, neighbours);
	}

	void ECSKDTree::QueryObstacleTreeRecursive(const FVector & pos, float rangeSq, const ObstacleTreeNode *node, std::vector<std::pair<float, ECSObstacle *>> & neighbours) const
	{
		if (node == nullptr) {
			return;
		}

		const ECSObstacle *const obstacle1 = node->obstacle;
		const ECSObstacle *const obstacle2 = obstacle1->nextObstacle_;

		const float agentLeftOfLine = ORCAMath::LeftOf(obstacle1->point_, obstacle2->point_, pos);

		QueryObstacleTreeRecursive(pos, rangeSq, (agentLeftOfLine >= 0.0f ? node->left : node->right), neighbours);

		const float distSqLine = ORCAMath::Sqr(agentLeftOfLine) / ORCAMath::AbsSq2D(obstacle2->point_ - obstacle1->point_);

		if (distSqLine < rangeSq) {
			if (agentLeftOfLine < 0.0f) {
				/*
				 * Try obstacle at this node only if agent is on right side of
				 * obstacle (and can see obstacle).
				 */
				// auto obs = node->obstacle;
				// const auto nextObstacle = obs->nextObstacle_;
				// const float distSq = ORCAMath::DistSqPointLineSegment(obs->point_, nextObstacle->point_, pos);
				// if (distSq < rangeSq) {
				// 	neighbours->push_back(obs);
				// }
				auto obstacle = node->obstacle;
				const ECSObstacle *const nextObstacle = obstacle->nextObstacle_;

				const float distSq = ORCAMath::DistSqPointLineSegment(obstacle->point_, nextObstacle->point_, pos);

				if (distSq < rangeSq) {
					neighbours.push_back(std::make_pair(distSq, obstacle));

					size_t i = neighbours.size() - 1;

					while (i != 0 && distSq < neighbours[i - 1].first) {
						neighbours[i] = neighbours[i - 1];
						--i;
					}

					neighbours[i] = std::make_pair(distSq, obstacle);
				}
			}

			/* Try other side of line. */
			QueryObstacleTreeRecursive(pos, rangeSq, (agentLeftOfLine >= 0.0f ? node->right : node->left), neighbours);

		}
	}

#pragma endregion "Obstacles"
	
}
