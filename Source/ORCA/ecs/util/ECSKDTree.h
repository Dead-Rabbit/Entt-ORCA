#pragma once
#include <vector>

#include "ECSObstacle.h"
#include "ORCA/Framework/Plugins/entt/entity/registry.hpp"

/*
 * ECSKDTree.h
 * RVO2 Library
 *
 * Copyright 2008 University of North Carolina at Chapel Hill
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
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/RVO2/>
 */

namespace ecs {
	/**
	 *Defines <i>k</i>d-trees for agents and static obstacles in the simulation.
	 */
	class ECSKDTree {
		
	public:

#pragma region "Agents"
		
		class AgentTreeNode {
		public:
			// The beginning node number.
			size_t begin;

			// The ending node number.
			size_t end;

			// The left node number.
			size_t left;

			// The maximum x-coordinate.
			float maxX;

			// The maximum y-coordinate.
			float maxY;

			// The minimum x-coordinate.
			float minX;

			// The minimum y-coordinate.
			float minY;

			// The right node number.
			size_t right;
		};
		
		class ECSKDAgent {
		public:
			ECSKDAgent(uint32_t entity) : agentEntity(entity) {}

			uint32_t agentEntity = 0;
			FVector agentPos = FVector::ZeroVector;

			float x() { return agentPos.X; }
			float y(){ return agentPos.Y; }
		};

		// Add ecs orca agent
		void AddAgent(uint32_t entityId)
		{
			agents_.push_back(new ECSKDAgent(entityId));
		}
		
		/**
		 * \brief      Builds an agent <i>k</i>d-tree.
		 */
		void BuildAgentTree(entt::DefaultRegistry * setReg);

		void BuildAgentTreeRecursive(size_t begin, size_t end, size_t node);

		/**
		 * \brief      Computes the agent neighbors of the specified agent.
		 * \param      agent           A pointer to the agent for which agent
		 *                             neighbors are to be computed.
		 * \param      rangeSq         The squared range around the agent.
		 */
		void ComputeAgentNeighbors(const FVector pos, float &rangeSq, std::vector<uint32_t>* neighbours) const;

		void QueryAgentTreeRecursive(const FVector pos, float &rangeSq,
									 size_t node, std::vector<uint32_t>* neighbours) const;

		void InitECSKDTree()
		{
			agentTree_.resize(2 * agents_.size() - 1);
		}
		
#pragma endregion "Agents"

#pragma region "Obstacles"
		
		// Defines an obstacle <i>k</i>d-tree node.
		class ObstacleTreeNode {
		public:
			// The left obstacle tree node.
			ObstacleTreeNode *left;

			// The obstacle number.
			ECSObstacle *obstacle;

			// The right obstacle tree node.
			ObstacleTreeNode *right;
		};

		// Add orca Obstacle
		// TODO 考虑 add bos 的位置
		size_t AddObstacle(const std::vector<FVector> &vertices);

		// Builds an obstacle <i>k</i>d-tree.
		void BuildObstacleTree();

		ObstacleTreeNode *BuildObstacleTreeRecursive(const std::vector<ECSObstacle *> & obstacles);

		void DeleteObstacleTree(ObstacleTreeNode *node);
		
		void ComputeObstacleNeighbors(const FVector & pos, float &rangeSq, std::vector<std::pair<float, ECSObstacle *>> & neighbours) const;
		
		void QueryObstacleTreeRecursive(const FVector & pos, float rangeSq,
										const ObstacleTreeNode *node, std::vector<std::pair<float, ECSObstacle *>> & neighbours) const;
		

#pragma endregion "Obstacles"
		
		ECSKDTree() {};
		
		// Destroys this kd-tree instance.
		~ECSKDTree();

		std::vector<ECSKDAgent *> agents_;
		std::vector<AgentTreeNode> agentTree_;

		std::vector<ECSObstacle *> obstacles_;
		ObstacleTreeNode * obstacleTree_ = nullptr;
		
		static const size_t MAX_LEAF_SIZE = 10;
		friend class Agent;
		friend class ECSKDAgent;
		friend class RVOSimulator;
	};
}
