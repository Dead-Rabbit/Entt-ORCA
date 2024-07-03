#pragma once
#include "ORCA/ecs/util/ECSKDTree.h"

namespace ecs
{
	class ECSBlockSimulate
	{
	public:
		ECSKDTree * ecsKDTree_ = nullptr;
		
		ECSBlockSimulate(AActor * entryPoint);
		void SetupScenario();
	private:
		AActor * entryPoint_ = nullptr;
		void AddAgent(const uint32_t entityID, const FVector &position, const FVector& targetPos, const float mass, float neighborDist, size_t maxNeighbors, float timeHorizon,
		float timeHorizonObst, float radius, float maxSpeed, const FVector &velocity, bool showNei = false);
		
		void AddObstacle(const uint32_t entityID, std::vector<FVector> points);
	};
}
