#pragma once
#include "ORCA/ecs/util/ECSObstacle.h"

namespace ecs
{
	struct ComTransform
	{
		FVector location;
		FVector rotation;
	};
	
	struct NavigationLine
	{
		FVector point;
		FVector direction;
	};

	struct ComNavigationAgent
	{
		int id = 0;
		
		int maxNeighbors = 0;
		float maxSpeed = 0;
		float neighborDist = 0;
		float radius = 0;
		float radiusSeq = 0;
		float timeHorizon = 5.0f;
		float timeHorizonObst = 5.0f;

		float mass = 1.0f;

		bool reached = false;
		FVector prefVelocity = FVector::ZeroVector;
		FVector velocity = FVector::ZeroVector;
		FVector newVelocity = FVector::ZeroVector;

		std::vector<uint32_t> neis;
		// std::vector<ECSObstacle*> obsNeis;
		std::vector<std::pair<float, ECSObstacle *>> obsNeis;
		std::vector<NavigationLine> orcaLines_;

		FVector targetPos = FVector::ZeroVector;

		AActor * actor = nullptr;
		bool debugShow = false;
	};

	struct ComNavigationObstacle
	{
		std::vector<FVector> points;
	};
}
