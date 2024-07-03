#pragma once
#include <vector>

#include "ORCA/normalOrca/RVOSimulator.h"
#include "ORCA/normalOrca/Vector2.h"

#include <vector>

namespace RVO
{
	class OriginBlockSimulate
	{
	public:
		const float M_PI = 3.14159265358979323846f;

		void Start(AActor * entryPoint);
		void Update();

	private:
		std::vector<Vector2> goals;
		std::vector<bool> reached;
		RVOSimulator* sim = nullptr;
		
		void setupScenario();
		void setPreferredVelocities();
		void reachedGoal();
		void UpdateVisualization();
	};
}
