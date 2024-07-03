#pragma once

#include "ECSSystem.h"
#include "ORCA/ecs/util/ECSKDTree.h"

namespace ecs
{
	class SysUnitNavigation : public ECSSystem
	{
	public:
		SysUnitNavigation() { }
		virtual ~SysUnitNavigation() override = default;
		ECSKDTree *kdTree_;

	protected:
		virtual void OnInit() override;
		virtual void OnStart() override;
		virtual void OnUpdate(float dt) override;
		virtual void OnDestroy() override;

	private:
		float globalTime_;
		float timeStep_ = 0.25;
		
		const float M_PI = 3.14159265358979323846f;
	};
}
