#pragma once
#include "ECSSystem.h"

namespace ecs
{
	class SysDebugDraw : public ECSSystem
	{
	public:
		SysDebugDraw() { }
		virtual ~SysDebugDraw() override = default;

	protected:
		virtual void OnInit() override;
		virtual void OnStart() override;
		virtual void OnUpdate(float dt) override;
		virtual void OnDestroy() override;
	};
}
