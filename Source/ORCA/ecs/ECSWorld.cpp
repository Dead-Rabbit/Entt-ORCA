#include "ECSWorld.h"

namespace ecs
{
	void ECSWorld::Start()
	{
		for (const auto& s : _systemList)
			s->Start();
	}

	void ECSWorld::Update(float dt)
	{
		UpdateSystems(dt);
	}
	
	void ECSWorld::UpdateSystems(float dt)
	{
		for (const auto& s : _systemList)
			s->Update(dt);
	}
}
