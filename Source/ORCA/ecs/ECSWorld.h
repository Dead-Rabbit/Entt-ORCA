#pragma once

#include "ORCA/Framework/Plugins/entt/entity/registry.hpp"
#include "system/ECSSystem.h"

namespace ecs
{
	class ECSWorld : public TSharedFromThis<ECSWorld>
	{
	public:
		ECSWorld()
		{
			reg = new entt::DefaultRegistry();
		}
		using Ptr = TSharedPtr<ECSWorld>;
		
		template<typename T>
		TSharedPtr<T> createAndRegisterSystem(float itvl = 0)
		{
			auto s = MakeShared<T>();
			s->init(AsShared(), itvl);
			_systemList.Push(s);
			return s;
		}

		void Start();
		void Update(float dt);
		entt::DefaultRegistry * GetReg(){ return reg;}

	private:
		entt::DefaultRegistry * reg;
		TArray<ECSSystem::Ptr> _systemList;
		void UpdateSystems(float dt);
	};
}
