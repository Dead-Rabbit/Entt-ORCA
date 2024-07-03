#pragma once
#include <memory>

namespace ecs
{
	class ECSWorld;
	class ECSSystem : public std::enable_shared_from_this<ECSSystem>
	{
	public:
		virtual ~ECSSystem() = default;
		using WorldPtr = TSharedPtr<ECSWorld>;
		using Ptr = TSharedPtr<ECSSystem>;
		
		void init(WorldPtr world, float itvl)
		{
			_world = world;
			OnInit();
		}
		void Start() { OnStart(); }
		void Update(float dt);
		void Destroy() 
		{ 
			OnDestroy();
			_world.Reset(); 
		}

	protected:
		virtual void OnInit() {}
		virtual void OnStart() {}
		virtual void OnUpdate(float dt) {};
		virtual void OnDestroy() {}
		
		WorldPtr _world;
	};
}
