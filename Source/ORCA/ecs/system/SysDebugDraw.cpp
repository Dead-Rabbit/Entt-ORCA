#include "SysDebugDraw.h"

#include "DrawDebugHelpers.h"
#include "ORCA/ORCAEntry.h"
#include "ORCA/ecs/ECSWorld.h"
#include "ORCA/ecs/component/Components.h"

namespace ecs {
	void SysDebugDraw::OnInit()
	{
		ECSSystem::OnInit();
	}

	void SysDebugDraw::OnStart()
	{
		ECSSystem::OnStart();
	}
	
	void SysDebugDraw::OnDestroy()
	{
		ECSSystem::OnDestroy();
	}

	void SysDebugDraw::OnUpdate(float dt)
	{
		auto renderWorld = AORCAEntry::get();
		assert(renderWorld);
		
		auto reg = _world->GetReg();
		if (reg == nullptr)
			return;
		
		static int32 GShowORCATeam = 1;
		static FAutoConsoleVariableRef CVarGShowORCA(
			TEXT("slg.GShowORCATeam"),
			GShowORCATeam,
			TEXT("When set to 1, will display ORCA team Debug Infos."),
			ECVF_Default
		);

		if (GShowORCATeam)
		{
			auto unitView = reg->view<ComTransform, ComNavigationAgent>();
			for (const auto unitEntity : unitView)
			{
				const auto pComNav = reg->getptr<ComNavigationAgent>(unitEntity);
				const auto pComTrans = reg->getptr<ComTransform>(unitEntity);
				const auto pos = pComTrans->location;
				DrawDebugLine(renderWorld->GetWorld(), pos, pos + pComTrans->rotation * pComNav->radius, FColor::Red);
				DrawDebugCircle(renderWorld->GetWorld(), pos, pComNav->radius, 30, FColor::Red,
					false, -1, 0, 6.0f, FVector::ForwardVector, FVector::RightVector, false);
				
				if (pComNav->debugShow) {
					// 绘制寻找 nei 的radius
					for (uint32_t otherEntity : pComNav->neis) {
						const auto pComOtherNavi = reg->getptr<ComNavigationAgent>(otherEntity);
						const auto pComOtherTrans = reg->getptr<ComTransform>(otherEntity);
						DrawDebugCircle(renderWorld->GetWorld(), pComOtherTrans->location + FVector(0, 0, 20), pComOtherNavi->radius, 40, FColor::Blue,
							false, -1, 0, 5.0f, FVector::ForwardVector, FVector::RightVector, false);

						// 绘制朝向
						
					}

					for (auto neiObs : pComNav->obsNeis) {
						ECSObstacle * beginObs = neiObs.second;
						ECSObstacle * nextObsPoint = neiObs.second->nextObstacle_;
						DrawDebugLine(renderWorld->GetWorld(), beginObs->point_ + FVector(0, 0, 20),
							nextObsPoint->point_ + FVector(0, 0, 20), FColor::Green);
					}
				}
			}

			auto obsView = reg->view<ComTransform, ComNavigationObstacle>();
			for(const auto obsEntity : obsView)
			{
				const auto comObs = obsView.get<ComNavigationObstacle>(obsEntity);
				const auto points = comObs.points;
				for (int i = 0 ;i < points.size(); i++) {
					auto point = points[i];
					auto nextPoint = points[i + 1 >= points.size() ? 0 : i + 1];
					if (i == 0) {
						DrawDebugSphere(renderWorld->GetWorld(), point, 10, 20, FColor::Red,
							false, -1, 0, 1.0f);
					}
					else if (i == 1) {
						DrawDebugSphere(renderWorld->GetWorld(), point, 10, 20, FColor::Yellow,
							false, -1, 0, 1.0f);
					}
					
					DrawDebugLine(renderWorld->GetWorld(), point, nextPoint, FColor::Blue);
				}
			}
		}
	}
}
