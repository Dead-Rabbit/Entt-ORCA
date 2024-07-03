#include "ECSBlockSimulate.h"

#include "ORCA/ORCAEntry.h"
#include "ORCA/ecs/component/Components.h"

namespace ecs
{
	ECSBlockSimulate::ECSBlockSimulate(AActor * entryPoint)
	{
		this->entryPoint_ = entryPoint;
	}
	
	void ECSBlockSimulate::SetupScenario()
	{
		if (!entryPoint_)
			return;

		const auto reg = Cast<AORCAEntry>(entryPoint_)->GetReg();
		if (!reg)
			return;
		
		ecsKDTree_ = new ECSKDTree();

		for (size_t i = 0; i < 100; ++i)
		{
			for (size_t j = 0; j < 10; ++j)
			{
				const uint32_t agentEntityID = reg->create();
				AddAgent(agentEntityID, FVector(2000 + i * 140.0f, 2000 + j * 140.0f, 0),
					FVector(0, 0, 0), 1.0f,
					200.0f, 10, 5.0f,
					5.0f, 50.0f, 50.0f, FVector(0, 0, 0), i == 0 && j == 0);
			}
		}

		// const uint32_t heavyEntity = reg->create();
		// AddAgent(heavyEntity, FVector(1000, 1000, 0),
		// 	FVector(1000, 1000, 0), 2.0f,
		// 	200.0f, 10, 10.0f,
		// 	5.0f, 300.0f, 16.0f, FVector(0, 0, 0));

		// 添加 agent 后，初始化 kdtree 内容
		ecsKDTree_->InitECSKDTree();

		std::vector<FVector> obstacle;
		obstacle.push_back(FVector(800.0f, 1200.0f, 0.0f));
		obstacle.push_back(FVector(800.0f, 800.0f, 0.0f));
		obstacle.push_back(FVector(1200.0f, 800.0f,  0.0f));
		obstacle.push_back(FVector(1200.0f, 1200.0f,  0.0f));
		const uint32_t obstacleEntity = reg->create();
		AddObstacle(obstacleEntity, obstacle);
		
		ecsKDTree_->BuildObstacleTree();
	}
	
	void ECSBlockSimulate::AddAgent(const uint32_t entityID, const FVector& position, const FVector& targetPos, float mass, float neighborDist,
		size_t maxNeighbors, float timeHorizon, float timeHorizonObst, float radius, float maxSpeed, const FVector& velocity, bool showNei)
	{
		if (!entryPoint_)
			return;

		const auto reg = Cast<AORCAEntry>(entryPoint_)->GetReg();
		if (!reg)
			return;
		
		ComTransform & comTrans = reg->assign<ComTransform>(entityID);
		comTrans.location = position;
		
		ComNavigationAgent & comAgent = reg->assign<ComNavigationAgent>(entityID);
		comAgent.neighborDist = neighborDist;
		comAgent.maxNeighbors = maxNeighbors;
		comAgent.timeHorizon = timeHorizon;
		comAgent.timeHorizonObst = timeHorizonObst;
		comAgent.radius = radius;
		// TODO radiusSeq 考虑改成当前最大的 radius * radius * 4 
		comAgent.radiusSeq = 400000.0f;
		comAgent.maxSpeed = maxSpeed;
		comAgent.velocity = velocity;
		comAgent.targetPos = targetPos;
		comAgent.mass = mass;
		comAgent.debugShow = showNei;
		
		// auto newAct = Cast<AORCAEntry>(entryPoint_)->CreateActor("Meshes/Charactors/BP_SummerGirl");
		// newAct->SetActorLocation(FVector(position.X, position.Y, 0));
		// comAgent.actor = newAct;

		// 追加 ecsEntity
		ecsKDTree_->AddAgent(entityID);
	}

	void ECSBlockSimulate::AddObstacle(const uint32_t entityID, std::vector<FVector> points)
	{
		if (!entryPoint_)
			return;

		const auto reg = Cast<AORCAEntry>(entryPoint_)->GetReg();
		if (!reg)
			return;

		if (points.size() <= 1)
			return;
		
		ComTransform & comTrans = reg->assign<ComTransform>(entityID);
		comTrans.location = points[0];
		
		ComNavigationObstacle & comObstacle = reg->assign<ComNavigationObstacle>(entityID);
		comObstacle.points = points;
		
		ecsKDTree_->AddObstacle(points);
	}

}
