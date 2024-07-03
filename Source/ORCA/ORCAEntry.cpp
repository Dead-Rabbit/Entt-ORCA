#include "ORCAEntry.h"

#define USE_ECS_SYSTEM_LOOP true

#if USE_ECS_SYSTEM_LOOP
#include "ecs/component/Components.h"
#include "ecs/system/SysUnitNavigation.h"
#include "ecs/system/SysDebugDraw.h"
#else
#include "gameLogic/OriginBlockSimulate.h"
#endif

AORCAEntry* AORCAEntry::_inst = nullptr;

void AORCAEntry::BeginPlay()
{
	AActor::BeginPlay();

#if USE_ECS_SYSTEM_LOOP
	// ECS 构建
	// 构建 world
	m_ECSWorld = MakeShared<ecs::ECSWorld>();
	auto sysNavigation = m_ECSWorld->createAndRegisterSystem<ecs::SysUnitNavigation>();
	m_ECSWorld->createAndRegisterSystem<ecs::SysDebugDraw>();
	
	ecsBlockSim = new ecs::ECSBlockSimulate(this);
	ecsBlockSim->SetupScenario();

	sysNavigation->kdTree_ = ecsBlockSim->ecsKDTree_;
	
	m_ECSWorld->Start();
	
	// // 测试 entt 增加 entity
	// auto reg = GetReg();
	// const auto entity = reg->create();
	// const auto entity2 = reg->create();
	//
	// ecs::ComUnit & pComUnit = reg->assign<ecs::ComUnit>(entity);
	// pComUnit.num = 1;
	// ecs::ComTransform & pComAssignTrans = reg->assign<ecs::ComTransform>(entity);
	// pComAssignTrans.location = FVector(100, 20, 0);
	//
	// ecs::ComUnit & pComUnit2 = reg->assign<ecs::ComUnit>(entity2);
	// pComUnit2.num = 2;
	// ecs::ComTransform & pComAssignTrans2 = reg->assign<ecs::ComTransform>(entity2);
	// pComAssignTrans2.location = FVector(0, 20, 0);

#else
	// Origin ORCA 构建
	originBlockSim = new RVO::OriginBlockSimulate();
	originBlockSim->Start(this);

#endif
}

void AORCAEntry::ManualTick(float DeltaTime)
{
#if USE_ECS_SYSTEM_LOOP
	if (m_ECSWorld)
		m_ECSWorld->Update(DeltaTime);
#else
	originBlockSim->Update();
#endif
	
}

AActor * AORCAEntry::SpawnActor(UClass* Class, FActorSpawnParameters ActorSpawnParameters) const
{
	if (nullptr != Class)
	{
		AActor* Actor = GetWorld()->SpawnActor<AActor>(Class, ActorSpawnParameters);
		if (nullptr != Actor)
		{
			return Actor;
		}

		UE_LOG(LogTemp, Warning, TEXT("[SYW]AMapcWorld::SpawnActor: Cant spawn actor with class : %s"), *Class->GetName());
	}
	
	return nullptr;
}

AActor * AORCAEntry::CreateActor(const FString& Path)
{
	if (Path.IsEmpty())
		return nullptr;

	FString InPath;
	//                 Models/Generals/Caoang/Sample/BP_Caoang
	//								||
	//								\/
	// Blueprint'/Game/Models/Generals/Caoang/Sample/BP_Caoang.BP_Caoang_C'

	TArray<FString> Sp;
	Path.ParseIntoArray(Sp, TEXT("/"), true);
	int Len = Sp.Num();

	TCHAR NewPath[512];
	FCString::Sprintf(NewPath, TEXT("Blueprint'/Game/ORCA/%s.%s_C'"), *Path, *Sp[Len - 1]);
	InPath = FString(NewPath);

	UClass* BPRes = LoadClass<AActor>(NULL, *InPath);
	if (BPRes != nullptr)
	{
		AActor* Ret = SpawnActor(BPRes, FActorSpawnParameters());
		if (nullptr != Ret)
		{
			return Ret;
		}
		else
			UE_LOG(LogTemp, Warning, TEXT("[SYW]AMapcWorld::CreateActor: Cant spawn actor with path : %s"), *InPath);
	}
	else
	{
		UE_LOG(LogTemp, Warning, TEXT("Cant find actor with path : %s"), *InPath);
	}
	return nullptr;
}
