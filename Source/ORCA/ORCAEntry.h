#pragma once
#include "ecs/ECSWorld.h"
#include "gameLogic/ECSBlockSimulate.h"
#include "gameLogic/OriginBlockSimulate.h"

#include "ORCAEntry.generated.h"

UCLASS()
class AORCAEntry : public AActor
{
	GENERATED_BODY()
	
public:
	AORCAEntry() : AActor()
	{
		_inst = this;
	}

	static AORCAEntry* get()
	{
		return _inst;
	}
	
	virtual void BeginPlay() override;
	
	UFUNCTION(BlueprintCallable, Category="EntryPointEvent")
	void ManualTick(float DeltaTime);
	entt::DefaultRegistry * GetReg()
	{
		return m_ECSWorld->GetReg();
	}
	
	AActor * CreateActor(const FString& Path);
	
private:
	static AORCAEntry* _inst;
	
	RVO::OriginBlockSimulate * originBlockSim = nullptr;
	TSharedPtr<ecs::ECSWorld> m_ECSWorld = nullptr;

	ecs::ECSBlockSimulate * ecsBlockSim = nullptr;
	
	AActor * SpawnActor(UClass* Class, FActorSpawnParameters ActorSpawnParameters) const;
};
