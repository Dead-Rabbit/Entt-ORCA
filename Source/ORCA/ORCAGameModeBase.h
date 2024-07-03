// Copyright Epic Games, Inc. All Rights Reserved.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "ORCAGameModeBase.generated.h"

/**
 * 
 */
UCLASS()
class ORCA_API AORCAGameModeBase : public AGameModeBase
{
	GENERATED_BODY()

	virtual void InitGame(const FString& MapName, const FString& Options, FString& ErrorMessage) override;
};
