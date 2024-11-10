// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "WheeledVehiclePawn.h"
#include "Components/SplineComponent.h"
#include "PIDCarPawn.generated.h"

class CHAOSVEHICLES_API UChaosVehicleMovementComponent;

/**
 * PIDパス移動テスト用車両
 */
UCLASS()
class PIDCAR_API APIDCarPawn : public AWheeledVehiclePawn
{
	GENERATED_BODY()
	
public:
	UPROPERTY(EditAnywhere, Category = "Control")
	FVector PIDGain = FVector(0.6f, 0.15f, 0.3f);

	UPROPERTY(EditAnywhere, Category = "Control")
	float AngularDampCoef = 1.0f;

	UPROPERTY(EditAnywhere, Category = "Control")
	float Throttle = 0.2f;

	UPROPERTY(EditAnywhere, Category = "Control")
	float IntegralMax = 20.0f;

	UPROPERTY(EditAnywhere, Category = "Control")
	AActor* SplineTarget = nullptr;

public:
	APIDCarPawn();
	virtual void BeginPlay() override;
	virtual void Tick(float DeltaTime) override;

private:
	USplineComponent* SplineComponent = nullptr;

	float PreviousError = 0.0f;
	float Integral = 0.0f;

	FVector TargetLocation = FVector(0);
	FVector TargetDirection = FVector(0);

	void ApplyTarget();
	void ApplyRotate(float DeltaTime);
	void ApplyMovement(float DeltaTime);

};
