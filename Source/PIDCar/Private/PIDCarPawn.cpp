// Fill out your copyright notice in the Description page of Project Settings.


#include "PIDCarPawn.h"
#include "ChaosVehicleMovementComponent.h"

//! コンストラクタ
//*****************************************************************************
APIDCarPawn::APIDCarPawn()
{
    PrimaryActorTick.bCanEverTick = true;
}

//! 開始
//*****************************************************************************
void APIDCarPawn::BeginPlay()
{
    Super::BeginPlay();

    if (SplineTarget)
    {
        SplineComponent = SplineTarget->FindComponentByClass<USplineComponent>();
    }
}

//! 更新
//*****************************************************************************
void APIDCarPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    ApplyTarget();

    ApplyRotate(DeltaTime);
    ApplyMovement(DeltaTime);
}

//! ターゲット更新
//*****************************************************************************
void APIDCarPawn::ApplyTarget()
{
    if (!SplineComponent) return;

    // 最近傍点の位置を取得
    const float ClosestInputKey = SplineComponent->FindInputKeyClosestToWorldLocation(GetActorLocation());
    const FVector ClosestPoint = SplineComponent->GetLocationAtSplineInputKey(ClosestInputKey, ESplineCoordinateSpace::World);

    // スプライン上の指定された距離に基づく位置を取得
    const float OffsetDistance = 256;
    const float Distance = SplineComponent->GetDistanceAlongSplineAtSplineInputKey(ClosestInputKey) + OffsetDistance;
    TargetLocation = SplineComponent->GetLocationAtDistanceAlongSpline(Distance, ESplineCoordinateSpace::World);
    TargetDirection = (TargetLocation - GetActorLocation()).GetSafeNormal();
}

//! 回転
//*****************************************************************************
void APIDCarPawn::ApplyRotate(float DeltaTime)
{
    if (FMath::IsNearlyZero(DeltaTime)) return;

    const FVector CurrentDirection = GetMesh()->GetForwardVector().GetSafeNormal();

    // 1. 回転軸を求める
    // クロス積で2つのベクトルの回転軸を計算
    const FVector RotationAxis = FVector::CrossProduct(CurrentDirection, TargetDirection).GetSafeNormal();

    // 2. エラー値を計算
    // ドット積はで角度のコサイン値を計算(-1 - 1)
    const float Dot = FVector::DotProduct(CurrentDirection, TargetDirection);
    // ドット積から角度を計算
    const float AngleDifference = FMath::Acos(Dot);

    // 3. PID計算
    const float Error = AngleDifference;
    Integral += Error * DeltaTime;
    // 積分項の飽和をClampで対策
    Integral = FMath::Clamp(Integral, 0, IntegralMax);

    const float Derivative = (Error - PreviousError) / DeltaTime;
    const float Magnitude = (PIDGain.X * Error) + (PIDGain.Y * Integral) + (PIDGain.Z * Derivative);

    PreviousError = Error;

    // 4. トルクを計算し適応
    // ダンピング効果を追加(現在の角速度を利用)
    const FVector AngularVelocity = GetMesh()->GetPhysicsAngularVelocityInRadians();
    const FVector AngularDamping = -AngularVelocity * AngularDampCoef;

    const FVector Torque = RotationAxis * Magnitude + AngularDamping;

    // Z成分をそのままステアリング値に使用してやる
    const float SteeringValue = ((Torque.Z + UE_PI) / UE_PI) - 1.0f;
    GetVehicleMovement()->SetSteeringInput(SteeringValue);
}

//! 前進
//*****************************************************************************
void APIDCarPawn::ApplyMovement(float DeltaTime)
{
    GetVehicleMovement()->SetThrottleInput(Throttle);
}
