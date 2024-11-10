## PIDCar
* PIDでパス移動するChaosVehicleの実装。  
* Splineの最近傍点にPIDでステアリングを行います。  
![ezgif-4-3da53ec1ad](https://github.com/user-attachments/assets/c728bf4e-9461-4882-babd-0372a0cf581d)

## セットアップ方法
* エンジンやホイールのセットアップは実装済みですが、車のアセットは再配布禁止なので別途導入が必要です。  
* スタティックメッシュとアニメーションブループリントを``BP_PIDCar``に設定してください。  
* 本プロジェクトでは実装・検証に https://fab.com/s/ab1a673f30f5 を使用しました。  
![image](https://github.com/user-attachments/assets/9842b60a-843f-44f1-a401-ab53cc10a251)

## プログラム説明
* PIDによるステアリングは``APIDCarPawn::ApplyRotate``に実装しています。
* Iの値はエラー値が蓄積するとどんどん大きくなってしまい、ステアリングが大げさになってしまう場合があります。本プロジェクトではClampで制限を設けて対策を行っています。（アンチワインドアップ）
```cpp
void APIDCarPawn::ApplyRotate() {
    // ...

    // 3. PID計算
    const float Error = AngleDifference;
    Integral += Error * DeltaTime;
    // 積分項の飽和をClampで対策
    Integral = FMath::Clamp(Integral, 0, IntegralMax);

    const float Derivative = (Error - PreviousError) / DeltaTime;
    const float Magnitude = (PIDGain.X * Error) + (PIDGain.Y * Integral) + (PIDGain.Z * Derivative);

    PreviousError = Error;

    // ...
}
```
